/*
 * SPDX-FileCopyrightText: 2024 DrWilco
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W6100 iperf Example for ESP32-S3-DevKitC
 *
 * Pin Configuration:
 *   MOSI: GPIO11
 *   MISO: GPIO13
 *   SCLK: GPIO12
 *   CS:   GPIO10
 *   INT:  GPIO14
 *   RST:  GPIO21
 *
 * Usage:
 *   1. Build and flash this example
 *   2. Connect via serial terminal
 *   3. Wait for IP address to be assigned
 *   4. Run iperf server on your PC: iperf -s
 *   5. From ESP32 console: iperf -c <PC_IP>
 *
 *   Or run server mode on ESP32:
 *   1. From ESP32 console: iperf -s
 *   2. From PC: iperf -c <ESP32_IP>
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sys/socket.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "argtable3/argtable3.h"
#include "cmd_system.h"
#include "iperf.h"
#include "w6100.h"

static const char *TAG = "w6100_iperf";

// Event group to signal when we have an IP address
static EventGroupHandle_t s_eth_event_group;
#define ETH_GOT_IP_BIT BIT0

// Ethernet handles for info command
static esp_eth_handle_t s_eth_handle = NULL;
static esp_netif_t *s_eth_netif = NULL;

/* "ethernet" command */
static struct {
    struct arg_str *control;
    struct arg_end *end;
} eth_control_args;

static int eth_cmd_control(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&eth_control_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, eth_control_args.end, argv[0]);
        return 1;
    }

    if (!strncmp(eth_control_args.control->sval[0], "info", 4)) {
        uint8_t mac_addr[6];
        esp_netif_ip_info_t ip;
        printf("W6100 Ethernet:\r\n");
        esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        printf("  HW ADDR: " MACSTR "\r\n", MAC2STR(mac_addr));
        esp_netif_get_ip_info(s_eth_netif, &ip);
        printf("  IP:      " IPSTR "\r\n", IP2STR(&ip.ip));
        printf("  NETMASK: " IPSTR "\r\n", IP2STR(&ip.netmask));
        printf("  GATEWAY: " IPSTR "\r\n", IP2STR(&ip.gw));
    }
    return 0;
}

/* "iperf" command */
static struct {
    struct arg_str *ip;
    struct arg_lit *server;
    struct arg_lit *udp;
    struct arg_int *port;
    struct arg_int *length;
    struct arg_int *interval;
    struct arg_int *time;
    struct arg_int *bw_limit;
    struct arg_lit *abort;
    struct arg_end *end;
} iperf_args;

static int eth_cmd_iperf(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&iperf_args);
    iperf_cfg_t cfg;

    if (nerrors != 0) {
        arg_print_errors(stderr, iperf_args.end, argv[0]);
        return 0;
    }

    memset(&cfg, 0, sizeof(cfg));

    // W6100 only supports IPv4
    cfg.type = IPERF_IP_TYPE_IPV4;

    /* iperf -a */
    if (iperf_args.abort->count != 0) {
        iperf_stop();
        return 0;
    }

    if (((iperf_args.ip->count == 0) && (iperf_args.server->count == 0)) ||
            ((iperf_args.ip->count != 0) && (iperf_args.server->count != 0))) {
        ESP_LOGE(__func__, "Must specify either -s (server) or -c <ip> (client)");
        return 0;
    }

    /* iperf -s */
    if (iperf_args.ip->count == 0) {
        cfg.flag |= IPERF_FLAG_SERVER;
    }
    /* iperf -c SERVER_ADDRESS */
    else {
        cfg.destination_ip4 = esp_ip4addr_aton(iperf_args.ip->sval[0]);
        cfg.flag |= IPERF_FLAG_CLIENT;
    }

    if (iperf_args.length->count == 0) {
        cfg.len_send_buf = 0;
    } else {
        cfg.len_send_buf = iperf_args.length->ival[0];
    }

    /* Wait for IP, could block here */
    ESP_LOGI(TAG, "Waiting for IP address...");
    xEventGroupWaitBits(s_eth_event_group, ETH_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    cfg.source_ip4 = INADDR_ANY;

    /* iperf -u */
    if (iperf_args.udp->count == 0) {
        cfg.flag |= IPERF_FLAG_TCP;
    } else {
        cfg.flag |= IPERF_FLAG_UDP;
    }

    /* iperf -p */
    if (iperf_args.port->count == 0) {
        cfg.sport = IPERF_DEFAULT_PORT;
        cfg.dport = IPERF_DEFAULT_PORT;
    } else {
        if (cfg.flag & IPERF_FLAG_SERVER) {
            cfg.sport = iperf_args.port->ival[0];
            cfg.dport = IPERF_DEFAULT_PORT;
        } else {
            cfg.sport = IPERF_DEFAULT_PORT;
            cfg.dport = iperf_args.port->ival[0];
        }
    }

    /* iperf -i */
    if (iperf_args.interval->count == 0) {
        cfg.interval = IPERF_DEFAULT_INTERVAL;
    } else {
        cfg.interval = iperf_args.interval->ival[0];
        if (cfg.interval <= 0) {
            cfg.interval = IPERF_DEFAULT_INTERVAL;
        }
    }

    /* iperf -t */
    if (iperf_args.time->count == 0) {
        cfg.time = IPERF_DEFAULT_TIME;
    } else {
        cfg.time = iperf_args.time->ival[0];
        if (cfg.time <= cfg.interval) {
            cfg.time = cfg.interval;
        }
    }

    /* iperf -b */
    if (iperf_args.bw_limit->count == 0) {
        cfg.bw_lim = IPERF_DEFAULT_NO_BW_LIMIT;
    } else {
        cfg.bw_lim = iperf_args.bw_limit->ival[0];
        if (cfg.bw_lim <= 0) {
            cfg.bw_lim = IPERF_DEFAULT_NO_BW_LIMIT;
        }
    }

    printf("mode=%s-%s sip=" IPSTR ":%" PRIu16 ", dip=%" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32 ":%" PRIu16 ", interval=%" PRIu32 ", time=%" PRIu32 "\r\n",
           cfg.flag & IPERF_FLAG_TCP ? "tcp" : "udp",
           cfg.flag & IPERF_FLAG_SERVER ? "server" : "client",
           (uint16_t) cfg.source_ip4 & 0xFF,
           (uint16_t) (cfg.source_ip4 >> 8) & 0xFF,
           (uint16_t) (cfg.source_ip4 >> 16) & 0xFF,
           (uint16_t) (cfg.source_ip4 >> 24) & 0xFF,
           cfg.sport,
           cfg.destination_ip4 & 0xFF, (cfg.destination_ip4 >> 8) & 0xFF,
           (cfg.destination_ip4 >> 16) & 0xFF, (cfg.destination_ip4 >> 24) & 0xFF, cfg.dport,
           cfg.interval, cfg.time);

    iperf_start(&cfg);
    return 0;
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        xEventGroupClearBits(s_eth_event_group, ETH_GOT_IP_BIT);
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        xEventGroupClearBits(s_eth_event_group, ETH_GOT_IP_BIT);
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP address: " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "Netmask:    " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "Gateway:    " IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);
}

static void register_ethernet_commands(void)
{
    eth_control_args.control = arg_str1(NULL, NULL, "<info>", "Get info of Ethernet");
    eth_control_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
        .command = "ethernet",
        .help = "Ethernet interface info",
        .hint = NULL,
        .func = eth_cmd_control,
        .argtable = &eth_control_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

    iperf_args.ip = arg_str0("c", "client", "<ip>", "run in client mode, connecting to <host>");
    iperf_args.server = arg_lit0("s", "server", "run in server mode");
    iperf_args.udp = arg_lit0("u", "udp", "use UDP rather than TCP");
    iperf_args.port = arg_int0("p", "port", "<port>", "server port to listen on/connect to");
    iperf_args.length = arg_int0("l", "len", "<length>", "set read/write buffer size");
    iperf_args.interval = arg_int0("i", "interval", "<interval>", "seconds between periodic bandwidth reports");
    iperf_args.time = arg_int0("t", "time", "<time>", "time in seconds to transmit for (default 10 secs)");
    iperf_args.bw_limit = arg_int0("b", "bandwidth", "<bandwidth>", "bandwidth to send at in Mbits/sec");
    iperf_args.abort = arg_lit0("a", "abort", "abort running iperf");
    iperf_args.end = arg_end(1);
    const esp_console_cmd_t iperf_cmd = {
        .command = "iperf",
        .help = "iperf command (use -s for server, -c <ip> for client)",
        .hint = NULL,
        .func = &eth_cmd_iperf,
        .argtable = &iperf_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&iperf_cmd));
}

static void initialize_console(void)
{
    /* Initialize VFS & mount console REPL */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "w6100>";
    repl_config.max_cmdline_length = 256;

    /* Register commands */
    esp_console_register_help_command();
    register_system();
    register_ethernet_commands();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
#else
#error Unsupported console type
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void)
{
    ESP_LOGI(TAG, "W6100 iperf Example");

    // Create event group
    s_eth_event_group = xEventGroupCreate();

    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Ethernet netif
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&cfg);

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Configure W6100
    w6100_init_config_t w6100_config = W6100_INIT_CONFIG_DEFAULT();
    
    // ESP32-S3-DevKitC pin configuration
    w6100_config.spi_host = SPI2_HOST;
    w6100_config.mosi_gpio = GPIO_NUM_11;
    w6100_config.miso_gpio = GPIO_NUM_13;
    w6100_config.sclk_gpio = GPIO_NUM_12;
    w6100_config.cs_gpio = GPIO_NUM_10;
    w6100_config.int_gpio = GPIO_NUM_14;
    w6100_config.rst_gpio = GPIO_NUM_21;
    w6100_config.spi_clock_mhz = 60;

    // Create W6100 Ethernet handle
    ESP_LOGI(TAG, "Creating W6100 Ethernet driver...");
    ESP_ERROR_CHECK(w6100_init(&w6100_config, &s_eth_handle));

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));

    // Start Ethernet driver
    ESP_LOGI(TAG, "Starting Ethernet...");
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    // Initialize console with iperf commands
    initialize_console();

    ESP_LOGI(TAG, "Console ready. Use 'help' for available commands.");
    ESP_LOGI(TAG, "Usage examples:");
    ESP_LOGI(TAG, "  Server mode: iperf -s");
    ESP_LOGI(TAG, "  Client mode: iperf -c <server_ip>");
    ESP_LOGI(TAG, "  UDP mode:    iperf -u -s  or  iperf -u -c <server_ip>");
}
