# W6100 Low-Level Example

This example demonstrates how to use the lower-level W6100 API (`esp_eth_mac_new_w6100` and `esp_eth_phy_new_w6100`) directly instead of the convenience `w6100_init()` function.

## Why Use the Low-Level API?

The low-level API gives you more control over:

- **SPI bus initialization**: Share the SPI bus with other devices
- **GPIO ISR service**: Manage interrupt handling yourself
- **MAC/PHY configuration**: Fine-tune driver parameters
- **Error handling**: Handle initialization failures at each step
- **Resource management**: Control when resources are allocated/freed

This approach is similar to how other SPI Ethernet drivers in ESP-IDF are used (W5500, DM9051, KSZ8851SNL, etc.) and is useful when integrating with larger systems or when you need maximum flexibility.

## Hardware Setup

### Pin Configuration (ESP32-S3-DevKitC)

| W6100 Pin | ESP32-S3 GPIO |
|-----------|---------------|
| MOSI      | GPIO11        |
| MISO      | GPIO13        |
| SCLK      | GPIO12        |
| CS        | GPIO10        |
| INT       | GPIO14        |
| RST       | GPIO21        |

Modify the `#define` statements at the top of `main.c` to match your hardware.

## Initialization Steps

The low-level initialization involves these steps:

1. **Initialize TCP/IP stack and event loop**
2. **Perform hardware reset** (optional, via RST pin)
3. **Install GPIO ISR service** (for interrupt mode)
4. **Initialize SPI bus** (`spi_bus_initialize`)
5. **Configure SPI device** (`spi_device_interface_config_t`)
6. **Create MAC instance** (`esp_eth_mac_new_w6100`)
7. **Create PHY instance** (`esp_eth_phy_new_w6100`)
8. **Install Ethernet driver** (`esp_eth_driver_install`)
9. **Set MAC address** (`esp_eth_ioctl` with `ETH_CMD_S_MAC_ADDR`)
10. **Attach to network interface** (`esp_netif_attach`)
11. **Start Ethernet** (`esp_eth_start`)

## Cleanup (for completeness)

When tearing down, clean up in reverse order:

```c
esp_eth_stop(eth_handle);
esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip_event_handler);
esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler);
esp_eth_driver_uninstall(eth_handle);
phy->del(phy);
mac->del(mac);
spi_bus_free(W6100_SPI_HOST);
gpio_uninstall_isr_service();
```

## Build and Flash

```bash
# Set target to ESP32-S3
idf.py set-target esp32s3

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash monitor
```

## Comparison with Basic Example

| Aspect | Basic Example | Low-Level Example |
|--------|---------------|-------------------|
| API | `w6100_init()` | `esp_eth_mac_new_w6100()` + `esp_eth_phy_new_w6100()` |
| SPI bus | Managed automatically | You initialize it |
| GPIO ISR | Managed automatically | You install it |
| Code size | Less code | More code |
| Flexibility | Less | More |
| Similar to | N/A | esp-eth-drivers patterns |

## Expected Output

```
I (xxx) w6100_low_level: W6100 Low-Level Ethernet Example
I (xxx) w6100_low_level: Resetting W6100...
I (xxx) w6100_low_level: Initializing SPI bus...
I (xxx) w6100_low_level: Creating W6100 MAC...
I (xxx) w6100.mac: W6100 chip ID verified: 0x6100
I (xxx) w6100.mac: W6100 version: 0x4661
I (xxx) w6100_low_level: Creating W6100 PHY...
I (xxx) w6100_low_level: Installing Ethernet driver...
I (xxx) w6100_low_level: Setting MAC address: xx:xx:xx:xx:xx:xx
I (xxx) w6100_low_level: Starting Ethernet...
I (xxx) w6100_low_level: Waiting for IP address...
I (xxx) w6100_low_level: Ethernet Started
I (xxx) w6100_low_level: Ethernet Link Up
I (xxx) w6100_low_level: Ethernet HW Addr xx:xx:xx:xx:xx:xx
I (xxx) w6100_low_level: Ethernet Got IP Address
I (xxx) w6100_low_level: ~~~~~~~~~~~
I (xxx) w6100_low_level: IP address: xxx.xxx.xxx.xxx
I (xxx) w6100_low_level: Netmask:    xxx.xxx.xxx.xxx
I (xxx) w6100_low_level: Gateway:    xxx.xxx.xxx.xxx
I (xxx) w6100_low_level: ~~~~~~~~~~~
```
