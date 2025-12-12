# W6100 iperf Example

This example provides an iperf-compatible console application for testing network throughput with the WIZnet W6100 Ethernet controller.

## Hardware Required

- ESP32-S3-DevKitC (or similar ESP32-S3 board)
- WIZnet W6100 module
- Ethernet cable and network with DHCP

### Pin Configuration (ESP32-S3-DevKitC)

| Signal | GPIO |
|--------|------|
| MOSI   | 11   |
| MISO   | 13   |
| SCLK   | 12   |
| CS     | 10   |
| INT    | 14   |
| RST    | 21   |

## Build and Flash

```bash
cd w6100/examples/iperf
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Usage

After flashing, the device will:
1. Initialize the W6100 Ethernet controller
2. Obtain an IP address via DHCP
3. Start a console REPL

### Console Commands

**Get Ethernet info:**
```
w6100> ethernet info
```

**Run as iperf server (receive test):**
```
w6100> iperf -s
```

**Run as iperf client (send test):**
```
w6100> iperf -c <server_ip>
```

**UDP mode:**
```
w6100> iperf -u -s
w6100> iperf -u -c <server_ip>
```

**With custom options:**
```
w6100> iperf -c <server_ip> -t 30 -i 3
```
- `-t 30`: Run for 30 seconds (default: 10)
- `-i 3`: Report every 3 seconds (default: 3)

**Abort running test:**
```
w6100> iperf -a
```

### iperf Command Options

| Option | Description |
|--------|-------------|
| `-s`   | Run in server mode |
| `-c <ip>` | Run in client mode, connect to `<ip>` |
| `-u`   | Use UDP instead of TCP |
| `-p <port>` | Port to use (default: 5001) |
| `-l <len>` | Buffer length |
| `-i <sec>` | Interval between reports |
| `-t <sec>` | Time to run |
| `-b <Mbps>` | Bandwidth limit (Mbps) |
| `-a`   | Abort running iperf |

## Testing with PC

### Install iperf on your PC

**Linux:**
```bash
sudo apt install iperf
```

**macOS:**
```bash
brew install iperf
```

**Windows:**
Download from https://iperf.fr/iperf-download.php

### Test Download Speed (ESP32 → PC)

1. On PC, start server:
   ```bash
   iperf -s
   ```

2. On ESP32 console:
   ```
   w6100> iperf -c <PC_IP>
   ```

### Test Upload Speed (PC → ESP32)

1. On ESP32 console:
   ```
   w6100> iperf -s
   ```

2. On PC:
   ```bash
   iperf -c <ESP32_IP>
   ```

## Expected Performance

The W6100 is a 10 Mbps Ethernet controller. Theoretical maximum:
- TCP: ~9.5 Mbps (due to protocol overhead)
- UDP: ~9.8 Mbps

Actual performance depends on:
- SPI clock speed (this example uses 60 MHz)
- TCP/lwIP configuration
- Network conditions

## Troubleshooting

### Low throughput

1. Check SPI clock speed in `main.c`
2. Verify `sdkconfig.defaults` lwIP settings are applied
3. Try unicore mode (already enabled by default)

### Connection issues

1. Check physical Ethernet connection
2. Verify DHCP server is running on network
3. Use `ethernet info` to check IP address

### Console not responding

1. Ensure correct UART port
2. Check baud rate (default 115200)
