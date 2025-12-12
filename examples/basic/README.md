# W6100 Basic Example

This example demonstrates basic usage of the W6100 Ethernet driver with ESP32-S3.

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

### Connections

1. Connect W6100 module to ESP32-S3 according to the pin table above
2. Connect W6100 to your network via Ethernet cable
3. Ensure W6100 has proper 3.3V power supply

## Build and Flash

```bash
# Set target to ESP32-S3
idf.py set-target esp32s3

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash monitor
```

## Expected Output

```
I (xxx) w6100_example: W6100 Ethernet Example
I (xxx) w6100_example: Resetting W6100...
I (xxx) w6100_example: Initializing SPI bus...
I (xxx) w6100_example: Creating W6100 MAC...
I (xxx) w6100.mac: W6100 chip ID verified: 0x6100
I (xxx) w6100.mac: W6100 version: 0x4661
I (xxx) w6100_example: Creating W6100 PHY...
I (xxx) w6100_example: Installing Ethernet driver...
I (xxx) w6100_example: Starting Ethernet...
I (xxx) w6100_example: Waiting for IP address...
I (xxx) w6100_example: Ethernet Started
I (xxx) w6100_example: Ethernet Link Up
I (xxx) w6100_example: Ethernet HW Addr xx:xx:xx:xx:xx:xx
I (xxx) w6100_example: Ethernet Got IP Address
I (xxx) w6100_example: ~~~~~~~~~~~
I (xxx) w6100_example: IP address: 192.168.1.xxx
I (xxx) w6100_example: Netmask:    255.255.255.0
I (xxx) w6100_example: Gateway:    192.168.1.1
I (xxx) w6100_example: ~~~~~~~~~~~
```

## Troubleshooting

- **No chip ID detected**: Check SPI wiring and CS pin
- **Link stays down**: Check Ethernet cable and network connection
- **No IP address**: Ensure DHCP server is available on your network
