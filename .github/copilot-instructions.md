# W6100 ESP-IDF Component

## Project Overview
This is an ESP-IDF component for the WIZnet W6100 hardwired TCP/IP Ethernet controller with IPv4/IPv6 dual-stack support.

## Architecture
- SPI-based communication with the W6100 chip
- Integration with ESP-NETIF for seamless network stack integration
- Socket-level API abstraction

## File Structure
- `include/` - Public headers for component users
- `src/` - Implementation files
- `Kconfig` - Component configuration options
- `CMakeLists.txt` - Build configuration

## Coding Guidelines
- Follow ESP-IDF coding style (4-space indentation, snake_case)
- Use ESP-IDF logging macros (ESP_LOGI, ESP_LOGE, etc.)
- All public APIs should be prefixed with `w6100_`
- Error handling should use `esp_err_t` return codes

## Hardware Interface
- SPI clock: up to 80 MHz
- Required pins: MOSI, MISO, SCLK, CS, INT, RST

## References
- [W6100 Datasheet](https://docs.wiznet.io/Product/iEthernet/W6100/datasheet)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
