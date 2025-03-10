# TODO:
- README content
- CI / auto-build
- monitor for buffer overflows `Serial::overflow()` (this also clears the buffer overflow flag)
- (use interrupts for data transfer)
- error detection → reset interface (detach, attach) - micro ros agent sometimes needs a reconnection of the serial port to re-initialize
         if (TinyUSBDevice.mounted()) { TinyUSBDevice.detach(); delay(10); TinyUSBDevice.attach(); }


# Sources
- How (Tiny)USB works: [pschatzmann.ch:tinyusb-a-simple-tutorial](https://www.pschatzmann.ch/home/2021/02/19/tinyusb-a-simple-tutorial/), [beyondlogic.org:usbnutshell](https://www.beyondlogic.org/usbnutshell/usb5.shtml)
- TinyUSB Vendor interface with arduino-pico core: [Adafruit_TinyUSB_Arduino:i2c_tiny_usb_adapter](https://github.com/adafruit/Adafruit_TinyUSB_Arduino/tree/master/examples/Vendor/i2c_tiny_usb_adapter)
- Configuring TinyUSB: comparing the `tusb_config.h` and `usb_descriptors.c` of a multitude of projects, especially the Adafruit one
- Multiple TinyUSB CDC interfaces: [Adafruit_TinyUSB_Arduino:cdc_multi.ino](https://github.com/adafruit/Adafruit_TinyUSB_Arduino/blob/master/examples/CDC/cdc_multi/cdc_multi.ino)
- Using 7 serial I/Os on the rp2040: [Stylesoftware/rpi-pico-2040-how-to-use-all-serial-ports](https://github.com/Stylesoftware/rpi-pico-2040-how-to-use-all-serial-ports)
- Quickly queueing UART data qith interrupts to not overflow buffers: [pico-examples:uart_rx/uart_rx_intr.c](https://github.com/raspberrypi/pico-examples/blob/master/pio/uart_rx/uart_rx_intr.c)
- Using the rp2040 as CAN↔USB bridge: [trnila/rp2040-can-mcp2515](https://github.com/trnila/rp2040-can-mcp2515)
<!-- - TODO  I forgot something -->
