/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 Copyright (c) 2025 Sam 'snaens' Pelz
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Wire.h>
#include "Adafruit_TinyUSB.h"

#include "Adafruit_USBD_CDC.h"
#include "Adafruit_USBD_I2C.h"

/* This sketch demonstrates tinyusb multiple CDC interfaces and
 * a vendor interface to implement i2c-tiny-usb adapter to use with Linux
 *
 * Reference:
 * - https://github.com/torvalds/linux/blob/master/drivers/i2c/busses/i2c-tiny-usb.c
 * - https://github.com/harbaum/I2C-Tiny-USB
 *
 * Requirement:
 * - Install i2c-tools with
 *    sudo apt install i2c-tools
 * - The max number of CDC ports (CFG_TUD_CDC) has to be changed to at least 2.
 *    add the build_flag "-DCFG_TUD_CDC=2" in platformio.ini
 *
 * How to test sketch:
 * - Compile and flash this sketch on your board with an i2c device, it should enumerated as
 *    ID 1c40:0534 EZPrototypes i2c-tiny-usb interface
 *
 * - Run "minicom --device /dev/ttyACM0" and "minicom --device /dev/ttyACM1"
 *    These ports are sent to each other - anything sent from 0 will show up in 1 and vice versa.
 *    Try typing some stuff!
 *
 * - Run "i2cdetect -l" to find our bus ID e.g
 *    i2c-8	i2c       	i2c-tiny-usb at bus 003 device 039	I2C adapter
 *
 * - Run "i2cdetect -y 8" to scan for on-board device (8 is the above bus ID)
 *         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
      00:                         -- -- -- -- -- -- -- --
      10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      70: -- -- -- -- -- -- -- 77

      You can then interact with sensor using following commands:
      i2cget i2cset i2cdump i2ctransfer or using any driver/tools that work on i2c device.
 */

// make sure enough USB CDCs are enabled
#if  CFG_TUD_CDC < 2
#error "CFG_TUD_CDC must be at least 2, change in platformio.ini"
#endif

static uint8_t i2c_buf[800];

#define MyWire    Wire

Adafruit_USBD_I2C i2c_usb(&MyWire);
Adafruit_USBD_CDC USBSer1; // Builtin USB serial active by default

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize main Serial interface
  Serial.begin(115200);

  // while (!Serial) {
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   delay(100);
  // }
  // digitalWrite(LED_BUILTIN, LOW);

  // initialize auxiliary CDC and Serial interfaces
  USBSer1.begin(115200);

  // init i2c usb with buffer and size
  i2c_usb.begin(i2c_buf, sizeof(i2c_buf));

  // if already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // wait for all ports to come online
  while (!Serial || !USBSer1) {
    if (!Serial)  USBSer1.println("Waiting for Serial");
    if (!USBSer1) Serial.println("Waiting for USBSer1");
    delay(1000);
  }

}

void loop() {

  // Passthrough data between SerialA and SerialB
  auto serial_passthrough = [](Stream &SerialA, Stream &SerialB) -> void {
    if (SerialA.available()) {
      SerialB.write(SerialA.read());
    }
    if (SerialB.available()) {
      SerialA.write(SerialB.read());
    }
  };

  serial_passthrough(Serial, USBSer1);

}


// callback from tinyusb
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request)
{
  return i2c_usb.handleControlTransfer(rhport, stage, request);
}

