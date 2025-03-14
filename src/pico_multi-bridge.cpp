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

#include "Adafruit_TinyUSB.h"
#include "Adafruit_USBD_CDC.h"
#include "Bento_USBD_CAN.h"
#include "Adafruit_USBD_Device.h"
#include "class/vendor/vendor_device.h"
#include "Adafruit_NeoPixel.h"
#include "gs_usb.h"
#include "pico/util/queue.h"
extern "C" {
#include "can2040.h"
}

/* This sketch implements multiple CDC interfaces and
 * a gs_usb CAN adapter using can2040 and Adafruit_TinyUSB

 * Reference:
   - https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c
   - https://github.com/trnila/rp2040-can-mcp2515

 * Requirement:
     The max number of CDC ports (CFG_TUD_CDC) has to be changed to at least 3.
      add the build_flag "-DCFG_TUD_CDC=3" in platformio.ini

 * How to test sketch:
    Compile and flash this sketch on your rp2040 and wire up a can transceiver, it should enumerated as
    ID 1209:2323 Generic bytewerk.org candleLight

 * Run "minicom --device /dev/ttyACM1" and/or "minicom --device /dev/ttyACM2"
    Now connect a jumper between the serial RX and TX pins.
    Whatever you type will show up in the RX you wired it into.
    Try typing some stuff!

 * Intall "can-utils" and wire up another CAN node (we need the ACK signal)
    run "sudo ip link set can0 up type can bitrate 500000" to bring up the CAN bus
    run "candump can0 -x" and "cangen can0 -L8"
    watch as the CAN messages fly past!

 * Status and activity LEDs - 2 colors (RX/TX)
    only use one Pin per UART:                    o ← GPIO {0V, 3.3V, Hi-Z}        0V  3.3V Hi-Z
                                  3.3V──100Ω──⯈⊢₁─┴─⯈⊢₂──100Ω──0V             LED1 ON  OFF  OFF
    Hi-Z is acheived by setting pinMode(x, INPUT);                            LED2 OFF ON   OFF
     switch beween 0V and 3.3V when data is recveived/transmitted
     switch to Hi-Z after ~10ms of inactivity => looks like it blinks when data is flowing
 */

// make sure enough USB CDCs are enabled
#if  CFG_TUD_CDC < 3
#error "CFG_TUD_CDC must be at least 3, change in platformio.ini"
#endif

// Utility macros for periodically doing stuff in a non-blocking loop.
// From: https://gist.github.com/EleotleCram/0893191b09043aa91585a1850c52c6d6
#define MILLISECONDS * 1
#define SECONDS * 1000 MILLISECONDS
#define EVERY(N) for (static uint32_t _lasttime;                   \
  (uint32_t)((uint32_t)millis() - _lasttime) >= (N); _lasttime += (N))

// Utility to display what PIO state machines are already allocated.
// Helps when figuring out what library is causing PIO crashes
#define print_pio_use(ser)                                  \
    for (int i = 0; i < 4; i++)                             \
      ser.print(pio_sm_is_claimed(pio0, i), BIN);           \
    ser.print(",");                                         \
    for (int i = 0; i < 4; i++)                             \
      ser.print(pio_sm_is_claimed(pio1, i), BIN);           \
    ser.println("");


static can2040 cbus;
#define PIN_LED_STRIP  2
#define PIN_LED_ACT   16

Bento_USBD_CAN can_usb(&cbus);
Adafruit_USBD_CDC USBSer1; // Builtin USB serial active by default
// Adafruit_USBD_CDC USBSer2;
Adafruit_USBD_CDC USBSer2;
// SerialPIO Serial3(8,9);
Adafruit_NeoPixel* strip{nullptr}; // defer initialization until later

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow() {
  static uint pixelCycle = 0;

  for(uint i=0; i < strip->numPixels(); i++) {
    strip->setPixelColor(i, wheel((3*i + pixelCycle) & 255)); //  Update delay time  
  }
  strip->show();                             //  Update strip to match
  pixelCycle++;                             //  Advance current cycle
  if(pixelCycle >= 256)
    pixelCycle = 0;                         //  Loop the cycle back to the begining
}

static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
  can_usb.handle_can2040_message(cd, notify, msg);
}

static void
PIOx_IRQHandler(void)
{
  can2040_pio_irq_handler(&cbus);
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // do a little panicy blink 
  while (!TinyUSBDevice.mounted()) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);

  /*NOTE for reasons beyond me, the can-usb interface
   * needs to be started *before* all usb CDCs.
   */
  // => kill Serial (Adafruit_TinyUSB starts this automatically)
  SerialTinyUSB.end();

  // Setup canbus
  uint32_t pio_num = 0;
  can2040_setup(&cbus, pio_num);
  can2040_callback_config(&cbus, can2040_cb);

  // Enable irqs for canbus
  irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
  irq_set_priority(PIO0_IRQ_0, 1);
  irq_set_enabled(PIO0_IRQ_0, true);
  pio_claim_sm_mask(pio0, 0b1111);
  // can2040 does not claim it's PIO block
  // this means other libraries will try to override it

  // init can usb with gpio_rx, gpio_tx, bitrate
  // canbus needs to be initialized before
  can_usb.begin(3, 4, 500000);

  // initialize main Serial interface
  SerialTinyUSB.begin(115200);

  strip = new Adafruit_NeoPixel(25, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
  strip->setBrightness(40);

  // // initialize auxiliary CDC and Serial interfaces
  USBSer1.begin(115200);
  pinMode(PIN_LED_ACT, INPUT); // Hi-Z
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(115200);

  USBSer2.begin(115200);
  //pinMode(PIN_LED_ACT, INPUT); // Hi-Z
  // Serial3.setTX(8);
  // Serial3.setRX(9);
  Serial2.begin(115200);

  // if already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  //NOTE: only use this if you want to make sure all interfaces are in use
  // // wait for all auxiliary ports to come online
  // while (!USBSer1 || !USBSer3) {
  //   // local hardware ports seem to immediately initialize, so leave them out
  //   if (!USBSer1) Serial.println("Waiting for USBSer1");
  //   if (!USBSer3) Serial.println("Waiting for USBSer3");
  //   delay(1000);
  // }

  // it seems we need to dispense a message for the gs_usb driver to understand what's going on
  delay(100);
  struct gs_host_frame frame;

  frame.echo_id = -1;
  frame.flags = frame.channel = 0;
  frame.can_dlc = 1;
  frame.data[0] = 42;

  tud_vendor_write(&frame, sizeof(frame));

}

static const pin_size_t NOPIN = 0xff; // Use in constructor to disable LED output

void loop() {

  // Passthrough data between SerialA and SerialB
  auto serial_passthrough = [](Stream &SerialA, Stream &SerialB, pin_size_t activity_leds_pin = NOPIN) -> void {
    if (SerialA.available()) {
      SerialB.write(SerialA.read());
      if (activity_leds_pin != NOPIN) {
        pinMode(activity_leds_pin, OUTPUT);
        digitalWrite(activity_leds_pin, HIGH);
      }
    }
    if (SerialB.available()) {
      SerialA.write(SerialB.read());
      if (activity_leds_pin != NOPIN) {
        pinMode(activity_leds_pin, OUTPUT);
        digitalWrite(activity_leds_pin, LOW);
      }
    }
  };


  serial_passthrough(USBSer1, Serial1, PIN_LED_ACT);
  serial_passthrough(USBSer2, Serial2, PIN_LED_ACT);

  // extends led on duration long enough for all data rates to be visible to humans
  EVERY(10 MILLISECONDS) {
    pinMode(PIN_LED_ACT, INPUT);
  }

  EVERY(1 SECONDS) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // print_pio_use(SerialTinyUSB);
  }

  // CAN RX
  if (tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
    struct gs_host_frame frame;

    if(queue_try_remove(&can_usb.rx_buf, &frame)) {
      tud_vendor_write(&frame, sizeof(frame));
      tud_vendor_write_flush();
    }
  }

  // CAN TX
  if (tud_vendor_available()) {
    gs_host_frame frame;
    uint32_t count = tud_vendor_read(&frame, sizeof(frame));
    assert(count == sizeof(frame));

    size_t hdr_size = 6;
    struct can2040_msg tx;
    tx.id = frame.can_id;
    tx.dlc = (uint32_t) frame.can_dlc;
    memcpy(tx.data, frame.data, frame.can_dlc);
    can2040_transmit(&cbus, &tx);
  }

  tud_task(); // seemingly has no effect?

  EVERY(10 MILLISECONDS) {
    rainbow();
  }
}

// void loop1() {
// }

//extern "C" {
// callback from tinyusb
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request)
{
  return can_usb.handleControlTransfer(rhport, stage, request);
}
//}
