/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Sam 'snaens' Pelz for Bento Robotics, adapted from Adafruit_USBD_I2C example
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BENTO_USBD_CAN_H_
#define BENTO_USBD_CAN_H_

#include "Adafruit_TinyUSB.h"
#include "pico/util/queue.h"
#include "gs_usb.h"
extern "C" {
#include "can2040.h"
}

class Bento_USBD_CAN: public Adafruit_USBD_Interface {
public:
  Bento_USBD_CAN(can2040* cbus);
  bool begin(pin_size_t gpio_rx, pin_size_t gpio_tx, uint32_t bitrate = 500000, size_t rx_bufsize = 4);

  bool handleControlTransfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request);
  void handle_can2040_message(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
  void spin_once();

  // from Adafruit_USBD_Interface
  virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t* buf, uint16_t bufsize);

  queue_t rx_buf;
private:
struct usb_control_out_t {
  uint8_t bRequest;
  void *buffer;
  uint16_t wLength;
};

  bool usb_handle_control_out(uint8_t req);

  can2040* _cbus;
};

#endif
