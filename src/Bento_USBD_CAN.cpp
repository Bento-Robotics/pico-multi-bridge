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

/*
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

#include "Bento_USBD_CAN.h"
#include "gs_usb.h"

Bento_USBD_CAN::Bento_USBD_CAN(can2040* cbus) {
  _cbus = cbus;
  //rx_buf = NULL;
  setStringDescriptor("CAN Interface");
}

uint16_t Bento_USBD_CAN::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t* buf, uint16_t bufsize) {
  uint8_t itfnum = 0;
  uint8_t ep_in = 0;
  uint8_t ep_out = 0;
  (void) itfnum_deprecated;

  // null buffer is used to get the length of descriptor only
  if (buf) {
    itfnum = TinyUSBDevice.allocInterface(1);
    ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
  }

  uint8_t const desc[] = { TUD_VENDOR_DESCRIPTOR(itfnum, _strid, ep_out, ep_in, 32) };
  uint16_t const len = sizeof(desc);

  if (buf) {
    if (bufsize < len) {
      return 0;
    }
    memcpy(buf, desc, len);
  }

  return len;
}

bool Bento_USBD_CAN::begin(pin_size_t gpio_rx, pin_size_t gpio_tx, uint32_t bitrate, size_t rx_bufsize) {
  queue_init(&rx_buf, sizeof(gs_host_frame), rx_bufsize);

  if (!_cbus || !rx_bufsize) return false;

  // needed to identify as a device for i2c_tiny_usb (EZPrototypes VID/PID)
  TinyUSBDevice.setID(0x1209, 0x2323);
  if (!TinyUSBDevice.addInterface(*this)) return false;

  can2040_start(_cbus, F_CPU, bitrate, gpio_rx, gpio_tx);
  return true;
}

void Bento_USBD_CAN::handle_can2040_message(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg){
  switch (notify) {
    case CAN2040_NOTIFY_RX:
    case CAN2040_NOTIFY_TX: // echo messages
      struct gs_host_frame rxf = {0};
      rxf.echo_id = (notify == CAN2040_NOTIFY_TX ? 0 : -1);
      rxf.can_dlc = msg->dlc;
      rxf.flags = 0;
      rxf.channel = 0;

      //if (msg->id & CAN2040_ID_EFF) // extended frame in use, see https://github.com/KevinOConnor/can2040/blob/master/docs/API.md#can2040_msg
      rxf.can_id = msg->id; // however can2040 already formats it correctly
      memcpy(rxf.data, msg->data, rxf.can_dlc);
      queue_try_add(&rx_buf, &rxf);
      break;
    // case CAN2040_NOTIFY_ERROR: //TODO
    //   break;
  }
}
//uint16_t Bento_USBD_I2C::i2c_read(uint8_t addr, uint8_t* buf, uint16_t len, bool stop_bit)

//uint16_t Bento_USBD_I2C::i2c_write(uint8_t addr, uint8_t const* buf, uint16_t len, bool stop_bit){}

uint32_t byte_order = 0;

struct usb_control_out_t {
  uint8_t bRequest;
  void *buffer;
  uint16_t wLength;
};

// extern "C" {
// bool Bento_USBD_CAN::usb_handle_control_out(uint8_t req) {
//   if (req == GS_USB_BREQ_HOST_FORMAT) {
//     return byte_order == 0xbeef;
//   } else if (req == GS_USB_BREQ_MODE) {
//     // mcp2515_set_mode(device_mode.mode ? MCP2515_MODE_NORMAL
//     //                  : MCP2515_MODE_CONFIG);
//     return true;
//   } else if (req == GS_USB_BREQ_BITTIMING) {
//     // mcp2515_write(MCP2515_CNF1,
//     //               (((device_bittiming.sjw - 1) & 0b11U) << 6U) |
//     //               ((device_bittiming.brp / 2 - 1) & 0b111111U));
//     // mcp2515_write(MCP2515_CNF2,
//     //               ((device_bittiming.prop_seg - 1) & 0b111U) |
//     //               (((device_bittiming.phase_seg1 - 1) & 0b111U) << 3) |
//     //               (1U << 7U));
//     // mcp2515_write(MCP2515_CNF3, (((device_bittiming.phase_seg2 - 1) & 0b111U)));
//     return true;
//   }
//   return false;
// }

  gs_device_bittiming device_bittiming;
  gs_device_mode device_mode;
  usb_control_out_t usb_control_out[] = {
    {GS_USB_BREQ_HOST_FORMAT, &byte_order, sizeof(byte_order)},
    {GS_USB_BREQ_BITTIMING, &device_bittiming, sizeof(device_bittiming)},
    {GS_USB_BREQ_MODE, &device_mode, sizeof(device_mode)},
  };

bool Bento_USBD_CAN::handleControlTransfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {

  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR || request->wIndex != 0) {
    return false;
  }

  if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
    for (size_t i = 0; i < sizeof(usb_control_out) / sizeof(*usb_control_out);
    i++) {
      if (usb_control_out[i].bRequest == request->bRequest) {
        if (stage == CONTROL_STAGE_SETUP) {
          if (usb_control_out[i].wLength == request->wLength) {
            return tud_control_xfer(rhport, request, usb_control_out[i].buffer,
                                    usb_control_out[i].wLength);
          }
        } else if (stage == CONTROL_STAGE_DATA) {
          return true; //return usb_handle_control_out(request->bRequest);
        } else if (stage == CONTROL_STAGE_ACK) {
          return true;
        }
      }
    }
  } else if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
    if (request->bRequest == GS_USB_BREQ_DEVICE_CONFIG) {
      if (stage == CONTROL_STAGE_SETUP) {
        struct gs_device_config res;
        res.icount = 0;
        res.sw_version = 18;
        res.hw_version = 11;
        return tud_control_xfer(rhport, request, (void *)&res, sizeof(res));
      } else {
        return true;
      }
    } else if (request->bRequest == GS_USB_BREQ_BT_CONST) {
      if (stage == CONTROL_STAGE_SETUP) {
        const int MCP2515_OSC_FREQ = 8000000;
        struct gs_device_bt_const res = {
          0, MCP2515_OSC_FREQ,
          // tseg1 1..8 (3 bits)
          1, 8,
          // tseg2 1..8 (3 bits)
          1, 8,
          // sjw 0..3 (2 bits)
          4,
          // brp 2..64 with increment of 2 (Tq = 2 * (BRP + 1) / Fosc)
          2, 64, 2};
        return tud_control_xfer(rhport, request, (void *)&res, sizeof(res));
      } else {
        return true;
      }
    }
  }

  return false;


  // uint8_t const cmd = request->bRequest;
  //
  // if ( stage == CONTROL_STAGE_SETUP )
  // {
  //   switch ( cmd )
  //   {
  //     case CMD_ECHO:
  //       // echo
  //       return tud_control_xfer(rhport, request, (void*) &request->wValue, sizeof(request->wValue));
  //
  //     case CMD_GET_FUNC:
  //       // capabilities
  //       return tud_control_xfer(rhport, request, (void*) &_functionality, sizeof(_functionality));
  //
  //     case CMD_SET_DELAY:
  //       if ( request->wValue == 0 )
  //       {
  //         _wire->setClock(115200);
  //       }
  //       else
  //       {
  //         int baudrate = 1000000 / request->wValue;
  //         if ( baudrate > 400000 ) baudrate = 400000;
  //         _wire->setClock(baudrate);
  //       }
  //       return tud_control_status(rhport, request);
  //
  //     case CMD_GET_STATUS:
  //       return tud_control_xfer(rhport, request, (void*) &_state, sizeof(_state));
  //
  //     case CMD_I2C_IO:
  //     case CMD_I2C_IO | CMD_I2C_IO_BEGIN:
  //     case CMD_I2C_IO | CMD_I2C_IO_END:
  //     case CMD_I2C_IO | CMD_I2C_IO_BEGIN | CMD_I2C_IO_END:
  //     {
  //       uint8_t const addr = (uint8_t) request->wIndex;
  //       // uint16_t const flags = request->wValue;
  //       uint16_t const len = tu_min16(request->wLength, _bufsize);
  //       bool const stop_bit = (cmd & CMD_I2C_IO_END) ? true : false;
  //
  //       if (request->bmRequestType_bit.direction == TUSB_DIR_OUT)
  //       {
  //         if (len == 0)
  //         {
  //           // zero write: do it here since there will be no data stage for len = 0
  //           i2c_write(addr, _buf, len, stop_bit);
  //         }
  //         return tud_control_xfer(rhport, request, _buf, len);
  //       }else
  //       {
  //         uint16_t const rd_count = i2c_read(addr, _buf, len, stop_bit);
  //         return tud_control_xfer(rhport, request, rd_count ? _buf : NULL, rd_count);
  //       }
  //     }
  //     break;
  //
  //     default: return true;
  //   }
  // }
  // else if ( stage == CONTROL_STAGE_DATA )
  // {
  //   switch ( cmd )
  //   {
  //     case CMD_I2C_IO:
  //     case CMD_I2C_IO | CMD_I2C_IO_BEGIN:
  //     case CMD_I2C_IO | CMD_I2C_IO_END:
  //     case CMD_I2C_IO | CMD_I2C_IO_BEGIN | CMD_I2C_IO_END:
  //       if (request->bmRequestType_bit.direction == TUSB_DIR_OUT)
  //       {
  //         uint8_t const addr = (uint8_t) request->wIndex;
  //         // uint16_t const flags = request->wValue;
  //         uint16_t const len = tu_min16(request->wLength, _bufsize);
  //         bool const stop_bit = (cmd & CMD_I2C_IO_END) ? true : false;
  //
  //         i2c_write(addr, _buf, len, stop_bit);
  //       }
  //       return true;
  //
  //     default: return true;
  //   }
  // }
  // else
  // {
  //   // CONTROL_STAGE_STATUS
  //   return true;
  // }
}
// }
