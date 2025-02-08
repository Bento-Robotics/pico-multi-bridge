#include "gs_usb.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "tusb.h"
#include <assert.h>
#include "hardware/clocks.h"

#include "RP2040.h"
#include "can2040.h"
// #if defined(PICO_CAN2040)
// #include "can2040.h"
// #else
// #error Invalid board selected
// #endif

struct usb_control_out_t {
  uint8_t bRequest;
  void *buffer;
  uint16_t wLength;
};

enum mcp2515_mode_t {
  MCP2515_MODE_NORMAL,
  MCP2515_MODE_SLEEP,
  MCP2515_MODE_LOOPBACK,
  MCP2515_MODE_LISTENONLY,
  MCP2515_MODE_CONFIG,
};

#define MCP2515_RX_BUFS 2
/* multiple TX buffers are not sending frames in FIFO order */
#define MCP2515_TX_BUFS 1
#define RX_FRAMES_QUEUE_LEN 64

struct gs_host_frame tx[MCP2515_TX_BUFS];

static uint32_t byte_order = 0;
static struct gs_device_bittiming device_bittiming;
static struct gs_device_mode device_mode;
struct usb_control_out_t usb_control_out[] = {
  {GS_USB_BREQ_HOST_FORMAT, &byte_order, sizeof(byte_order)},
  {GS_USB_BREQ_BITTIMING, &device_bittiming, sizeof(device_bittiming)},
  {GS_USB_BREQ_MODE, &device_mode, sizeof(device_mode)},
};

static queue_t rx_frames;
static struct can2040 cbus;

static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  switch (notify) {
    case CAN2040_NOTIFY_RX:
    case CAN2040_NOTIFY_TX: // echo messages
      struct gs_host_frame rxf = {0};
      rxf.echo_id = -1;
      rxf.can_dlc = msg->dlc;
      rxf.flags = 0;
      rxf.channel = 0;

      //if (msg->id & CAN2040_ID_EFF) // extended frame in use, see https://github.com/KevinOConnor/can2040/blob/master/docs/API.md#can2040_msg
      rxf.can_id = msg->id; // however can2040 already formats it correctly
      memcpy(rxf.data, msg->data, rxf.can_dlc);
      queue_try_add(&rx_frames, &rxf);
      break;
    case CAN2040_NOTIFY_ERROR: //TODO
      break;
  }
}

static void PIOx_IRQHandler(void) {
  can2040_pio_irq_handler(&cbus);
}


ssize_t mcp2515_get_free_tx() {
  for (size_t i = 0; i < sizeof(tx) / sizeof(*tx); i++) {
    if (tx[i].echo_id == -1) {
      return i;
    }
  }

  return -1;
}


int main() {
  tusb_init();

  for (size_t i = 0; i < sizeof(tx) / sizeof(*tx); i++) {
    tx[i].echo_id = -1;
  }

  queue_init(&rx_frames, sizeof(struct gs_host_frame), RX_FRAMES_QUEUE_LEN);

  uint32_t pio_num = 0;
  uint32_t sys_clock = 125000000, bitrate = 500000;
  set_sys_clock_hz(sys_clock, true);
  uint32_t gpio_rx = 18, gpio_tx = 19;

  // Setup canbus
  can2040_setup(&cbus, pio_num);
  can2040_callback_config(&cbus, can2040_cb);

  // Enable irqs
  irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
  NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
  NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

  // Start canbus
  can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
  sleep_ms(250);


  for (;;) {
    // wait for empty buffer to send rx frames one by one
    if (tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
      struct gs_host_frame frame;
      if (queue_try_remove(&rx_frames, &frame)) {
        tud_vendor_write(&frame, sizeof(frame));
        tud_vendor_write_flush();
      }
    }

    tud_task();

    if (tud_vendor_available()) {
      ssize_t txn = mcp2515_get_free_tx();
      if (txn >= 0) {
        struct gs_host_frame *frame = &tx[txn];
        uint32_t count = tud_vendor_read(frame, sizeof(*frame));
        assert(count == sizeof(*frame));

        size_t hdr_size = 6;
        struct can2040_msg tx = {0};
        tx.id = frame->can_id;
        tx.dlc = frame->can_dlc;
        memcpy(tx.data, frame->data, frame->can_dlc);
        can2040_transmit(&cbus, &tx);
      }
    }
  }
}

bool usb_handle_control_out(uint8_t req) {
  if (req == GS_USB_BREQ_HOST_FORMAT) {
    return byte_order == 0xbeef;
  } else if (req == GS_USB_BREQ_MODE) {
    // mcp2515_set_mode(device_mode.mode ? MCP2515_MODE_NORMAL
    //                  : MCP2515_MODE_CONFIG);
    return true;
  } else if (req == GS_USB_BREQ_BITTIMING) {
    // mcp2515_write(MCP2515_CNF1,
    //               (((device_bittiming.sjw - 1) & 0b11U) << 6U) |
    //               ((device_bittiming.brp / 2 - 1) & 0b111111U));
    // mcp2515_write(MCP2515_CNF2,
    //               ((device_bittiming.prop_seg - 1) & 0b111U) |
    //               (((device_bittiming.phase_seg1 - 1) & 0b111U) << 3) |
    //               (1U << 7U));
    // mcp2515_write(MCP2515_CNF3, (((device_bittiming.phase_seg2 - 1) & 0b111U)));
    return true;
  }
  return false;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                const tusb_control_request_t *request) {
  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
    request->wIndex != 0) {
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
          return usb_handle_control_out(request->bRequest);
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
}
