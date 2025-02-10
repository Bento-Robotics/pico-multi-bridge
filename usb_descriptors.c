#include "tusb.h"

#define STRING_DESC_MANUFACTURER 1
#define STRING_DESC_PRODUCT 2
#define STRING_DESC_SERIAL 3
#define STRING_DESC_XX 4

#define CONFIG_TOTAL_LEN                                                       \
  (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN +                      \
   CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN)

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------
enum
{
  ITF_NUM_VENDOR = 0,
  ITF_NUM_VENDOR_DATA,
  ITF_NUM_CDC_0,
  ITF_NUM_CDC_0_DATA,
  ITF_NUM_CDC_1,
  ITF_NUM_CDC_1_DATA,
  ITF_NUM_TOTAL
};
#define EPNUM_VENDOR_DATA_IN   0x81
#define EPNUM_VENDOR_DATA_OUT 0x02

#define EPNUM_CDC_0_NOTIF   0x83
#define EPNUM_CDC_0_DATA    0x04

#define EPNUM_CDC_1_NOTIF   0x85
#define EPNUM_CDC_1_DATA    0x06

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute,
    // power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & IN address, EP size
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRING_DESC_XX, EPNUM_VENDOR_DATA_OUT, EPNUM_VENDOR_DATA_IN, 32),

    // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_DATA, 0x80 | EPNUM_CDC_0_DATA, 64),

    // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_DATA, 0x80 | EPNUM_CDC_1_DATA, 64),

  //TODO high speed mode?
  // https://github.com/raspberrypi/tinyusb/blob/pico/examples/device/cdc_dual_ports/src/usb_descriptors.c#L114
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  return desc_configuration;
}

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, // Supported USB standard (2.1)
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, // Endpoint 0 packet size
    .idVendor = 0x1209,                        // Vendor identifier
    .idProduct = 0x2323,                       // Product identifier
    .bcdDevice = 0x0100,                       // Protocol version
    .iManufacturer = STRING_DESC_MANUFACTURER, // Index of manufacturer name string
    .iProduct = STRING_DESC_PRODUCT,     // Index of product name string
    .iSerialNumber = STRING_DESC_SERIAL, // Index of serial number string
    .bNumConfigurations = 0x01           // Number of configurations supported
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},    // 0: is supported language is English (0x0409)
    "snaens",                      // 1: Manufacturer
    "rp2040-can-and-uart-adaptor", // 2: Product
    "123456",                      // 3: Serials, should use chip ID
    "TinyUSB CDC",                 // 4: CDC Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}
