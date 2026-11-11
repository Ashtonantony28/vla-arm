/* USB descriptors: one CDC-ACM function (host link, spec/protocol.md "Transport").
 *
 * VID/PID: 0x1209 / 0x0001 is the pid.codes *test* PID. It must be replaced by an allocated PID
 * (pid.codes gives them to open hardware projects) before boards are handed out. */
#include <string.h>

#include "stm32g4xx.h"
#include "tusb.h"

#define USB_VID 0x1209u
#define USB_PID 0x0001u
#define USB_BCD_DEVICE 0x0100u /* = ARMFW_FW_VERSION */

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const tusb_desc_device_t k_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    /* IAD-style composite class codes, as TinyUSB's CDC examples use */
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD_DEVICE,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const uint8_t k_config[CONFIG_TOTAL_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

static const char *const k_strings[] = {
    NULL, /* 0: language, handled below */
    "vla-arm",
    "VLA arm controller",
    NULL, /* 3: serial from the 96-bit unique ID */
    "Host link",
};

uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&k_device; }

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return k_config;
}

static uint16_t s_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t n = 0;
    if (index == 0) {
        s_str[1] = 0x0409; /* English (US) */
        n = 1;
    } else if (index == 3) {
        const uint32_t *uid = (const uint32_t *)UID_BASE;
        static const char hex[] = "0123456789ABCDEF";
        for (int w = 0; w < 3; w++)
            for (int b = 7; b >= 0; b--) s_str[1 + n++] = (uint16_t)hex[(uid[w] >> (4 * b)) & 0xFu];
    } else if (index < sizeof k_strings / sizeof k_strings[0] && k_strings[index]) {
        const char *s = k_strings[index];
        while (s[n] && n < 31) {
            s_str[1 + n] = (uint16_t)(uint8_t)s[n];
            n++;
        }
    } else {
        return NULL;
    }
    s_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * n + 2u));
    return s_str;
}
