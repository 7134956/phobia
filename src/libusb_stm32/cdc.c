/* This file is the part of the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright ©2016 Dmitry Filimonchuk <dmitrystu[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "cdc.h"
#include "hal/hal.h"
#include "inc/usb.h"
#include "inc/usb_cdc.h"
#include "libc.h"
#include "stm32.h"

#define GPIO_USB_DM			XGPIO_DEF4('A', 11, 0, 10)
#define GPIO_USB_DP			XGPIO_DEF4('A', 12, 0, 10)

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x81
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x82
#define CDC_NTF_SZ      0x08

//#define SIGNAL_MODEM  /* uncomment to signal modem capabilities */

#if defined(SIGNAL_MODEM)
#define CDC_PROTOCOL USB_CDC_PROTO_V25TER
#else
#define CDC_PROTOCOL USB_PROTO_NONE
#endif

QueueHandle_t		queue_RX;
QueueHandle_t		queue_TX;

/* Declaration of the report descriptor */
struct cdc_config {
    struct usb_config_descriptor        config;
    struct usb_iad_descriptor           comm_iad;
    struct usb_interface_descriptor     comm;
    struct usb_cdc_header_desc          cdc_hdr;
    struct usb_cdc_call_mgmt_desc       cdc_mgmt;
    struct usb_cdc_acm_desc             cdc_acm;
    struct usb_cdc_union_desc           cdc_union;
    struct usb_endpoint_descriptor      comm_ep;
    struct usb_interface_descriptor     data;
    struct usb_endpoint_descriptor      data_eprx;
    struct usb_endpoint_descriptor      data_eptx;
} __attribute__((packed));

/* Device descriptor */
static const struct usb_device_descriptor device_desc = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = VERSION_BCD(2,0,0),
    .bDeviceClass       = USB_CLASS_IAD,
    .bDeviceSubClass    = USB_SUBCLASS_IAD,
    .bDeviceProtocol    = USB_PROTO_IAD,
    .bMaxPacketSize0    = CDC_EP0_SIZE,
    .idVendor           = 0x0483,
    .idProduct          = 0x5740,
    .bcdDevice          = VERSION_BCD(1,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

/* Device configuration descriptor */
static const struct cdc_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct cdc_config),
        .bNumInterfaces         = 2,
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower              = USB_CFG_POWER_MA(100),
    },
    .comm_iad = {
        .bLength = sizeof(struct usb_iad_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFASEASSOC,
        .bFirstInterface        = 0,
        .bInterfaceCount        = 2,
        .bFunctionClass         = USB_CLASS_CDC,
        .bFunctionSubClass      = USB_CDC_SUBCLASS_ACM,
        .bFunctionProtocol      = CDC_PROTOCOL,
        .iFunction              = NO_DESCRIPTOR,
    },
    .comm = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_CDC,
        .bInterfaceSubClass     = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol     = CDC_PROTOCOL,
        .iInterface             = NO_DESCRIPTOR,
    },
    .cdc_hdr = {
        .bFunctionLength        = sizeof(struct usb_cdc_header_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_HEADER,
        .bcdCDC                 = VERSION_BCD(1,1,0),
    },
    .cdc_mgmt = {
        .bFunctionLength        = sizeof(struct usb_cdc_call_mgmt_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_CALL_MANAGEMENT,
        .bmCapabilities         = 0,
        .bDataInterface         = 1,

    },
    .cdc_acm = {
        .bFunctionLength        = sizeof(struct usb_cdc_acm_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_ACM,
        .bmCapabilities         = 0,
    },
    .cdc_union = {
        .bFunctionLength        = sizeof(struct usb_cdc_union_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_UNION,
        .bMasterInterface0      = 0,
        .bSlaveInterface0       = 1,
    },
    .comm_ep = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_NTF_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = CDC_NTF_SZ,
        .bInterval              = 0xFF,
    },
    .data = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTO_NONE,
        .iInterface             = NO_DESCRIPTOR,
    },
    .data_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_RXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
    .data_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_TXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
};

static const struct usb_string_descriptor lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("STM32 USB stack");
static const struct usb_string_descriptor prod_desc_en  = USB_STRING_DESC("PESC(Speed controller)");
static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
};

usbd_device udev;
uint32_t    ubuf[0x20];

static struct usb_cdc_line_coding cdc_line = {
    .dwDTERate          = 115200,
    .bCharFormat        = USB_CDC_1_STOP_BITS,
    .bParityType        = USB_CDC_NO_PARITY,
    .bDataBits          = 8,
};

static usbd_respond cdc_getdesc (usbd_ctlreq *req, void **address, uint16_t *length) {
    const uint8_t dtype = req->wValue >> 8;
    const uint8_t dnumber = req->wValue & 0xFF;
    const void* desc;
    uint16_t len = 0;
    switch (dtype) {
    case USB_DTYPE_DEVICE:
        desc = &device_desc;
        break;
    case USB_DTYPE_CONFIGURATION:
        desc = &config_desc;
        len = sizeof(config_desc);
        break;
    case USB_DTYPE_STRING:
        if (dnumber < 3) {
            desc = dtable[dnumber];
        } else {
            return usbd_fail;
        }
        break;
    default:
        return usbd_fail;
    }
    if (len == 0) {
        len = ((struct usb_header_descriptor*)desc)->bLength;
    }
    *address = (void*)desc;
    *length = len;
    return usbd_ack;
};


static usbd_respond cdc_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS)
        && req->wIndex == 0 ) {
        switch (req->bRequest) {
        case USB_CDC_SET_CONTROL_LINE_STATE:
            return usbd_ack;
        case USB_CDC_SET_LINE_CODING:
            memcpy( req->data, &cdc_line, sizeof(cdc_line));
            return usbd_ack;
        case USB_CDC_GET_LINE_CODING:
            dev->status.data_ptr = &cdc_line;
            dev->status.data_count = sizeof(cdc_line);
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
    return usbd_fail;
}

int USBCDC_getc()
{
	char		xC;

	xQueueReceive(queue_RX, &xC, portMAX_DELAY);

	return (int) xC;
}

void USBCDC_putc(int c)
{
	char		xC = (char) c;

	GPIO_set_HIGH(GPIO_LED);

	xQueueSendToBack(queue_TX, &xC, portMAX_DELAY);

	GPIO_set_LOW(GPIO_LED);
}

static void cdc_rxonly(usbd_device *dev, uint8_t event, uint8_t ep) {
	BaseType_t		xWoken = pdFALSE;
	uint8_t buf[CDC_DATA_SZ];
	IODEF_TO_USBCDC();
	int cnt = usbd_ep_read(dev, CDC_RXD_EP, &buf, CDC_DATA_SZ);
	for (int N = 0; N < cnt; ++N) {
		xQueueSendToBackFromISR(queue_RX, &buf[N], &xWoken);
	}
	portYIELD_FROM_ISR(xWoken);
}

static void cdc_txonly(usbd_device *dev, uint8_t event, uint8_t ep) {
	BaseType_t		xWoken = pdFALSE;
	uint8_t buf[CDC_DATA_SZ];
	int i = 0;
	char xC;
	while ((i < CDC_DATA_SZ) && (xQueueReceiveFromISR(queue_TX, &xC, &xWoken) == pdTRUE)) {
		buf[i++] = xC;
	}
	usbd_ep_write(dev, CDC_TXD_EP, &buf, i);
	portYIELD_FROM_ISR(xWoken);
}

#if ((CDC_TXD_EP & 0x7F) == (CDC_RXD_EP & 0x7F))
static void cdc_rxtx(usbd_device *dev, uint8_t event, uint8_t ep) {
	if (event == usbd_evt_eptx) {
		cdc_txonly(dev, event, ep);
	} else {
		cdc_rxonly(dev, event, ep);
	}
}
#endif

static usbd_respond cdc_setconf (usbd_device *dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(dev, CDC_NTF_EP);
        usbd_ep_deconfig(dev, CDC_TXD_EP);
        usbd_ep_deconfig(dev, CDC_RXD_EP);
        usbd_reg_endpoint(dev, CDC_RXD_EP, 0);
        usbd_reg_endpoint(dev, CDC_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(dev, CDC_RXD_EP, USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF, CDC_DATA_SZ);
        usbd_ep_config(dev, CDC_TXD_EP, USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF, CDC_DATA_SZ);
//        usbd_ep_config(dev, CDC_RXD_EP, USB_EPTYPE_BULK, CDC_DATA_SZ);
//        usbd_ep_config(dev, CDC_TXD_EP, USB_EPTYPE_BULK, CDC_DATA_SZ);
        usbd_ep_config(dev, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
#if ((CDC_TXD_EP & 0x7F) == (CDC_RXD_EP & 0x7F))
        usbd_reg_endpoint(dev, CDC_RXD_EP, cdc_rxtx);
        usbd_reg_endpoint(dev, CDC_TXD_EP, cdc_rxtx);
#else
        usbd_reg_endpoint(dev, CDC_RXD_EP, cdc_rxonly);
        usbd_reg_endpoint(dev, CDC_TXD_EP, cdc_txonly);
#endif
        usbd_ep_write(dev, CDC_TXD_EP, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

static void cdc_init_usbd(void) {
    usbd_init(&udev, &usbd_hw, CDC_EP0_SIZE, ubuf, sizeof(ubuf));
    usbd_reg_config(&udev, cdc_setconf);
    usbd_reg_control(&udev, cdc_control);
    usbd_reg_descr(&udev, cdc_getdesc);
}

void irq_OTG_FS(void) {
	usbd_poll(&udev);
}

extern QueueHandle_t USART_queue_RX();

//Запустить комуникацию по VCP
void USBCDC_startup() {

	queue_RX = USART_queue_RX();
	queue_TX = xQueueCreate(128, sizeof(char));
	/* Enable USB pins. PA11 and PA12 to AF10 (USB_FS)
	 * */
	GPIO_set_mode_FUNCTION(GPIO_USB_DM);
	GPIO_set_mode_FUNCTION(GPIO_USB_DP);
	// Прерывание USB_FS
	NVIC_SetPriority(OTG_FS_IRQn, 12);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	cdc_init_usbd();
	usbd_enable(&udev, true);
	usbd_connect(&udev, true);
}
