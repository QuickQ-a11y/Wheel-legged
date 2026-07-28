#include "driver_usb.h"

#include "usbd_cdc_if.h"

static driver_usb_receive_callback_t usbReceiveCallback;
static driver_usb_transmit_callback_t usbTransmitCallback;
static driver_usb_transmit_callback_t usbDisconnectCallback;

void Driver_USB_Init(driver_usb_receive_callback_t receiveCallback,
                     driver_usb_transmit_callback_t transmitCallback,
                     driver_usb_transmit_callback_t disconnectCallback)
{
    usbReceiveCallback = receiveCallback;
    usbTransmitCallback = transmitCallback;
    usbDisconnectCallback = disconnectCallback;
}

uint8_t Driver_USB_Send(uint8_t *data, uint16_t length)
{
    return CDC_Transmit_HS(data, length);
}

void Driver_USB_Receive(const uint8_t *data, uint32_t length)
{
    usbReceiveCallback(data, length);
}

void Driver_USB_TransmitComplete(void)
{
    usbTransmitCallback();
}

void Driver_USB_Disconnect(void)
{
    usbDisconnectCallback();
}
