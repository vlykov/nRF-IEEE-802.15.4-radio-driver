## Driver integrations

The nRF IEEE 802.15.4 radio driver is integrated into several radio stacks using the IEEE 802.15.4 protocol.

### OpenThread

![OpenThread](https://raw.githubusercontent.com/openthread/openthread/master/doc/images/openthread_logo.png)

The nRF IEEE 802.15.4 radio driver has been successfully integrated into OpenThread. The stack implements the Thread protocol.

[The radio platform API](https://github.com/openthread/openthread/blob/master/include/openthread/platform/radio.h) defined by the stack has been ported to use the nRF IEEE 802.15.4 driver. For reference, look at the thin layer that matches two API sets called [radio.c](https://github.com/openthread/openthread/blob/master/examples/platforms/nrf52840/radio.c).

OpenThread is built on autotools build system. Check [Makefile.am](https://github.com/openthread/openthread/blob/master/examples/platforms/nrf52840/Makefile.am) to see which files of the nRF IEEE 802.15.4 radio driver have been added to the compilation process.

### Zigbee

![Zigbee](https://www.zigbee.org/wp-content/uploads/2017/12/zb_logo-a_color_rgb-1024x246.png)

The nRF IEEE 802.15.4 radio driver serves as a PHY layer for [Nordic Semiconductor Zigbee stack](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/Zigbee). The driver provides the dynamic multi-protocol feature for the Zigbee project.

### Zephyr

![Zephyr](https://www.zephyrproject.org/wp-content/uploads/sites/38/2017/08/zephyr-logo.png)

The nRF IEEE 802.15.4 radio driver is a part of the Zephyr project. It provides the 802.15.4 functionality to SoCs of the nRF52 family.

A copy of the driver source code is included in the [Zephyr repository](https://github.com/zephyrproject-rtos/hal_nordic/tree/master/drivers/nrf_radio_802154). It provides [a glue layer](https://github.com/zephyrproject-rtos/zephyr/blob/master/drivers/ieee802154/ieee802154_nrf5.c) that ports the driver to Zephyr 802.15.4 API.

Zephyr is built with the CMake build system. See [CMakeLists.txt](https://github.com/zephyrproject-rtos/zephyr/blob/master/ext/hal/nordic/drivers/nrf_radio_802154/CMakeLists.txt) to check how the driver was added to the compilation process.

You can try the following Zephyr examples with the nRF 802.15.4 driver in the OS:
* [Socket Echo Server](https://docs.zephyrproject.org/latest/samples/net/sockets/echo_server/README.html)
* [Socket Echo Client](https://docs.zephyrproject.org/latest/samples/net/sockets/echo_client/README.html)

These examples can be built with OpenThread as the protocol stack or without OpenThread.
