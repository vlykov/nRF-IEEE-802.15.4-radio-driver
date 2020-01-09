## Multiprotocol support

The dynamic multiprotocol feature allows RADIO peripheral sharing between the 802.15.4 driver and other PHY protocol drivers. It may be used to run several radio protocols simultaneously without the time-expensive uninitialization and initialization in-between the switching. Switching between protocols requires only a reinitialization of the radio peripheral, since protocols may operate on different frequencies, modulations, etc. Therefore, the time of switching is much shorter than in the switched multiprotocol method (when you need to completely shut down one protocol before enabling another).

The driver defines the generic API of _Radio Arbiter_ which grants or denies access to the RADIO peripheral. The nRF IEEE 802.15.4 driver is arbiter-agnostic - it can cooperate with any arbiter through the Radio Arbiter Abstraction Layer (RAAL).

Currently, three arbiter abstractions are implemented:

| Arbiter | Description |
| ------- | ----------- |
| **Single - PHY** | Used when only the 802.15.4 protocol is using the RADIO peripheral. |
| **SoftDevice** | Implementation of SoftDevice's Timeslot API client. It allows concurrent access to the radio by SoftDevice internal protocols (mostly Bluetooth Low Energy) and the 802.15.4 driver. |
| **Simulator** | Simulated arbiter that simulates concurrent access of Bluetooth Low Energy and 802.15.4 to the RADIO peripheral. |

### Single PHY arbiter

The single PHY arbiter should be used when the IEEE 802.15.4 protocol is the only one that uses the RADIO peripheral. Using this arbiter, you can create, for example, Thread-only or Zigbee-only applications.

This arbiter always grants access to the RADIO peripheral, and it does not revoke timeslots.

### SoftDevice arbiter

> Supported versions of SoftDevice: S140 5.0.0-2.alpha and 6.0.0-6.alpha

In the dynamic multiprotocol solution, radio hardware is time-sliced between all protocols. Each radio protocol (for example, BLE or 802.15.4) requests a timeslot prior to any radio operation. This solution allows for keeping established connections in a few protocols at the same time. Transmitting and receiving data does not break connections from any of the radio protocols used, and therefore both connections on 802.15.4 based protocol and BLE may be maintained without disconnecting the other one. This method requires concurrent (time-multiplex) radio access.

The figure below shows how IEEE 802.15.4 based protocol and BLE protocols operate in dynamic multiprotocol mode.

<a href="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/dynamic_multiprotocol.png" target="_blank"><img src="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/dynamic_multiprotocol.png" /></a>
(Click on image to enlarge)
***

Time-multiplexed radio access is achieved by utilizing a radio arbiter. The nRF IEEE 802.15.4 radio driver manages its radio transactions through radio arbiter and requests timeslots prior to any radio activity.

The figure below presents the internal interactions between radio arbiter, radio driver, and application.

<a href="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/dynamic_multiprotocol_requests.png" target="_blank"><img src="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/dynamic_multiprotocol_requests.png" /></a>
(Click on image to enlarge)
***

The SoftDevice (Nordic’s BLE stack) includes a Radio Event Manager (REM) that manages radio arbitration and exposes a [Timeslot API](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.s140.api.v5.0.0%2Fgroup___n_r_f___s_o_c___f_u_n_c_t_i_o_n_s.html&anchor=ga55a52eb4d0491cb7105de6a13eb2b11b) for any other radio driver.

Switching between BLE and IEEE 802.15.4 based protocol is done automatically in the background of the currently running application and therefore is transparent for a user. A user writes the Bluetooth part of a multiprotocol application as if it was a Bluetooth-only application, and the IEEE 802.15.4 part of the multiprotocol application as if it was an IEEE 802.15.4 based-only application.

In this solution, BLE always has priority over Thread. Because of the BLE protocol nature (TDMA), without any external interference, the Bluetooth packet error rate should be 0%. Note that in receiving mode, some radio packets may be lost due to BLE activity. Dropped packets are common in wireless networks, and IEEE 802.15.4 based protocol are usually resiliant to that. To mitigate the number of packets lost on IEEE 802.15.4 based protocol, prolong BLE timing parameters such as advertising interval or connection interval.

This arbiter depends on SoC events from the SoftDevice. They have to be supplied to the driver using the `nrf_raal_softdevice_soc_evt_handler` function. For more information about reading SoC events, refer to the SVC call `sd_evt_get`.

#### Implementation specifics

The SoftDevice RAAL client tries to constantly allocate timeslots with length of 6400 us. The time has been chosen to be longer than the longest single transaction on IEEE 802.15.4 protocol, and also to be a multiple of 200 us. The second requirement results from the smallest possible extension length, which is 200 us.

The SoftDevice uses the Memory Watch Unit to protect access to the RADIO peripheral outside of granted timeslot. Any access to the peripheral in this time causes a HardFault error. To prevent this, without adding large safety margin time (needed to ensure that there are no outgoing nRF IEEE 802.15.4 requests), the SoftDevice RAAL client configures the nRF IEEE 802.15.4 driver to use software interrupts for communication with the application. Because of the implemented approach, the switching time between two protocols could be minimized.

### Simulator

The simulator arbiter has been created to easily simulate Bluetooth LE multiconcurrency. The arbiter implements the behavior of the SoftDevice and blocks access to the RADIO peripheral during simulated Bluetooth LE timeslot, using MWU.

### Handling high-frequency clock

When the RADIO peripheral is used, the Radio Scheduler module makes sure that the high-frequency clock (HFCLK) is running before starting the radio procedures.

Note that the 802.15.4 driver uses the nrf_drv_clock driver to request or release the high-frequency clock (HFCLK). Any other radio stacks or modules (like NFC) should also use nrf_drv_clock to ensure HFCLK is not accidently stopped within the radio activity of the nRF IEEE 802.15.4 radio driver.

### Adding a new Radio Arbiter

To add a new Radio Arbiter, implement all functions defined in [nrf_raal_api.h](https://github.com/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/blob/master/src/raal/nrf_raal_api.h). A new define, for example, RAAL_NEW_ARBITER, should also be introduced to make the compilation process easier.
