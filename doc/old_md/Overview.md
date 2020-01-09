# nRF 802.15.4 radio driver

The nRF 802.15.4 radio driver implements the 802.15.4 PHY layer on the Nordic Semiconductor nRF52811 and nRF52840 SoCs.

## Architecture

The architecture of the nRF IEEE 802.15.4 radio driver is independent of the OS and the IEEE 802.15.4 MAC layer. It allows to use the driver in any of the stacks based on the IEEE 802.15.4 that implement protocols such as Thread, ZigBee, or RF4CE. The exposed API makes the driver easily pluggable to high-level stacks like OpenThread (for Thread protocol), by creating a thin platform layer between the stack and the nRF IEEE 802.15.4 radio driver.

In addition, the driver was designed to work with multiprotocol applications. The driver allows to share the RADIO peripheral with other PHY protocol drivers, for example, Bluetooth LE. Radio drivers request access to the peripheral from an arbiter. The arbiter grants or denies access. The nRF 802.15.4 driver is arbiter-agnostic - it can cooperate with any arbiter through the Radio Arbiter Abstraction Layer (RAAL), which is a part of the nRF IEEE 802.15.4 driver. The currently implemented arbiters allow to run 802.15.4-based protocol either alone (single-PHY arbiter) or with a SoftDevice that uses the Timeslot API (SoftDevice REM arbiter). The second option allows for dynamic multiprotocol support with Bluetooth LE.

The following block datagram shows the architecture of the driver.

![nRF IEEE 802.15.4 radio driver - block diagram](https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/nRF%20IEEE802_15_4_radio_driver_block_diagram.gif)

For more details on the dynamic multiprotocol solution, see [Multiprotocol support](Multiprotocol-support).

## Features

This driver implements only the __non-beacon mode__ of operation.
It supports the following features:
 * Reception of unicast and broadcast frames (with filtering).
 * Automatic sending of ACK frames.
 * Setting a pending bit in the ACK frame according to pending data for the given destination.
 * Transmission of unicast and broadcast frames.
 * Automatic CCA procedure before transmission.
 * Automatic receiving of ACK frames.
 * Stand-alone CCA procedure.
 * Low power mode (sleep).
 * Energy detection.
 * Promiscuous mode.
 * Continuous carrier transmission (for radio testing).
 * Dynamic multiprotocol support.
 * Front-end modules support.
 * ACK time-out handling.
 * Automatic CSMA-CA procedure during transmission.
 * Providing diagnostic information (for example, CRC errors).

For more details on supported features, see [Features description](Features-description).
