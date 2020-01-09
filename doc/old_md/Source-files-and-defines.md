## Source files

The driver supports multiple [radio arbiters](Multiprotocol-support). Depending on the selected arbiter, you must use a different set of files for the driver build. 
* _Core files_ - these files are used regardless of the selected arbiter or implementation of the platform API.
* _Arbiter-dependent files_ 
* _Platform abstraction files_

### Core files
Source files:
 * nrf_802154.c
 * nrf_802154_core.c
 * nrf_802154_core_hooks.c
 * nrf_802154_critical_section.c
 * nrf_802154_debug.c
 * nrf_802154_pib.c
 * nrf_802154_revision.c
 * nrf_802154_rssi.c
 * nrf_802154_rx_buffer.c
 * nrf_802154_timer_coord.c
 * fal/nrf_802154_fal.c
 * fem/nrf_fem_control.c
 * mac_features/nrf_802154_csma_ca.c
 * mac_features/nrf_802154_delayed_trx.c
 * mac_features/nrf_802154_filter.c
 * mac_features/nrf_802154_frame_parser.c
 * mac_features/nrf_802154_precise_ack_timeout.c
 * mac_features/ack_generator/nrf_802154_ack_data.c
 * mac_features/ack_generator/nrf_802154_ack_generator.c
 * mac_features/ack_generator/nrf_802154_enh_ack_generator.c
 * mac_features/ack_generator/nrf_802154_imm_ack_generator.c
 * rsch/nrf_802154_rsch.c
 * rsch/nrf_802154_rsch_crit_sect.c
 * timer_scheduler/nrf_802154_timer_sched.c

Header files:
 * nrf_802154.h
 * nrf_802154_config.h
 * nrf_802154_const.h
 * nrf_802154_core.h
 * nrf_802154_core_hooks.h
 * nrf_802154_critical_section.h
 * nrf_802154_debug.h
 * nrf_802154_notification.h
 * nrf_802154_pib.h
 * nrf_802154_priority_drop.h
 * nrf_802154_procedures_duration.h
 * nrf_802154_request.h
 * nrf_802154_revision.h
 * nrf_802154_rssi.h
 * nrf_802154_rx_buffer.h
 * nrf_802154_timer_coord.h
 * nrf_802154_types.h
 * nrf_802154_utils.h
 * fal/nrf_802154_fal.h
 * fem/nrf_fem_control_api.h
 * fem/nrf_fem_control_config.h
 * mac_features/nrf_802154_ack_timeout.h
 * mac_features/nrf_802154_csma_ca.h
 * mac_features/nrf_802154_delayed_trx.h
 * mac_features/nrf_802154_filter.h
 * mac_features/nrf_802154_frame_parser.h
 * mac_features/ack_generator/nrf_802154_ack_data.h
 * mac_features/ack_generator/nrf_802154_ack_generator.h
 * mac_features/ack_generator/nrf_802154_enh_ack_generator.h
 * mac_features/ack_generator/nrf_802154_imm_ack_generator.h
 * platform/clock/nrf_802154_clock.h
 * platform/hp_timer/nrf_802154_hp_timer.h
 * platform/lp_timer/nrf_802154_lp_timer.h
 * platform/temperature/nrf_802154_temperature.h
 * rsch/nrf_802154_rsch.h
 * rsch/nrf_802154_rsch_crit_sect.h
 * timer_scheduler/nrf_802154_timer_sched.h

Hardware access layer files:
 * external/hal/nrf_clock.h
 * external/hal/nrf_egu.h
 * external/hal/nrf_gpio.h
 * external/hal/nrf_gpiote.h
 * external/hal/nrf_peripherals.h
 * external/hal/nrf_ppi.h
 * external/hal/nrf_rtc.h
 * external/hal/nrf_timer.h
 * hal/nrf_radio.h

Radio arbiter abstraction layer headers:
 * rsch/raal/nrf_raal_api.h
 * rsch/raal/nrf_raal_config.h

### Arbiter-dependent files

#### Single-PHY
 * nrf_802154_notification_direct.c
 * nrf_802154_priority_drop_direct.c
 * nrf_802154_request_direct.c
 * rsch/raal/single_phy/single_phy.c

#### Simulator
 * nrf_802154_notification_direct.c
 * nrf_802154_priority_drop_direct.c
 * nrf_802154_request_direct.c
 * rsch/raal/simulator/nrf_raal_simulator.c

#### SoftDevice
 * nrf_802154_notification_swi.c
 * nrf_802154_priority_drop_swi.c
 * nrf_802154_request_swi.c
 * nrf_802154_swi.c
 * nrf_802154_swi.h
 * rsch/raal/softdevice/nrf_raal_softdevice.c
 * rsch/raal/softdevice/nrf_raal_softdevice.h

### Platform abstraction examples

The application or platform must provide indirect access to some peripherals or functionalities by defining the platform API. Example implementations of each of the platform APIs are provided with the driver source code. You can choose to use one of provided examples, as __only one example of each platform API implementation can be used at a time__. For example, do not use both `nrf_802154_clock_nodrv.c` and `nrf_802154_clock_sdk.c` in one project. Select one of them or provide clock abstraction implementation by any other module used in your project.

#### Clock
 * platform/clock/nrf_802154_clock_nodrv.c - uses the CLOCK peripheral directly.
 * platform/clock/nrf_802154_clock_sdk.c - uses the clock driver from the nRF5 SDK.

#### High Precision Timer
 * platform/hp_timer/nrf_802154_hp_timer.c - uses a TIMER peripheral (shared with simulator of SoftDevice RAALs) as a clock source for the high precision timer module.

#### Low Power Timer
 * platform/lp_timer/nrf_802154_lp_timer_nodrv.c - uses an RTC peripheral as a clock source for the low power timer module.
 * platform/lp_timer/nrf_802154_lp_timer_none.c - dummy driver: select this implementation if none of the optional features that require the timer are enabled.

#### Pseudorandom Number Generator
 * platform/random/nrf_802154_random_stdlib.c - uses the `rand()` function from the standard C library. The implementation of the standard library can make the `rand()` function reentrant or not. By default, the function is not required to be reentrant.

#### Temperature
 * platform/temperature/nrf_802154_temperature_none.c - reports dummy temperature 20° C; does not allow to use temperature RSSI correction by the driver.


## Configuration
Parameters of the driver can be adjusted in the [config file](https://github.com/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/blob/master/src/nrf_802154_config.h).

## Defines

Some features of the driver can be enabled using compile-time definitions. These definitions can be provided for the build through the -D compiler option.


| Define | Description |
| -------- | ----------- |
| **RAAL_SINGLE_PHY** | Enable this option when the Single-PHY arbiter is in use. |
| **RAAL_SIMULATOR**  | Enable this option when the simulated arbiter is in use. |
| **RAAL_SOFTDEVICE**  | Enable this option when the SoftDevice arbiter is in use. |
| **ENABLE_DEBUG_LOG**  | This option provides an array containing events logged by the driver. [See how to decode the logs](Debug-mode). |
| **ENABLE_DEBUG_ASSERT**  | This option disables all IRQ on an assert and enters indefinite loop. It is helpful with the DEBUG_LOG option, because it prevents updating logs from IRQ handlers after the assertion fails. |
| **ENABLE_DEBUG_GPIO**  | This option connects RADIO events with GPIO output. It allows event timing measurements with a logic analyzer. |
| **ENABLE_FEM** | Enable this option when [Front-End Module support](Front-end-module-support) is required. |
| **NRF52840_AAAA** | Enable this option to build code for nRF52840 rev. AAAA. Other revisions may work incorrectly with this flag set. Without chip revision flags, non-optimized portable code is generated. |
| **NRF52840_AABA** | Enable this option to build code for nRF52840 rev. AABA, AACA, or AAC0. Other revisions may work incorrectly with this flag set. Without chip revision flags, non-optimized portable code is generated. |
| **NRF52811_XXAA** | Enable this option to build code for nRF52811. |
