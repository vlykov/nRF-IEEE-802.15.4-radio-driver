# Hardware resources used by the nRF IEEE 802.15.4 radio driver

The radio driver requires some hardware resources to operate correctly. The driver uses several periperals directly. It also uses some peripherals indirectly through platform abstraction API.

## Peripherals used by the driver directly
* RADIO - may be shared with arbitration in multiprotocol configuration
* 2 TIMERs (or 1 TIMER if BCMATCH is enabled - see `nrf_802154_config.h` configuration file)
* 1 EGU - channel 15 (or all channels in multiprotocol configuration)
* 7 PPI channels (or 12 PPI channels in debug configuration)
* 1 PPI group
* 2 GPIOTE channels - if FEM support is enabled

## Functionalities provided by the platform abstraction
* Clock - needed to make sure high frequency clock is enabled during RADIO operations
* Temperature - provides temperature data for RSSI correction
* Timer (low frequency) - used by optional features like CSMA-CA, ACK timeout, or timestamps of received frames
* Timer (high precision) - used by optional features like received frame timestamps

Note that the platform or the application must provide an implementation of each of the driver's platform abstraction APIs. Example implementations are located in the `src/platform` directory. You can select one of the example implementations for each of platform abstraction APIs. You can also implement platform abstraction API in any other module instead of using the provided example.
