## Introduction

The nRF 802.15.4 radio driver can be configured to enable _Frontend module_ support. When enabled, the radio driver will toggle GPIO pins before and after radio transmission or radio reception to control the Power Amplifier (PA) and/or Low Noise Amplifier (LNA) of the _Frontend module_.

## General description

<p align="center">
  <img height="300" src="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/fem-chart.png" alt="PA/LNA timings">
</p>
<p align="center">
<em>PA/LNA timings with Frontend module support enabled</em>
</p>

The PA and LNA of the _Frontend module_ are controlled by one GPIO pin each, which are activated during radio transmission and radio reception respectively. The pins can be configured to be active low or active high. The radio driver uses a GPIOTE connected with a PPI channel to a timer to assure that the pins are set to active 5 μs before the EVENT_RXREADY signal and 22 μs before the EVENT_TXREADY signal on the RADIO peripheral. The selected time difference allows for sufficient ramp-up time for the amplifiers. The pins are restored to inactive state using a PPI connected to the EVENT_DISABLED event on the radio.

## Enabling Frontend module support

To enable the _Frontend module_ support in the radio driver, you must build it with the `ENABLE_FEM` compile-time option. When _Frontend module_ support is enabled and configured, it has the following peripheral requirements:
 * 2 GPIO pins,
 * 2 GPIOTE channels,
 * 2 PPI channels.

## Configuring Frontend module support

Once the _Frontend module_ support is enabled, it can be configured using an API function, for example:
```c
nrf_802154_fem_control_cfg_t cfg;

memset(&cfg, 0, sizeof(cfg));

cfg.pa_cfg.enable       = 1;
cfg.pa_cfg.active_high  = 1;
cfg.pa_cfg.gpio_pin     = NRF_FEM_CONTROL_DEFAULT_PA_PIN;

cfg.lna_cfg.enable      = 1;
cfg.lna_cfg.active_high = 1;
cfg.lna_cfg.gpio_pin    = NRF_FEM_CONTROL_DEFAULT_LNA_PIN;

cfg.ppi_ch_id_clr       = NRF_FEM_CONTROL_DEFAULT_CLR_PPI_CHANNEL;
cfg.ppi_ch_id_set       = NRF_FEM_CONTROL_DEFAULT_SET_PPI_CHANNEL;
cfg.pa_gpiote_ch_id     = NRF_FEM_CONTROL_DEFAULT_PA_GPIOTE_CHANNEL;
cfg.lna_gpiote_ch_id    = NRF_FEM_CONTROL_DEFAULT_LNA_GPIOTE_CHANNEL;

nrf_802154_fem_control_cfg_set(&cfg);
```

## Limitations during simultaneous operations of the SoftDevice
* Do not use the PPI channels reserved by the SoftDevice. The S140 SoftDevice v6.1.0 reserves PPI channels 17-31. Check definition of the `NRF_SOC_SD_PPI_CHANNELS_SD_ENABLED_MSK` macro in the `nrf_soc.h` file to verify the list of reserved PPI channels.
* The SoftDevice and the 802.15.4 driver must use separate sets of PPI channels, for example (1, 2) and (3, 4).
* The SoftDevice and the 802.15.4 driver must use separate sets of GPIOTE channels, for example 4 and (6, 7).
