/* Copyright (c) 2019, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *   This file implements the 802.15.4 antenna diversity module.
 *
 */
#include <assert.h>

#include "nrf_802154_ant_diversity.h"
#include "nrf_gpio.h"

void nrf_802154_ant_div_init(void)
{
    nrf_802154_ant_div_config_t cfg = nrf_802154_ant_div_config_get();

    nrf_gpio_cfg_output(cfg.ant_sel_pin);
}

bool nrf_802154_ant_div_antenna_set(nrf_802154_ant_div_antenna_t antenna)
{
    bool status                     = true;
    nrf_802154_ant_div_config_t cfg = nrf_802154_ant_div_config_get();

    if ((NRF_802154_ANT_DIV_ANTENNA_1 == antenna) || (NRF_802154_ANT_DIV_ANTENNA_2 == antenna))
    {
        nrf_gpio_pin_write(cfg.ant_sel_pin, antenna);
    }
    else
    {
        status = false;
    }

    return status;
}

nrf_802154_ant_div_antenna_t nrf_802154_ant_div_antenna_get(void)
{
    nrf_802154_ant_div_config_t cfg = nrf_802154_ant_div_config_get();

    return nrf_gpio_pin_out_read(cfg.ant_sel_pin);
}

void nrf_802154_ant_div_antenna_toggle()
{
    nrf_802154_ant_div_config_t cfg = nrf_802154_ant_div_config_get();
    
    nrf_gpio_pin_toggle(cfg.ant_sel_pin);
}
