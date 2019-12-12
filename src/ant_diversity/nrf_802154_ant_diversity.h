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
 * @brief 802.15.4 antenna diversity module.
 */

#ifndef NRF_802154_ANT_DIV_H
#define NRF_802154_ANT_DIV_H

#include <stdint.h>

#include "nrf_802154_types.h"
#include "nrf_802154_ant_diversity_config.h"

#ifndef NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN
#define NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN 23
#endif

/**
 * @brief Initializes antenna diversity module.
 *
 * If pinout other than default is to be used, @ref nrf_802154_ant_div_set_config
 * should be called before this function.
 */
void nrf_802154_ant_div_init(void);

/**
 * @brief Selects an antenna to use.
 *
 * @param[in] antenna  Antenna to be used.
 *
 * @retval true  Antenna switched successfully.
 * @retval false Invalid antenna passed to the function.
 */
bool nrf_802154_ant_div_antenna_set(nrf_802154_ant_div_antenna_t antenna);

/**
 * @brief Gets currently used antenna.
 *
 * @return Currently used antenna.
 */
nrf_802154_ant_div_antenna_t nrf_802154_ant_div_antenna_get(void);

/**
 * @brief Switches the antenna currently in use.
 */
void nrf_802154_ant_div_antenna_toggle();

/**
 * @brief Sets the antenna diversity configuration.
 *
 * Should not be called after @ref nrf_802154_ant_div_init.
 * @note Implementation of this function is delegated to specific ant_diversity variant.
 *
 * @param[in] ant_div_config  Antenna diversity configuration structure.
 */
void nrf_802154_ant_div_config_set(nrf_802154_ant_div_config_t ant_div_config);

/**
 * @brief Retrieves the antenna diversity configuration.
 *
 * @note Implementation of this function is delegated to specific ant_diversity variant.
 * 
 * @return Current antenna diversity module configuration.
 */
nrf_802154_ant_div_config_t nrf_802154_ant_div_config_get(void);

/**
 * @brief Starts toggling the antenna periodically.
 * 
 * @note Implementation depends on the antenna diversity variant used.
 * 
 * @retval true  Sweep started successfully.
 * @retval false Error while starting sweep.
 */
bool nrf_8021514_ant_div_sweep_start();

/**
 * @brief Terminates the periodic antenna toggling.
 * 
 * @note Implementation depends on the antenna diversity variant used.
 * 
 * @retval true  Sweep terminated successfully.
 * @retval false Error while terminating sweep.
 */
bool nrf_8021514_ant_div_sweep_stop();

/**
 * @brief Check whether the antenna is currently toggling periodically.
 * 
 * @note Implementation depends on the antenna diversity variant used.
 * 
 * @retval true  Antenna is currently toggling.
 * @retval false Antenna is not currently toggling.
 */
bool nrf_8021514_ant_div_sweep_is_running();

#endif // NRF_802154_ANT_DIV_H
