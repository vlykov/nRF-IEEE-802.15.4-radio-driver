/* Copyright (c) 2020, Nordic Semiconductor ASA
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

/**
 * @brief Initializes antenna diversity module.
 *
 * If pinout other than default is to be used, @ref nrf_802154_ant_diversity_config_set
 * should be called before this function.
 */
void nrf_802154_ant_diversity_init(void);

/**
 * @brief Selects an antenna to use.
 *
 * @note This function has no effect while antenna diversity module is currently
 * toggling antenna in PPI variant - that is:
 *  - Antenna diversity is in PPI variant and auto mode is enabled
 *  - rx is enabled
 *  - no PPDU is currently being received
 *
 * @param[in] antenna  Antenna to be used.
 *
 * @retval true  Antenna switched successfully.
 * @retval false Invalid antenna passed to the function.
 */
bool nrf_802154_ant_diversity_antenna_set(nrf_802154_ant_diversity_antenna_t antenna);

/**
 * @brief Gets currently used antenna.
 *
 * @return Currently used antenna.
 */
nrf_802154_ant_diversity_antenna_t nrf_802154_ant_diversity_antenna_get(void);

/**
 * @brief Gets which antenna was selected as best for the last reception.
 *
 * @note In three cases @ref NRF_802154_ANT_DIVERSITY_ANTENNA_NONE may be returned:
 *  - No frame was received yet.
 *  - Last frame was received with antenna diversity auto mode disabled.
 *  - RSSI measurements didn't have enough time to finish during last frame reception
 *    and it is unspecified which antenna was selected.
 *
 * @return Antenna selected during last successful reception in automatic mode.
 */
nrf_802154_ant_diversity_antenna_t nrf_802154_ant_diversity_last_rx_best_antenna_get(void);

/**
 * @brief Sets the antenna diversity configuration.
 *
 * Should not be called after @ref nrf_802154_ant_diversity_init.
 *
 * @param[in] ant_diversity_config  Antenna diversity configuration structure.
 */
void nrf_802154_ant_diversity_config_set(nrf_802154_ant_diversity_config_t ant_diversity_config);

/**
 * @brief Retrieves the antenna diversity configuration.
 *
 * @return Current antenna diversity module configuration.
 */
nrf_802154_ant_diversity_config_t nrf_802154_ant_diversity_config_get(void);

/**
 * @brief Notification to be called when antenna diversity auto mode is enabled.
 */
void nrf_802154_ant_diversity_enable_notify();

/**
 * @brief Notification to be called when antenna diversity auto mode is disabled.
 */
void nrf_802154_ant_diversity_disable_notify();

/**
 * @brief Notification to be called when radio rx is started.
 */
void nrf_802154_ant_diversity_rx_started_notify();

/**
 * @brief Notification to be called when radio rx is aborted.
 */
void nrf_802154_ant_diversity_rx_aborted_notify();

/**
 * @brief Notification to be called when preamble is detected.
 */
void nrf_802154_ant_diversity_preamble_detected_notify();

/**
 * @brief Notification to be called when frame start is detected during reception.
 *
 * @retval true  RSSI measurements have finished and currently selected antenna is optimal for reception.
 * @retval false RSSI measurements have not yet finished and currently selected antenna is random.
 */
bool nrf_802154_ant_diversity_frame_started_notify();

/**
 * @brief Notification to be called when frame is received successfuly.
 */
void nrf_802154_ant_diversity_frame_received_notify();

/**
 * @brief Notification to be called when timeout expires after preamble detection.
 */
void nrf_802154_ant_diversity_preamble_timeout_notify();

#endif // NRF_802154_ANT_DIV_H
