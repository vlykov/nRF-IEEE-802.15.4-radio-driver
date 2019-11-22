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

#include "nrf_error.h"

#ifndef NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN
#define NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN 23
#endif

/**
 * @brief Configuration structure for antenna diversity
 */
typedef struct
{
    uint8_t ant_sel_pin; /* Pin used for antenna selection */
} nrf_802154_ant_div_config_t;

/**
 * @brief Available antennas
 */
typedef enum
{
    NRF_ANT_DIV_ANTENNA_1 = 0,
    NRF_ANT_DIV_ANTENNA_2 = 1
} nrf_802154_ant_div_antenna_t;

#if ENABLE_ANT_DIV
/**
 * @brief Initializes antenna diversity module.
 *
 * If pinout other than default is to be used, @ref nrf_802154_ant_div_set_config
 * should be called before this function.
 *
 * @retval ::NRF_SUCCESS             Antenna diversity module initialized successfuly
 * @retval ::NRF_ERROR_NOT_SUPPORTED Antenna diversity module is not supported
 */
uint32_t nrf_802154_ant_div_init(void);

/**
 * @brief Sets the antenna diversity configuration.
 *
 * Should not be called after @ref nrf_802154_ant_div_init.
 *
 * @param[in] ant_div_config  Pointer to the antenna diversity configuration structure.
 *
 * @retval ::NRF_SUCCESS             Configuration set successfully
 * @retval ::NRF_ERROR_NOT_SUPPORTED Antenna diversity module is not supported
 */
uint32_t nrf_802154_ant_div_config_set(const nrf_802154_ant_div_config_t * p_ant_div_config);

/**
 * @brief Retrieves te antenna diversity configuration.
 *
 * @param[out] ant_div_config  Pointer to configuration structure to be populated.
 *
 * @retval ::NRF_SUCCESS             Configuration retrieved successfully
 * @retval ::NRF_ERROR_NOT_SUPPORTED Antenna diversity module is not supported
 */
uint32_t nrf_802154_ant_div_config_get(nrf_802154_ant_div_config_t * p_ant_div_config);

/**
 * @brief Select an antenna to use.
 *
 * @param[in] antenna  Antenna to be used
 *
 * @retval ::NRF_SUCCESS             Antenna switched successfully
 * @retval ::NRF_ERROR_INVALID_PARAM Invalid antenna passed to the function
 * @retval ::NRF_ERROR_NOT_SUPPORTED Antenna diversity module is not supported
 */
uint32_t nrf_802154_ant_div_antenna_set(nrf_802154_ant_div_antenna_t antenna);

/**
 * @brief Get currently used antenna.
 *
 * @param[out] antenna  Currently used antenna. Not modified if antenna diversity is not supported.
 *
 * @retval ::NRF_SUCCESS             Current antenna retrieved successfuly
 * @retval ::NRF_ERROR_NOT_SUPPORTED Antenna diversity module is not supported
 */
uint32_t nrf_802154_ant_div_antenna_get(nrf_802154_ant_div_antenna_t * p_antenna);

#else // ENABLE_ANT_DIV

static inline uint32_t nrf_802154_ant_div_init(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

static inline uint32_t nrf_802154_ant_div_config_set(
    const nrf_802154_ant_div_config_t * p_ant_div_config)
{
    (void)p_ant_div_config;
    return NRF_ERROR_NOT_SUPPORTED;
}

static inline uint32_t nrf_802154_ant_div_config_get(nrf_802154_ant_div_config_t * p_ant_div_config)
{
    (void)p_ant_div_config;
    return NRF_ERROR_NOT_SUPPORTED;
}

static inline uint32_t nrf_802154_ant_div_antenna_set(nrf_802154_ant_div_antenna_t antenna)
{
    (void)antenna;
    return NRF_ERROR_NOT_SUPPORTED;
}

static inline uint32_t nrf_802154_ant_div_antenna_get(nrf_802154_ant_div_antenna_t * p_antenna)
{
    (void)p_antenna;
    return NRF_ERROR_NOT_SUPPORTED;
}

#endif // ENABLE_ANT_DIV

#endif // NRF_802154_ANT_DIV_H
