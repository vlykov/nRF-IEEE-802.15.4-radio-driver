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
#define NRF_802154_MODULE_ID NRF_802154_MODULE_ID_AD

#include <assert.h>

#include "../nrf_802154_ant_diversity.h"
#include "nrf_timer.h"
#include "../nrf_802154_pib.h"
#include "../nrf_802154_debug.h"
#include "nrf.h"

static nrf_802154_ant_div_config_t m_ant_div_config = /**< Antenna Diversity configuration. */
{
    .ant_sel_pin = NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN,
    .p_timer = NULL
};

void nrf_802154_ant_div_config_set(nrf_802154_ant_div_config_t ant_div_config)
{
    m_ant_div_config = ant_div_config;
}

nrf_802154_ant_div_config_t nrf_802154_ant_div_config_get(void)
{
    return m_ant_div_config;
}

// static void toggle_timer_fired(void * p_context)
// {
//     (void)p_context;
//     nrf_802154_ant_div_antenna_toggle();
//     nrf_8021514_ant_div_sweep_start();
//     // ANT_DIV_TODO
// }

bool nrf_8021514_ant_div_sweep_start()
{
    // Stub
    assert(m_ant_div_config.p_timer != NULL);
    // (void)toggle_timer_fired;
                                                                   
    m_ant_div_config.p_timer->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    m_ant_div_config.p_timer->MODE &= (!TIMER_MODE_MODE_Msk);
    m_ant_div_config.p_timer->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    m_ant_div_config.p_timer->PRESCALER = (unsigned long int) NRF_TIMER_FREQ_500kHz;
    m_ant_div_config.p_timer->CC[0] = nrf_timer_us_to_ticks(nrf_802154_pib_ant_div_toggle_time_get(), m_ant_div_config.p_timer->PRESCALER);
    m_ant_div_config.p_timer->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    m_ant_div_config.p_timer->TASKS_CLEAR = 1UL;
    m_ant_div_config.p_timer->TASKS_START = 1UL;

    NVIC_SetPriority(TIMER4_IRQn, 1);
    NVIC_ClearPendingIRQ(TIMER4_IRQn);
    NVIC_EnableIRQ(TIMER4_IRQn);

    return true;
}

bool nrf_8021514_ant_div_sweep_stop()
{
    bool result = (m_ant_div_config.p_timer->INTENSET & TIMER_INTENSET_COMPARE0_Msk) >>  TIMER_INTENSET_COMPARE0_Pos;
    m_ant_div_config.p_timer->INTENCLR |= TIMER_INTENCLR_COMPARE0_Msk;
    m_ant_div_config.p_timer->TASKS_STOP = 1UL;
    NVIC_DisableIRQ(TIMER4_IRQn);
    __DSB();
    __ISB();
    return result;
}


bool nrf_8021514_ant_div_sweep_is_running()
{
    return (m_ant_div_config.p_timer->INTENSET & TIMER_INTENSET_COMPARE0_Msk) >>  TIMER_INTENSET_COMPARE0_Pos;;
}

void TIMER4_IRQHandler(void)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);
    nrf_802154_ant_div_antenna_toggle();
    // NVIC_ClearPendingIRQ(TIMER4_IRQn);    
    m_ant_div_config.p_timer->EVENTS_COMPARE[0] = 0UL;
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}
