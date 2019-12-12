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


#include "../nrf_802154_ant_diversity.h"
#include "../timer_scheduler/nrf_802154_timer_sched.h"
#include "../nrf_802154_pib.h"
#include "../nrf_802154_debug.h"

static nrf_802154_ant_div_config_t m_ant_div_config = /**< Antenna Diversity configuration. */
{
    .ant_sel_pin = NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN
};

static nrf_802154_timer_t m_toggle_timer;

void nrf_802154_ant_div_config_set(nrf_802154_ant_div_config_t ant_div_config)
{
    m_ant_div_config = ant_div_config;
}

nrf_802154_ant_div_config_t nrf_802154_ant_div_config_get(void)
{
    return m_ant_div_config;
}

static void toggle_timer_fired(void * p_context)
{
    (void)p_context;
    nrf_802154_ant_div_antenna_toggle();
    nrf_8021514_ant_div_sweep_start();
    // ANT_DIV_TODO
}

bool nrf_8021514_ant_div_sweep_start()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);
    m_toggle_timer.t0 = nrf_802154_timer_sched_time_get();
    m_toggle_timer.dt = nrf_802154_pib_ant_div_toggle_time_get();
    m_toggle_timer.callback = toggle_timer_fired;
    m_toggle_timer.p_context = NULL;

    nrf_802154_timer_sched_add(&m_toggle_timer, true);

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
    return true;
}

bool nrf_8021514_ant_div_sweep_stop()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);
    bool was_running;
    nrf_802154_timer_sched_remove(&m_toggle_timer, &was_running);
    return was_running;
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}


bool nrf_8021514_ant_div_sweep_is_running()
{
    return nrf_802154_timer_sched_is_running(&m_toggle_timer);
}