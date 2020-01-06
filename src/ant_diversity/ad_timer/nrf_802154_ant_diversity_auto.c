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
#include "nrf_802154_peripherals.h"
#include "nrf_802154_rssi.h"
#include "rsch/nrf_802154_rsch.h"
#include "nrf_802154_trx.h"
#include "../nrf_802154_pib.h"
#include "../nrf_802154_debug.h"
#include "nrf.h"
#include "nrfx.h"

// ANT_DIV_TODO
#define RSSI_SETTLE_TIME_US 16
typedef enum
{
    AD_STATE_DISABLED,  /// Antenna diversity module is disabled and control of the antenna is ceded.
    AD_STATE_SLEEP,     /// Antenna diversity module is in sleeping state - either radio is not in receive state,
                        /// or rssi measurements are finished and timeout or framestart are expected.
    AD_STATE_TOGGLE,    /// Antenna diversity module is toggling the antenna periodically.
    AD_STATE_SETTLE_1,  /// Antenna diversity module is waiting for first RSSI measurement to settle after the frame prestarted event.
    AD_STATE_SETTLE_2   /// Antenna diversity module is waiting for the second RSSI measurement to settle after the first measurement.
} ad_state_t;

static nrf_802154_ant_div_config_t m_ant_div_config = /**< Antenna Diversity configuration. */
{
    .ant_sel_pin = NRF_802154_ANT_DIV_ANT_SEL_DEFAULT_PIN
};

static ad_state_t m_ad_state            = AD_STATE_DISABLED; /// Automatic switcher state machine current state.
static int8_t     m_prev_rssi           = 0;                 /// First measured rssi, stored for comparison with second measurement.
static bool       m_comparison_finished = false;             /// Flag indicating that the algorithm has been performed in time.
                                                             /// If this is set to false during frame reception, the algorithm didn't
                                                             /// have enough time and current antenna has been selected at random.

void nrf_802154_ant_div_config_set(nrf_802154_ant_div_config_t ant_div_config)
{
    m_ant_div_config = ant_div_config;
}

nrf_802154_ant_div_config_t nrf_802154_ant_div_config_get(void)
{
    return m_ant_div_config;
}

// ANT_DIV_TODO
static void ad_timer_rssi_configure()
{                                                    
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->MODE &= (!TIMER_MODE_MODE_Msk);
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->PRESCALER = (unsigned long int) NRF_TIMER_FREQ_500kHz;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->CC[0] = nrf_timer_us_to_ticks(RSSI_SETTLE_TIME_US, NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->PRESCALER);
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_CLEAR = 1UL;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_START = 1UL;

    NVIC_SetPriority(NRF_802154_ANT_DIVERSITY_TIMER_IRQN, 1);
    NVIC_ClearPendingIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    NVIC_EnableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);

}

static void ad_timer_rssi_deconfigure()
{
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENCLR |= TIMER_INTENCLR_COMPARE0_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_STOP = 1UL;
    NVIC_DisableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    __DSB();
    __ISB();
}

static void ad_timer_toggle_configure()
{                                                    
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->MODE &= (!TIMER_MODE_MODE_Msk);
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->PRESCALER = (unsigned long int) NRF_TIMER_FREQ_500kHz;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->CC[0] = nrf_timer_us_to_ticks(nrf_802154_pib_ant_div_toggle_time_get(), NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->PRESCALER);
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_CLEAR = 1UL;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_START = 1UL;

    NVIC_SetPriority(NRF_802154_ANT_DIVERSITY_TIMER_IRQN, 1);
    NVIC_ClearPendingIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    NVIC_EnableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    
}

static void ad_timer_toggle_deconfigure()
{
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENCLR |= TIMER_INTENCLR_COMPARE0_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_STOP = 1UL;
    NVIC_DisableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    __DSB();
    __ISB();
}

static int8_t ad_rssi_measure()
{
    int8_t result = NRF_802154_RSSI_INVALID;
    // This function is supposed to be called after detecting frame prestarted event, but before
    // detecting valid frame address. This means that we're currently in critical section, but the
    // timeslot is not yet extended due to detecting valid frame. To avoid invalid timeslot extension
    // due to blocking rssi measurements, antenna check can be aborted here if timeslot is about to end.
    // Antenna switching takes 200 ns (250 ns with safety margin), while rssi measurement - 250, 
    // which gives total time of 750 ns.
    // 750 ns is less than safety margin, so check for time left different than 0 is sufficient.
    if(!nrf_802154_rsch_timeslot_us_left_get())
    {
        return result;
    }

    nrf_802154_trx_rssi_measure();

    if (nrf_802154_trx_rssi_measure_is_started())
    {
        while (!nrf_802154_trx_rssi_sample_is_available())
        {
            // Intentionally empty: This function is called from a critical section.
            // WFE would not be waken up by a RADIO event.
        }

        uint8_t rssi_sample = nrf_802154_trx_rssi_last_sample_get();

        rssi_sample = nrf_802154_rssi_sample_corrected_get(rssi_sample);

        result = -((int8_t)rssi_sample);
    }
    return result;
}

static void ad_rssi_first_measure()
{
    m_prev_rssi = ad_rssi_measure();
    if( m_prev_rssi == NRF_802154_RSSI_INVALID)
    {
        ad_timer_rssi_deconfigure();
        m_ad_state = AD_STATE_SLEEP;
        return;
    }
    nrf_802154_ant_div_antenna_toggle();
    ad_timer_rssi_configure();
    m_ad_state = AD_STATE_SETTLE_2;

}

static void ad_rssi_second_measure()
{
    int8_t rssi_current = ad_rssi_measure();
    if( rssi_current != NRF_802154_RSSI_INVALID)
    {
        if (rssi_current < m_prev_rssi)
        {
            nrf_802154_ant_div_antenna_toggle();
        }
        m_comparison_finished = true;
    }

    ad_timer_rssi_deconfigure();
    m_ad_state = AD_STATE_SLEEP;
}

void nrf_802154_ant_div_enable_notify()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            m_comparison_finished = false;
            m_ad_state = AD_STATE_SLEEP;
            break;

        case AD_STATE_SLEEP:
        case AD_STATE_TOGGLE:
        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            // Intentionally empty
            break;

        default:
            assert(false);
    }
}

void nrf_802154_ant_div_disable_notify()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            m_comparison_finished = false;
            m_ad_state = AD_STATE_DISABLED;
            break;
        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            m_ad_state = AD_STATE_DISABLED;
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            ad_timer_rssi_deconfigure();
            m_ad_state = AD_STATE_DISABLED;
            break;

        default:
            assert(false);
    }
}

void nrf_802154_ant_div_rx_started_notify()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            m_comparison_finished = false;
            ad_timer_toggle_configure();
            m_ad_state = AD_STATE_TOGGLE;
            break;

        case AD_STATE_TOGGLE:
        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            // ANT_DIV_TODO
            break;

        default:
            assert(false);
    }
}

void nrf_802154_ant_div_rx_aborted_notify()
{
    // ANT_DIV_TODO
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            // Intentionally empty
            break;

        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            ad_timer_rssi_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            m_comparison_finished = false;
            break;

        default:
            assert(false);
    }
}

void nrf_802154_ant_div_preamble_detected_notify()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            // ANT_DIV_TODO
            break;

        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            ad_timer_rssi_configure();
            m_ad_state = AD_STATE_SETTLE_1;
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            // Intentionally empty - multiple prestarted events may be present.
            break;

        default:
            assert(false);
    }
}

bool nrf_802154_ant_div_frame_started_notify()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            return false;
            break;

        case AD_STATE_SLEEP:
            return m_comparison_finished;

        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            return false;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            ad_timer_rssi_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            return false;

        default:
            assert(false);
    }
}

void nrf_802154_ant_div_preamble_timeout_notify()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            // Timeout should never happen outside of RX state, therefore this branch
            // can only be entered if RSSI measurement is already finished.
            if (m_comparison_finished)
            {
                m_comparison_finished = false;
                ad_timer_toggle_configure();
                m_ad_state = AD_STATE_TOGGLE;
            }
            else
            {
                assert(false); // ANT_DIV_TODO
            }
            break;

        case AD_STATE_TOGGLE:
            // ANT_DIV_TODO - assert?
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            ad_timer_rssi_deconfigure();
            m_comparison_finished = false;
            ad_timer_toggle_configure();
            m_ad_state = AD_STATE_TOGGLE;
            break;

        default:
            assert(false);
    }
}

bool nrf_8021514_ant_div_sweep_start()
{
    // Stub
                                                                   
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->MODE &= (!TIMER_MODE_MODE_Msk);
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->PRESCALER = (unsigned long int) NRF_TIMER_FREQ_500kHz;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->CC[0] = nrf_timer_us_to_ticks(nrf_802154_pib_ant_div_toggle_time_get(), NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->PRESCALER);
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_CLEAR = 1UL;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_START = 1UL;

    NVIC_SetPriority(NRF_802154_ANT_DIVERSITY_TIMER_IRQN, 1);
    NVIC_ClearPendingIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    NVIC_EnableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);

    return true;
}

bool nrf_8021514_ant_div_sweep_stop()
{
    bool result = (NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENSET & TIMER_INTENSET_COMPARE0_Msk) >>  TIMER_INTENSET_COMPARE0_Pos;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENCLR |= TIMER_INTENCLR_COMPARE0_Msk;
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->TASKS_STOP = 1UL;
    NVIC_DisableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    __DSB();
    __ISB();
    return result;
}


bool nrf_8021514_ant_div_sweep_is_running()
{
    return (NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->INTENSET & TIMER_INTENSET_COMPARE0_Msk) >>  TIMER_INTENSET_COMPARE0_Pos;;
}

void NRF_802154_ANT_DIVERSITY_TIMER_IRQHANDLER()
{
    switch(m_ad_state)
    {
        case AD_STATE_DISABLED:
        case AD_STATE_SLEEP:
            break;

        case AD_STATE_TOGGLE:
            nrf_802154_ant_div_antenna_toggle(); 
            break;

        case AD_STATE_SETTLE_1:
            ad_rssi_first_measure();
            break;

        case AD_STATE_SETTLE_2:
            ad_rssi_second_measure();
            break;

        default:
            assert(false);
    }
    NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE->EVENTS_COMPARE[0] = 0UL;
}
