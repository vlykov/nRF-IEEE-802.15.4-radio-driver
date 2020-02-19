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
 * @file
 *   This file implements the 802.15.4 antenna diversity module.
 *
 */
#define NRF_802154_MODULE_ID NRF_802154_MODULE_ID_ANT_DIVERSITY

#include "nrf_802154_ant_diversity.h"

#include <assert.h>

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "nrf_802154_debug.h"
#include "nrf_802154_peripherals.h"
#include "nrf_802154_pib.h"
#include "nrf_802154_rssi.h"
#include "nrf_802154_trx.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "rsch/nrf_802154_rsch.h"

#define ANT_DIV_TIMER NRF_802154_ANT_DIVERSITY_TIMER_INSTANCE
#define ANT_DIV_PPI   NRF_802154_PPI_TIMER_COMPARE_TO_ANTENNA_TOGGLE

typedef enum
{
    AD_STATE_DISABLED,                                      /// Antenna diversity module is disabled and control of the antenna is ceded.
    AD_STATE_SLEEP,                                         /// Antenna diversity module is in sleeping state - either radio is not in receive state,
    /// or rssi measurements are finished and timeout or framestart are expected.
    AD_STATE_TOGGLE,                                        /// Antenna diversity module is toggling the antenna periodically while waiting for preamble.
    AD_STATE_SETTLE_1,                                      /// Antenna diversity module is waiting for first RSSI measurement to settle after the frame prestarted event.
    AD_STATE_SETTLE_2                                       /// Antenna diversity module is waiting for the second RSSI measurement to settle after the first measurement.
} ad_state_t;

static nrf_802154_ant_diversity_config_t m_ant_div_config = /**< Antenna Diversity configuration. */
{
    .ant_sel_pin = NRF_802154_ANT_DIVERSITY_ANT_SEL_PIN_DEFAULT,
    .toggle_time = NRF_802154_ANT_DIVERSITY_TOGGLE_TIME_DEFAULT,
};

static ad_state_t m_ad_state            = AD_STATE_DISABLED; /// Automatic switcher state machine current state.
static int8_t     m_prev_rssi           = 0;                 /// First measured rssi, stored for comparison with second measurement.
static bool       m_comparison_finished = false;             /// Flag indicating that the algorithm has been performed in time.
/// If this is set to false during frame reception, the algorithm didn't have enough time and current antenna has been selected at random.
static nrf_802154_ant_diversity_antenna_t m_last_selected_antenna =
    NRF_802154_ANT_DIVERSITY_ANTENNA_NONE;                   /// Last antenna successfully used for reception.

static void ad_timer_init()
{
    nrf_timer_mode_set(ANT_DIV_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(ANT_DIV_TIMER, NRF_TIMER_BIT_WIDTH_8);
    nrf_timer_frequency_set(ANT_DIV_TIMER, NRF_TIMER_FREQ_1MHz);
}

#if ANT_DIVERSITY_PPI
static void ad_ppi_and_gpiote_init()
{
    nrf_gpiote_task_configure(NRF_802154_ANT_DIVERSITY_GPIOTE_CHANNEL,
                              m_ant_div_config.ant_sel_pin,
                              (nrf_gpiote_polarity_t)GPIOTE_CONFIG_POLARITY_Toggle,
                              (nrf_gpiote_outinit_t)(nrf_802154_ant_diversity_antenna_get() ?
                                                     NRF_GPIOTE_INITIAL_VALUE_HIGH :
                                                     NRF_GPIOTE_INITIAL_VALUE_LOW));

    nrf_ppi_channel_endpoint_setup(ANT_DIV_PPI,
                                   (uint32_t)nrf_timer_event_address_get(
                                       ANT_DIV_TIMER,
                                       NRF_TIMER_EVENT_COMPARE0),
                                   (uint32_t)nrf_gpiote_task_addr_get(
                                       NRF_802154_ANT_DIVERSITY_GPIOTE_TASK));
}

#endif // ANT_DIVERSITY_PPI

void nrf_802154_ant_diversity_init(void)
{
    ad_timer_init();
#if ANT_DIVERSITY_PPI
    ad_ppi_and_gpiote_init();
#endif // ANT_DIVERSITY_PPI

    nrf_gpio_cfg_output(m_ant_div_config.ant_sel_pin);
    // Configure the pin as a watcher - connect input buffer to the pin.
    // This allows for reading the actual pin state instead of the desired pin value.
    nrf_gpio_cfg_watcher(m_ant_div_config.ant_sel_pin);
}

bool nrf_802154_ant_diversity_antenna_set(nrf_802154_ant_diversity_antenna_t antenna)
{
    bool status = true;

    if ((NRF_802154_ANT_DIVERSITY_ANTENNA_1 == antenna) ||
        (NRF_802154_ANT_DIVERSITY_ANTENNA_2 == antenna))
    {
        nrf_gpio_pin_write(m_ant_div_config.ant_sel_pin, antenna);
    }
    else
    {
        status = false;
    }

    return status;
}

nrf_802154_ant_diversity_antenna_t nrf_802154_ant_diversity_antenna_get(void)
{
    return nrf_gpio_pin_read(m_ant_div_config.ant_sel_pin);
}

nrf_802154_ant_diversity_antenna_t nrf_802154_ant_diversity_last_rx_best_antenna_get(void)
{
    return m_last_selected_antenna;
}

/**
 * @brief Switches the antenna currently in use.
 *
 * @note This function has no effect while antenna diversity module is currently
 * toggling antenna in PPI variant - that is:
 *  - Antenna diversity is in PPI variant and auto mode is enabled
 *  - rx is enabled
 *  - no PPDU is currently being received
 */
static void nrf_802154_ant_diversity_antenna_toggle()
{
    nrf_gpio_pin_toggle(m_ant_div_config.ant_sel_pin);
}

void nrf_802154_ant_diversity_config_set(nrf_802154_ant_diversity_config_t ant_diversity_config)
{
    m_ant_div_config = ant_diversity_config;
}

nrf_802154_ant_diversity_config_t nrf_802154_ant_diversity_config_get(void)
{
    return m_ant_div_config;
}

static void ad_timer_rssi_configure()
{
    // Anomaly 78: CLEAR instead of STOP, SHUTDOWN triggered manually in IRQHandler.
    nrf_timer_shorts_enable(ANT_DIV_TIMER,
                            NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
    nrf_timer_cc_write(ANT_DIV_TIMER,
                       NRF_TIMER_CC_CHANNEL0,
                       nrf_timer_us_to_ticks(RSSI_SETTLE_TIME_US, NRF_TIMER_FREQ_1MHz));

    nrf_timer_int_enable(ANT_DIV_TIMER, NRF_TIMER_INT_COMPARE0_MASK);

    NVIC_SetPriority(NRF_802154_ANT_DIVERSITY_TIMER_IRQN, 1);
    NVIC_ClearPendingIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    NVIC_EnableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);

    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_START);
}

static void ad_timer_rssi_deconfigure()
{
    // Anomaly 78: CLEAR instead of STOP, SHUTDOWN triggered manually in IRQHandler.
    nrf_timer_shorts_disable(ANT_DIV_TIMER,
                             NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
    nrf_timer_int_disable(ANT_DIV_TIMER, NRF_TIMER_INT_COMPARE0_MASK);

    // Anomaly 78: use SHUTDOWN instead of STOP.
    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_SHUTDOWN);

    NVIC_DisableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    __DSB();
    __ISB();
}

static void ad_timer_toggle_configure()
{
    nrf_timer_shorts_enable(ANT_DIV_TIMER,
                            NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

    nrf_timer_cc_write(ANT_DIV_TIMER,
                       NRF_TIMER_CC_CHANNEL0,
                       nrf_timer_us_to_ticks((uint32_t)m_ant_div_config.toggle_time,
                                             NRF_TIMER_FREQ_1MHz));

#if ANT_DIVERSITY_SW
    nrf_timer_int_enable(ANT_DIV_TIMER, NRF_TIMER_INT_COMPARE0_MASK);

    NVIC_SetPriority(NRF_802154_ANT_DIVERSITY_TIMER_IRQN, 1);
    NVIC_ClearPendingIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    NVIC_EnableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
#elif ANT_DIVERSITY_PPI
    nrf_gpiote_task_enable(NRF_802154_ANT_DIVERSITY_GPIOTE_CHANNEL);
    nrf_ppi_channel_enable(ANT_DIV_PPI);
#else
#error Antenna diversity variant unsupported or not implemented
#endif

    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_START);
}

static void ad_timer_toggle_deconfigure()
{
    nrf_timer_shorts_disable(ANT_DIV_TIMER,
                             NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
#if ANT_DIVERSITY_SW
    nrf_timer_int_disable(ANT_DIV_TIMER, NRF_TIMER_INT_COMPARE0_MASK);
    NVIC_DisableIRQ(NRF_802154_ANT_DIVERSITY_TIMER_IRQN);
    __DSB();
    __ISB();
#elif ANT_DIVERSITY_PPI
    // Set GPIO output pin value to the same as currently set by GPIOTE module
    // This prevents pin switching after control of the pin is ceded by GPIOTE
    nrf_gpio_pin_write(m_ant_div_config.ant_sel_pin, nrf_802154_ant_diversity_antenna_get());
    nrf_gpiote_task_disable(NRF_802154_ANT_DIVERSITY_GPIOTE_CHANNEL);
    nrf_ppi_channel_disable(ANT_DIV_PPI);
#else
#error Antenna diversity variant unsupported or not implemented
#endif
    // Anomaly 78: use SHUTDOWN instead of STOP.
    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_SHUTDOWN);

}

/**
 * Measure and correct rssi value.
 * RSSI needs to be settled already after enabling RX or switching antenna,
 * and this function must be called from critical section.
 *
 * @return Corrected measured RSSI value or NRF_802154_RSSI_INVALID if measurement failed.
 */
static int8_t ad_rssi_measure()
{
    int8_t result = NRF_802154_RSSI_INVALID;

    // This function is supposed to be called after detecting frame prestarted event, but before
    // detecting valid frame address. This means that we're currently in critical section, but the
    // timeslot is not yet extended due to detecting valid frame. To avoid invalid timeslot extension
    // due to blocking rssi measurements, antenna check can be aborted here if timeslot is about to end.
    // Antenna switching takes 200 ns (250 ns with safety margin), while rssi measurement - 250,
    // which gives total time of 750 ns.
    // 750 ns is less than safety margin, so timeslot us left different than 0 is sufficient.
    if (!nrf_802154_rsch_timeslot_us_left_get())
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
    // Anomaly 78: SHUTDOWN has to be triggered manually here, as no short to SHUTDOWN is present
    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_SHUTDOWN);

    m_prev_rssi = ad_rssi_measure();
    // If timeslot has ended, switch to sleep state, rx will not take place.
    if ( m_prev_rssi == NRF_802154_RSSI_INVALID)
    {
        ad_timer_rssi_deconfigure();
        m_ad_state = AD_STATE_SLEEP;
        return;
    }
    nrf_802154_ant_diversity_antenna_toggle();
    ad_timer_rssi_configure();
    m_ad_state = AD_STATE_SETTLE_2;

}

static void ad_rssi_second_measure()
{
    // Anomaly 78: SHUTDOWN has to be triggered manually here, as no short to SHUTDOWN is present
    nrf_timer_task_trigger(ANT_DIV_TIMER, NRF_TIMER_TASK_SHUTDOWN);

    int8_t rssi_current = ad_rssi_measure();

    if ( rssi_current != NRF_802154_RSSI_INVALID)
    {
        if (rssi_current < m_prev_rssi)
        {
            nrf_802154_ant_diversity_antenna_toggle();
        }
        m_comparison_finished = true;
    }

    // Sleep state is entered regardless of the measurement result.
    // m_comparison_finished flag is used to indicate whether RSSI was measured successfuly.
    ad_timer_rssi_deconfigure();
    m_ad_state = AD_STATE_SLEEP;
}

void nrf_802154_ant_diversity_enable_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
            m_comparison_finished = false;
            m_ad_state            = AD_STATE_SLEEP;
            break;

        case AD_STATE_SLEEP:
        case AD_STATE_TOGGLE:
        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            // Intentionally empty
            break;

        default:
            assert(false);
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

void nrf_802154_ant_diversity_disable_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            m_comparison_finished = false;
            m_ad_state            = AD_STATE_DISABLED;
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
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

void nrf_802154_ant_diversity_rx_started_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
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
            assert(false);
            break;

        default:
            assert(false);
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

void nrf_802154_ant_diversity_rx_aborted_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            // Intentionally empty
            m_comparison_finished = false;
            break;

        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            ad_timer_rssi_deconfigure();
            m_ad_state            = AD_STATE_SLEEP;
            m_comparison_finished = false;
            break;

        default:
            assert(false);
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

void nrf_802154_ant_diversity_preamble_detected_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
            // Intentionally empty
            break;

        case AD_STATE_SLEEP:
            // Intentionally empty - can be called after RSSI measurement but before framestart.
            break;

        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            ad_timer_rssi_configure();
            m_ad_state = AD_STATE_SETTLE_1;
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            // Intentionally empty - additional prestarted events are ignored until timeout.
            break;

        default:
            assert(false);
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

bool nrf_802154_ant_diversity_frame_started_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);
    bool result = false;

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
            result = false;
            break;

        case AD_STATE_SLEEP:
            result = m_comparison_finished;
            break;

        case AD_STATE_TOGGLE:
            ad_timer_toggle_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            result     = false;
            break;

        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            ad_timer_rssi_deconfigure();
            m_ad_state = AD_STATE_SLEEP;
            result     = false;
            break;

        default:
            assert(false);
            break;
    }

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
    return result;
}

void nrf_802154_ant_diversity_frame_received_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
            m_last_selected_antenna = NRF_802154_ANT_DIVERSITY_ANTENNA_NONE;
            break;

        case AD_STATE_SLEEP:
            if (m_comparison_finished)
            {
                m_last_selected_antenna = nrf_802154_ant_diversity_antenna_get();
            }
            else
            {
                m_last_selected_antenna = NRF_802154_ANT_DIVERSITY_ANTENNA_NONE;
            }
            break;

        case AD_STATE_TOGGLE:
        case AD_STATE_SETTLE_1:
        case AD_STATE_SETTLE_2:
            assert(false);
            break;

        default:
            assert(false);
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

void nrf_802154_ant_diversity_preamble_timeout_notify()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
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
                assert(false);
            }
            break;

        case AD_STATE_TOGGLE:
            // Framestart timeout timer should not be configured without notifying
            // antenna_diversity module of prestarted event and switching to AD_STATE_SETTLE_1
            assert(false);
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
            break;
    }
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

void NRF_802154_ANT_DIVERSITY_TIMER_IRQHANDLER()
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    switch (m_ad_state)
    {
        case AD_STATE_DISABLED:
        case AD_STATE_SLEEP:
            // Intentionally empty
            break;

        case AD_STATE_TOGGLE:
            nrf_802154_ant_diversity_antenna_toggle();
            break;

        case AD_STATE_SETTLE_1:
            ad_rssi_first_measure();
            break;

        case AD_STATE_SETTLE_2:
            ad_rssi_second_measure();
            break;

        default:
            assert(false);
            break;
    }

    nrf_timer_event_clear(ANT_DIV_TIMER, NRF_TIMER_EVENT_COMPARE0);
    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}
