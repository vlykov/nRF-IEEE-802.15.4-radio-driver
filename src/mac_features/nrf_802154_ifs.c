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
 *   This file implements Long/Short Interframe Spacing handling procedure for the 802.15.4 driver.
 *
 */

#include "nrf_802154_ifs.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154_request.h"
#include "nrf_802154_pib.h"
#include "mac_features/nrf_802154_frame_parser.h"
#include "timer_scheduler/nrf_802154_timer_sched.h"

typedef struct
{
    uint8_t * p_data;
    bool cca;
} ifs_operation_t;

static uint32_t m_current_timestamp;
static uint8_t m_last_short_address[SHORT_ADDRESS_SIZE];
static uint8_t m_last_extended_address[EXTENDED_ADDRESS_SIZE];
static bool m_is_last_address_extended;
static uint32_t m_last_frame_timestamp;
static uint8_t m_last_frame_length;
static ifs_operation_t m_context;
static nrf_802154_timer_t m_timer;

static void callback_fired(void * p_context)
{
    ifs_operation_t * context = (ifs_operation_t *)p_context;
    nrf_802154_request_transmit(NRF_802154_TERM_802154,
                                REQ_ORIG_IFS,
                                context->p_data,
                                context->cca,
                                true,
                                NULL);
}

/**@brief Checks if the IFS is needed by comparing the addresses of the actual and the last frames. */
static bool is_ifs_needed_by_address(const uint8_t * p_frame)
{
    if (nrf_802154_pib_ifs_address_match_only_get() == false)
    {
        /* We dont't care about the address match. */
        return true;
    }

    bool is_extended;
    const uint8_t * addr = nrf_802154_frame_parser_dst_addr_get(p_frame, &is_extended);
    if (!addr)
    {
        return true;
    }

    if (is_extended == m_is_last_address_extended)
    {
        uint8_t * last_addr = is_extended ? m_last_extended_address : m_last_short_address;
        size_t addr_len     = is_extended ? EXTENDED_ADDRESS_SIZE : SHORT_ADDRESS_SIZE;

        if (!memcmp(addr, last_addr, addr_len))
        {
            return true;
        }
    }

    return false;
}

/**@brief Checks if the IFS is needed by measuring time between the actual and the last frames. */
static bool is_ifs_needed_by_time(const uint8_t * p_frame, bool * is_lifs)
{
    assert(m_current_timestamp > m_last_frame_timestamp);
    uint32_t dt = m_current_timestamp - m_last_frame_timestamp;

    uint16_t ifs_period;
    if (m_last_frame_length > MAX_SIFS_FRAME_SIZE)
    {
        *is_lifs = true;
        ifs_period = nrf_802154_pib_ifs_min_lifs_period_get();
    }
    else
    {
        *is_lifs = false;
        ifs_period = nrf_802154_pib_ifs_min_sifs_period_get();
    }

    if (dt > ifs_period)
    {
        return false;
    }

    return true;
}

bool nrf_802154_ifs_pretransmission_hook(const uint8_t * p_frame, bool cca)
{
    m_current_timestamp = nrf_802154_timer_sched_time_get();

    if (!m_last_frame_length)
    {
        // No frame was transmitted before - skip the routine
        return true;
    }

    if (!is_ifs_needed_by_address(p_frame))
    {
        return true;
    }

    bool is_lifs;
    if (!is_ifs_needed_by_time(p_frame, &is_lifs))
    {
        return true;
    }

    uint32_t dt = is_lifs ? nrf_802154_pib_ifs_min_lifs_period_get() : nrf_802154_pib_ifs_min_sifs_period_get();

    m_context.p_data = (uint8_t *)p_frame;
    m_context.cca = cca;
    m_timer.t0 = m_current_timestamp;
    m_timer.dt = dt;
    m_timer.callback = callback_fired;
    m_timer.p_context = &m_context;

    nrf_802154_timer_sched_add(&m_timer, true);

    return false;
}

void nrf_802154_ifs_transmitted_hook(const uint8_t * p_frame)
{
    m_last_frame_timestamp = nrf_802154_timer_sched_time_get();

    const uint8_t * addr = nrf_802154_frame_parser_dst_addr_get(p_frame, &m_is_last_address_extended);
    if (!addr)
    {
        m_last_frame_length = 0;
        return;
    }

    if (m_is_last_address_extended)
    {
        memcpy(m_last_extended_address, addr, EXTENDED_ADDRESS_SIZE);
    }
    else
    {
        memcpy(m_last_short_address, addr, SHORT_ADDRESS_SIZE);
    }

    m_last_frame_length = p_frame[0];
    assert(m_last_frame_length < 128);
}

bool nrf_802154_ifs_abort_hook(nrf_802154_term_t term_lvl, req_originator_t req_orig)
{
    bool result = true;
    if (req_orig == REQ_ORIG_IFS)
    {
        // Ignore if self-request.
    }
    else if (nrf_802154_timer_sched_is_running(&m_timer))
    {
        if (term_lvl >= NRF_802154_TERM_802154)
        {
            ifs_operation_t * op = (ifs_operation_t *)m_timer.p_context;
            bool was_running;
            nrf_802154_notify_transmit_failed(op->p_data, NRF_802154_TX_ERROR_ABORTED);
            nrf_802154_timer_sched_remove(&m_timer, &was_running);
        }
        else
        {
            result = false;
        }
    }

    return result;
}
