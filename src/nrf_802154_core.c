/* Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
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
 *   This file implements Finite State Machine of nRF 802.15.4 radio driver.
 *
 */

#include "nrf_802154_core.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154.h"
#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "nrf_802154_critical_section.h"
#include "nrf_802154_debug.h"
#include "nrf_802154_notification.h"
#include "nrf_802154_peripherals.h"
#include "nrf_802154_pib.h"
#include "nrf_802154_procedures_duration.h"
#include "nrf_802154_rssi.h"
#include "nrf_802154_rx_buffer.h"
#include "nrf_802154_utils.h"
#include "nrf_802154_timer_coord.h"
#include "nrf_802154_trx.h"
#include "nrf_802154_types.h"
#include "nrf_802154_utils.h"
#include "nrf_egu.h"
#include "nrf_error.h"
#include "nrf_ppi.h"
#include "nrf_radio.h"
#include "nrf_timer.h"
#include "fem/nrf_fem_protocol_api.h"
#include "mac_features/nrf_802154_delayed_trx.h"
#include "mac_features/nrf_802154_filter.h"
#include "mac_features/nrf_802154_frame_parser.h"
#include "mac_features/ack_generator/nrf_802154_ack_data.h"
#include "mac_features/ack_generator/nrf_802154_ack_generator.h"
#include "rsch/nrf_802154_rsch.h"
#include "rsch/nrf_802154_rsch_crit_sect.h"

#include "nrf_802154_core_hooks.h"

#define EGU_EVENT                  NRF_EGU_EVENT_TRIGGERED15
#define EGU_TASK                   NRF_EGU_TASK_TRIGGER15
#define PPI_CHGRP0                 NRF_802154_PPI_CORE_GROUP                     ///< PPI group used to disable self-disabling PPIs
#define PPI_CHGRP0_DIS_TASK        NRF_PPI_TASK_CHG0_DIS                         ///< PPI task used to disable self-disabling PPIs

#define PPI_DISABLED_EGU           NRF_802154_PPI_RADIO_DISABLED_TO_EGU          ///< PPI that connects RADIO DISABLED event with EGU task
#define PPI_EGU_RAMP_UP            NRF_802154_PPI_EGU_TO_RADIO_RAMP_UP           ///< PPI that connects EGU event with RADIO TXEN or RXEN task
#define PPI_EGU_TIMER_START        NRF_802154_PPI_EGU_TO_TIMER_START             ///< PPI that connects EGU event with TIMER START task
#define PPI_CRCERROR_CLEAR         NRF_802154_PPI_RADIO_CRCERROR_TO_TIMER_CLEAR  ///< PPI that connects RADIO CRCERROR event with TIMER CLEAR task
#define PPI_CCAIDLE_FEM            NRF_802154_PPI_RADIO_CCAIDLE_TO_FEM_GPIOTE    ///< PPI that connects RADIO CCAIDLE event with GPIOTE tasks used by FEM
#define PPI_TIMER_TX_ACK           NRF_802154_PPI_TIMER_COMPARE_TO_RADIO_TXEN    ///< PPI that connects TIMER COMPARE event with RADIO TXEN task
#define PPI_CRCOK_DIS_PPI          NRF_802154_PPI_RADIO_CRCOK_TO_PPI_GRP_DISABLE ///< PPI that connects RADIO CRCOK event with task that disables PPI group

#if NRF_802154_DISABLE_BCC_MATCHING
#define PPI_ADDRESS_COUNTER_COUNT  NRF_802154_PPI_RADIO_ADDR_TO_COUNTER_COUNT    ///< PPI that connects RADIO ADDRESS event with TIMER COUNT task
#define PPI_CRCERROR_COUNTER_CLEAR NRF_802154_PPI_RADIO_CRCERROR_COUNTER_CLEAR   ///< PPI that connects RADIO CRCERROR event with TIMER CLEAR task
#endif  // NRF_802154_DISABLE_BCC_MATCHING

#if NRF_802154_DISABLE_BCC_MATCHING
#define SHORT_ADDRESS_BCSTART 0UL
#else // NRF_802154_DISABLE_BCC_MATCHING
#define SHORT_ADDRESS_BCSTART NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK
#endif  // NRF_802154_DISABLE_BCC_MATCHING

/// Value set to SHORTS register when no shorts should be enabled.
#define SHORTS_IDLE             0

/// Value set to SHORTS register for RX operation.
#define SHORTS_RX               (NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK | \
                                 NRF_RADIO_SHORT_END_DISABLE_MASK |       \
                                 SHORT_ADDRESS_BCSTART)

#define SHORTS_RX_FREE_BUFFER   (NRF_RADIO_SHORT_RXREADY_START_MASK)

#define SHORTS_TX_ACK           (NRF_RADIO_SHORT_TXREADY_START_MASK | \
                                 NRF_RADIO_SHORT_PHYEND_DISABLE_MASK)

#define SHORTS_CCA_TX           (NRF_RADIO_SHORT_RXREADY_CCASTART_MASK | \
                                 NRF_RADIO_SHORT_CCABUSY_DISABLE_MASK |  \
                                 NRF_RADIO_SHORT_CCAIDLE_TXEN_MASK |     \
                                 NRF_RADIO_SHORT_TXREADY_START_MASK |    \
                                 NRF_RADIO_SHORT_PHYEND_DISABLE_MASK)

#define SHORTS_TX               (NRF_RADIO_SHORT_TXREADY_START_MASK | \
                                 NRF_RADIO_SHORT_PHYEND_DISABLE_MASK)

#define SHORTS_RX_ACK           (NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK | \
                                 NRF_RADIO_SHORT_END_DISABLE_MASK)

#define SHORTS_ED               (NRF_RADIO_SHORT_READY_EDSTART_MASK)

#define SHORTS_CCA              (NRF_RADIO_SHORT_RXREADY_CCASTART_MASK | \
                                 NRF_RADIO_SHORT_CCABUSY_DISABLE_MASK)

/// Delay before first check of received frame: 24 bits is PHY header and MAC Frame Control field.
#define BCC_INIT                (3 * 8)

/// Duration of single iteration of Energy Detection procedure
#define ED_ITER_DURATION        128U
/// Overhead of hardware preparation for ED procedure (aTurnaroundTime) [number of iterations]
#define ED_ITERS_OVERHEAD       2U

#define CRC_LENGTH              2               ///< Length of CRC in 802.15.4 frames [bytes]
#define CRC_POLYNOMIAL          0x011021        ///< Polynomial used for CRC calculation in 802.15.4 frames

#define MHMU_MASK               0xff000700      ///< Mask of known bytes in ACK packet
#define MHMU_PATTERN            0x00000200      ///< Values of known bytes in ACK packet
#define MHMU_PATTERN_DSN_OFFSET 24              ///< Offset of DSN in MHMU_PATTER [bits]

#define ACK_IFS                 TURNAROUND_TIME ///< Ack Inter Frame Spacing [us] - delay between last symbol of received frame and first symbol of transmitted Ack
#define TXRU_TIME               40              ///< Transmitter ramp up time [us]
#define EVENT_LAT               23              ///< END event latency [us]

#define MAX_CRIT_SECT_TIME      60              ///< Maximal time that the driver spends in single critical section.

#define LQI_VALUE_FACTOR        4               ///< Factor needed to calculate LQI value based on data from RADIO peripheral
#define LQI_MAX                 0xff            ///< Maximal LQI value

/** Get LQI of given received packet. If CRC is calculated by hardware LQI is included instead of CRC
 *  in the frame. Length is stored in byte with index 0; CRC is 2 last bytes.
 */
#define RX_FRAME_LQI(data)      ((data)[(data)[0] - 1])

#if NRF_802154_RX_BUFFERS > 1
/// Pointer to currently used receive buffer.
static rx_buffer_t * mp_current_rx_buffer;

#else
/// If there is only one buffer use const pointer to the receive buffer.
static rx_buffer_t * const mp_current_rx_buffer = &nrf_802154_rx_buffers[0];

#endif

static const uint8_t * mp_ack;         ///< Pointer to Ack frame buffer.
static const uint8_t * mp_tx_data;     ///< Pointer to the data to transmit.
static uint32_t        m_ed_time_left; ///< Remaining time of the current energy detection procedure [us].
static uint8_t         m_ed_result;    ///< Result of the current energy detection procedure.

static volatile radio_state_t m_state; ///< State of the radio driver.

typedef struct
{
    bool frame_filtered        : 1; ///< If frame being received passed filtering operation.
    bool rx_timeslot_requested : 1; ///< If timeslot for the frame being received is already requested.
    bool rssi_started          : 1; // TODO: REMOVE THIS, this is in trx now
} nrf_802154_flags_t;

static nrf_802154_flags_t m_flags;               ///< Flags used to store the current driver state.

static volatile bool m_rsch_timeslot_is_granted; ///< State of the RSCH timeslot.

/***************************************************************************************************
 * @section Common core operations
 **************************************************************************************************/

/** Set driver state.
 *
 * @param[in]  state  Driver state to set.
 */
static void state_set(radio_state_t state)
{
    m_state = state;

    nrf_802154_log(EVENT_SET_STATE, (uint32_t)state);

    /* We should request preconditions according to desired state, currently we
     * request preconditions only for non-sleep and drop preconditions for sleep */

    if (state == RADIO_STATE_SLEEP)
    {
        nrf_802154_rsch_crit_sect_prio_request(RSCH_PRIO_IDLE);
    }
    else
    {
        nrf_802154_rsch_crit_sect_prio_request(RSCH_PRIO_MAX);
    }
}

/** Clear flags describing frame being received. */
static void rx_flags_clear(void)
{
    m_flags.frame_filtered        = false;
    m_flags.rx_timeslot_requested = false;
}

/** Request the RSSI measurement. */
static void rssi_measure(void)
{
    m_flags.rssi_started = true;
    nrf_radio_event_clear(NRF_RADIO_EVENT_RSSIEND);
    nrf_radio_task_trigger(NRF_RADIO_TASK_RSSISTART);
}

/** Wait for the RSSI measurement. */
static void rssi_measurement_wait(void)
{
    while (!nrf_radio_event_check(NRF_RADIO_EVENT_RSSIEND))
    {
        // Intentionally empty: This function is called from a critical section.
        // WFE would not be waken up by a RADIO event.
    }
}

/** Get the result of the last RSSI measurement.
 *
 * @returns  Result of the last RSSI measurement in dBm.
 */
static int8_t rssi_last_measurement_get(void)
{
    uint8_t rssi_sample = nrf_radio_rssi_sample_get();

    rssi_sample = nrf_802154_rssi_sample_corrected_get(rssi_sample);

    return -((int8_t)rssi_sample);
}

/** Get LQI of a received frame.
 *
 * @param[in]  p_data  Pointer to buffer containing PHR and PSDU of received frame
 *
 * @returns  LQI of given frame.
 */
static uint8_t lqi_get(const uint8_t * p_data)
{
    uint32_t lqi = RX_FRAME_LQI(p_data);

    lqi  = nrf_802154_rssi_lqi_corrected_get(lqi);
    lqi *= LQI_VALUE_FACTOR;

    if (lqi > LQI_MAX)
    {
        lqi = LQI_MAX;
    }

    return (uint8_t)lqi;
}

static void received_frame_notify(uint8_t * p_data)
{
    nrf_802154_notify_received(p_data,                      // data
                               rssi_last_measurement_get(), // rssi
                               lqi_get(p_data));            // lqi
}

/** Allow nesting critical sections and notify MAC layer that a frame was received. */
static void received_frame_notify_and_nesting_allow(uint8_t * p_data)
{
    nrf_802154_critical_section_nesting_allow();

    received_frame_notify(p_data);

    nrf_802154_critical_section_nesting_deny();
}

/** Notify MAC layer that receive procedure failed. */
static void receive_failed_notify(nrf_802154_rx_error_t error)
{
    nrf_802154_critical_section_nesting_allow();

    nrf_802154_notify_receive_failed(error);

    nrf_802154_critical_section_nesting_deny();
}

/** Notify MAC layer that transmission of requested frame has started. */
static void transmit_started_notify(void)
{
    const uint8_t * p_frame = mp_tx_data;

    if (nrf_802154_core_hooks_tx_started(p_frame))
    {
        nrf_802154_tx_started(p_frame);
    }

}

#if !NRF_802154_DISABLE_BCC_MATCHING
/** Notify that reception of a frame has started. */
static void receive_started_notify(void)
{
    const uint8_t * p_frame = mp_current_rx_buffer->data;

    nrf_802154_core_hooks_rx_started(p_frame);
}

#endif

/** Notify MAC layer that a frame was transmitted. */
static void transmitted_frame_notify(uint8_t * p_ack, int8_t power, uint8_t lqi)
{
    const uint8_t * p_frame = mp_tx_data;

    nrf_802154_critical_section_nesting_allow();

    nrf_802154_core_hooks_transmitted(p_frame);
    nrf_802154_notify_transmitted(p_frame, p_ack, power, lqi);

    nrf_802154_critical_section_nesting_deny();
}

/** Notify MAC layer that transmission procedure failed. */
static void transmit_failed_notify(nrf_802154_tx_error_t error)
{
    const uint8_t * p_frame = mp_tx_data;

    if (nrf_802154_core_hooks_tx_failed(p_frame, error))
    {
        nrf_802154_notify_transmit_failed(p_frame, error);
    }
}

/** Allow nesting critical sections and notify MAC layer that transmission procedure failed. */
static void transmit_failed_notify_and_nesting_allow(nrf_802154_tx_error_t error)
{
    nrf_802154_critical_section_nesting_allow();

    transmit_failed_notify(error);

    nrf_802154_critical_section_nesting_deny();
}

/** Notify MAC layer that energy detection procedure ended. */
static void energy_detected_notify(uint8_t result)
{
    nrf_802154_critical_section_nesting_allow();

    nrf_802154_notify_energy_detected(result);

    nrf_802154_critical_section_nesting_deny();
}

/** Notify MAC layer that CCA procedure ended. */
static void cca_notify(bool result)
{
    nrf_802154_critical_section_nesting_allow();

    nrf_802154_notify_cca(result);

    nrf_802154_critical_section_nesting_deny();
}

/** Update CCA configuration in RADIO registers. */
static void cca_configuration_update(void)
{
    nrf_802154_cca_cfg_t cca_cfg;

    nrf_802154_pib_cca_cfg_get(&cca_cfg);
    nrf_radio_cca_configure(cca_cfg.mode,
                            nrf_802154_rssi_cca_ed_threshold_corrected_get(cca_cfg.ed_threshold),
                            cca_cfg.corr_threshold,
                            cca_cfg.corr_limit);
}

/** Check if timeslot is currently granted.
 *
 * @retval true   The timeslot is granted.
 * @retval false  The timeslot is not granted.
 */
static bool timeslot_is_granted(void)
{
    return m_rsch_timeslot_is_granted;
}

/***************************************************************************************************
 * @section RX buffer management
 **************************************************************************************************/

/** Set currently used rx buffer to given address.
 *
 * @param[in]  p_rx_buffer  Pointer to receive buffer that should be used now.
 */
static void rx_buffer_in_use_set(rx_buffer_t * p_rx_buffer)
{
#if NRF_802154_RX_BUFFERS > 1
    mp_current_rx_buffer = p_rx_buffer;
#else
    (void)p_rx_buffer;
#endif
}

/** Check if currently there is available rx buffer.
 *
 * @retval true   There is available rx buffer.
 * @retval false  Currently there is no available rx buffer.
 */
static bool rx_buffer_is_available(void)
{
    return (mp_current_rx_buffer != NULL) && (mp_current_rx_buffer->free);
}

/** Get pointer to available rx buffer.
 *
 * @returns Pointer to available rx buffer or NULL if rx buffer is not available.
 */
static uint8_t * rx_buffer_get(void)
{
    return rx_buffer_is_available() ? mp_current_rx_buffer->data : NULL;
}

/***************************************************************************************************
 * @section Radio parameters calculators
 **************************************************************************************************/

/** Set radio channel
 *
 *  @param[in]  channel  Channel number to set (11-26).
 */
static void channel_set(uint8_t channel)
{
    assert(channel >= 11 && channel <= 26);

    nrf_radio_frequency_set(2405 + 5 * (channel - 11));
}

/***************************************************************************************************
 * @section ACK transmission management
 **************************************************************************************************/

/** Check if ACK is requested in given frame.
 *
 * @param[in]  p_frame  Pointer to a frame to check.
 *
 * @retval  true   ACK is requested in given frame.
 * @retval  false  ACK is not requested in given frame.
 */
static bool ack_is_requested(const uint8_t * p_frame)
{
    return nrf_802154_frame_parser_ar_bit_is_set(p_frame);
}

/***************************************************************************************************
 * @section RADIO peripheral management
 **************************************************************************************************/

/** Deinitialize interrupts for radio peripheral. */
static void irq_deinit(void)
{
    NVIC_DisableIRQ(RADIO_IRQn);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 0);

    __DSB();
    __ISB();
}

/***************************************************************************************************
 * @section Energy detection management
 **************************************************************************************************/

/** Get ED result value.
 *
 *  @param[in]  ed_sample   Energy Detection sample gathered from TRX module
 *  @returns ED result based on data collected during Energy Detection procedure.
 */
static uint8_t ed_result_get(uint8_t ed_sample)
{
    uint32_t result;

    result  = nrf_802154_rssi_ed_corrected_get(ed_sample);
    result *= ED_RESULT_FACTOR;

    if (result > ED_RESULT_MAX)
    {
        result = ED_RESULT_MAX;
    }

    return (uint8_t)result;
}

/** Setup next iteration of energy detection procedure.
 *
 *  Energy detection procedure is performed in iterations to make sure it is performed for requested
 *  time regardless radio arbitration.
 *
 *  @param[inout] p_requested_ed_time_us  Remaining time of energy detection procedure [us]. Value will be updated
 *                                        with time remaining for the next attempt of energy detection.
 *  @param[out]   p_next_trx_ed_count     Number of trx energy detection iterations to perform.
 *
 *  @retval  true   Next iteration of energy detection procedure will be performed now.
 *  @retval  false  Next iteration of energy detection procedure will not be performed now due to
 *                  ending timeslot.
 */
static bool ed_iter_setup(uint32_t * p_requested_ed_time_us, uint32_t * p_next_trx_ed_count)
{
    uint32_t iters_left_in_timeslot = nrf_802154_rsch_timeslot_us_left_get() / ED_ITER_DURATION;

    if (iters_left_in_timeslot > ED_ITERS_OVERHEAD)
    {
        /* Note that in single phy iters_left_in_timeslot will always be very big thus we will get here. */
        iters_left_in_timeslot -= ED_ITERS_OVERHEAD;

        uint32_t requested_iters = *p_requested_ed_time_us / ED_ITER_DURATION;

        if (requested_iters < iters_left_in_timeslot)
        {
            /* We will finish all iterations before timeslot end, thus no time is left */
            *p_requested_ed_time_us = 0U;
        }
        else
        {
            *p_requested_ed_time_us = *p_requested_ed_time_us - (iters_left_in_timeslot * ED_ITER_DURATION);
            requested_iters = iters_left_in_timeslot;
        }

        *p_next_trx_ed_count = requested_iters;

        return true;
    }

    return false;
}

/***************************************************************************************************
 * @section FSM transition request sub-procedures
 **************************************************************************************************/

/** Check if time remaining in the timeslot is long enough to process whole critical section. */
static bool remaining_timeslot_time_is_enough_for_crit_sect(void)
{
    return nrf_802154_rsch_timeslot_us_left_get() >= MAX_CRIT_SECT_TIME;
}

/** Check if critical section can be processed at the moment.
 *
 * @note This function returns valid result only inside critical section.
 *
 * @retval true   There is enough time in current timeslot or timeslot is denied at the moment.
 * @retval false  Current timeslot ends too shortly to process critical section inside.
 */
static bool critical_section_can_be_processed_now(void)
{
    return !timeslot_is_granted() || remaining_timeslot_time_is_enough_for_crit_sect();
}

/** Enter critical section and verify if there is enough time to complete operations within. */
static bool critical_section_enter_and_verify_timeslot_length(void)
{
    bool result = nrf_802154_critical_section_enter();

    if (result)
    {
        if (!critical_section_can_be_processed_now())
        {
            result = false;

            nrf_802154_critical_section_exit();
        }
    }

    return result;
}

static bool can_terminate_current_operation(radio_state_t state, nrf_802154_term_t term_lvl, bool receiving_psdu_now)
{
    bool result = false;

    switch (state)
    {
        case RADIO_STATE_SLEEP:
        case RADIO_STATE_FALLING_ASLEEP:
        case RADIO_STATE_CONTINUOUS_CARRIER:
            result = true;
            break;

        case RADIO_STATE_RX:
            result = (term_lvl >= NRF_802154_TERM_802154) || !receiving_psdu_now ;
            break;

        case RADIO_STATE_TX_ACK:
        case RADIO_STATE_CCA_TX:
        case RADIO_STATE_TX:
        case RADIO_STATE_RX_ACK:
        case RADIO_STATE_ED:
        case RADIO_STATE_CCA:
            result = (term_lvl >= NRF_802154_TERM_802154);
            break;

        default:
            assert(false);
    }

    return result;
}

static void operation_terminated_notify(radio_state_t state, bool receiving_psdu_now)
{
    switch (state)
    {
        case RADIO_STATE_SLEEP:
        case RADIO_STATE_FALLING_ASLEEP:
        case RADIO_STATE_CONTINUOUS_CARRIER:
            break;

        case RADIO_STATE_RX:
            if (receiving_psdu_now)
            {
                nrf_802154_notify_receive_failed(NRF_802154_RX_ERROR_ABORTED);
            }

            break;

        case RADIO_STATE_TX_ACK:
            mp_current_rx_buffer->free = false;
            received_frame_notify(mp_current_rx_buffer->data);
            break;

        case RADIO_STATE_CCA_TX:
        case RADIO_STATE_TX:
            transmit_failed_notify(NRF_802154_TX_ERROR_ABORTED);
            break;

        case RADIO_STATE_RX_ACK:
            transmit_failed_notify(NRF_802154_TX_ERROR_ABORTED);
            break;

        case RADIO_STATE_ED:
            nrf_802154_notify_energy_detection_failed(NRF_802154_ED_ERROR_ABORTED);
            break;

        case RADIO_STATE_CCA:
            nrf_802154_notify_cca_failed(NRF_802154_CCA_ERROR_ABORTED);
            break;

        default:
            assert(false);
    }
}

/** Terminate ongoing operation.
 *
 * This function is called when MAC layer requests transition to another operation.
 *
 * After calling this function RADIO should enter DISABLED state and Radio Scheduler
 * should be in continuous mode.
 *
 * @param[in]  term_lvl      Termination level of this request. Selects procedures to abort.
 * @param[in]  req_orig      Module that originates termination request.
 * @param[in]  notify_abort  If Termination of current operation shall be notified.
 *
 * @retval true   Terminated ongoing operation.
 * @retval false  Ongoing operation was not terminated.
 */
static bool current_operation_terminate(nrf_802154_term_t term_lvl,
                                        req_originator_t  req_orig,
                                        bool              notify)
{
    bool result = nrf_802154_core_hooks_terminate(term_lvl, req_orig);

    if (result)
    {
        bool receiving_psdu_now = false;
        if (m_state == RADIO_STATE_RX)
        {
            receiving_psdu_now = nrf_802154_trx_psdu_is_being_received();
        }

        result = can_terminate_current_operation(m_state, term_lvl, receiving_psdu_now);

        if (result)
        {
            nrf_802154_trx_abort();

            if (notify)
            {
                operation_terminated_notify(m_state, receiving_psdu_now);
            }
        }

    }

    return result;
}

/** Enter Sleep state. */
static void sleep_init(void)
{
    nrf_802154_trx_disable();
    nrf_802154_timer_coord_stop();
}

/** Initialize Falling Asleep operation. */
static void falling_asleep_init(void)
{
    if (!timeslot_is_granted())
    {
        sleep_init();
        state_set(RADIO_STATE_SLEEP);
        return;
    }

    if (nrf_802154_trx_go_idle())
    {
        // There will be nrf_802154_trx_in_idle call, where we will continue processing
    }
    else
    {
        sleep_init();
        state_set(RADIO_STATE_SLEEP);
    }
}

/** Initialize RX operation. */
static void rx_init(bool disabled_was_triggered)
{
    bool    free_buffer;

    if (!timeslot_is_granted())
    {
        return;
    }

    // Clear filtering flag
    rx_flags_clear();

    // Clear the RSSI measurement flag.
    // TODO: Move this to trx (partially moved)
    m_flags.rssi_started = false;

    // Find available RX buffer
    free_buffer = rx_buffer_is_available();

    nrf_802154_trx_receive_buffer_set(rx_buffer_get());

    nrf_802154_trx_receive_frame(BCC_INIT / 8U);

    // TODO: This is UGLY looking into radio's internal, should be wrapped by trx.
    // TODO: This is done also after triggering receive. There is possibility that frame will be received before
    // we connect event crcok to timer coord
    // Configure the timer coordinator to get a timestamp of the CRCOK event.
    nrf_802154_timer_coord_timestamp_prepare(
        (uint32_t)nrf_radio_event_address_get(NRF_RADIO_EVENT_CRCOK));

    // Find RX buffer if none available
    if (!free_buffer)
    {
        rx_buffer_in_use_set(nrf_802154_rx_buffer_free_find());

        nrf_802154_trx_receive_buffer_set(rx_buffer_get());
    }
}

/** Initialize TX operation. */
static bool tx_init(const uint8_t * p_data, bool cca, bool disabled_was_triggered)
{
    if (!timeslot_is_granted() || !nrf_802154_rsch_timeslot_request(
            nrf_802154_tx_duration_get(p_data[0], cca, ack_is_requested(p_data))))
    {
        return false;
    }

    nrf_802154_trx_transmit_frame(p_data, cca);

    return true;
}

/** Initialize ED operation */
static void ed_init(bool disabled_was_triggered)
{
    if (!timeslot_is_granted())
    {
        return;
    }

    uint32_t trx_ed_count = 0U;

    if (!ed_iter_setup(&m_ed_time_left, &trx_ed_count))
    {
        // Just wait for next timeslot if there is not enough time in this one.
        return;
    }

    nrf_802154_trx_energy_detection(trx_ed_count);
}

/** Initialize CCA operation. */
static void cca_init(bool disabled_was_triggered)
{
    if (!timeslot_is_granted() || !nrf_802154_rsch_timeslot_request(nrf_802154_cca_duration_get()))
    {
        return;
    }

    nrf_802154_trx_standalone_cca();
}

/** Initialize Continuous Carrier operation. */
static void continuous_carrier_init(bool disabled_was_triggered)
{
    if (!timeslot_is_granted())
    {
        return;
    }

    nrf_802154_trx_continuous_carrier();
}

/***************************************************************************************************
 * @section Radio Scheduler notification handlers
 **************************************************************************************************/

static void cont_prec_approved(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_TIMESLOT_STARTED);

    if (remaining_timeslot_time_is_enough_for_crit_sect() && !timeslot_is_granted())
    {
        nrf_802154_trx_enable();

        m_rsch_timeslot_is_granted = true;

        nrf_802154_timer_coord_start();

        switch (m_state)
        {
            case RADIO_STATE_SLEEP:
                // Intentionally empty. Appropriate action will be performed on state change.
                break;

            case RADIO_STATE_RX:
                rx_init(false);
                break;

            case RADIO_STATE_CCA_TX:
                (void)tx_init(mp_tx_data, true, false);
                break;

            case RADIO_STATE_TX:
                (void)tx_init(mp_tx_data, false, false);
                break;

            case RADIO_STATE_ED:
                ed_init(false);
                break;

            case RADIO_STATE_CCA:
                cca_init(false);
                break;

            case RADIO_STATE_CONTINUOUS_CARRIER:
                continuous_carrier_init(false);
                break;

            default:
                assert(false);
        }
    }

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_TIMESLOT_STARTED);
}

static void cont_prec_denied(void)
{
    bool result;

    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_TIMESLOT_ENDED);

    if (timeslot_is_granted())
    {
        m_rsch_timeslot_is_granted = false;

        bool receiving_psdu_now = false;
        if (m_state == RADIO_STATE_RX)
        {
            receiving_psdu_now = nrf_802154_trx_psdu_is_being_received();
        }

        nrf_802154_trx_disable();

        nrf_802154_timer_coord_stop();

        result = nrf_802154_core_hooks_terminate(NRF_802154_TERM_802154, REQ_ORIG_RSCH);
        assert(result);
        (void)result;

        switch (m_state)
        {
            case RADIO_STATE_FALLING_ASLEEP:
                state_set(RADIO_STATE_SLEEP);
                break;

            case RADIO_STATE_RX:
                if (receiving_psdu_now)
                {
                    receive_failed_notify(NRF_802154_RX_ERROR_TIMESLOT_ENDED);
                }

                break;

            case RADIO_STATE_TX_ACK:
                state_set(RADIO_STATE_RX);
                mp_current_rx_buffer->free = false;
                received_frame_notify_and_nesting_allow(mp_current_rx_buffer->data);
                break;

            case RADIO_STATE_CCA_TX:
            case RADIO_STATE_TX:
            case RADIO_STATE_RX_ACK:
                state_set(RADIO_STATE_RX);
                transmit_failed_notify_and_nesting_allow(NRF_802154_TX_ERROR_TIMESLOT_ENDED);
                break;

            case RADIO_STATE_ED:
            case RADIO_STATE_CCA:
            case RADIO_STATE_CONTINUOUS_CARRIER:
            case RADIO_STATE_SLEEP:
                // Intentionally empty.
                break;

            default:
                assert(false);
        }
    }

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_TIMESLOT_ENDED);
}

void nrf_802154_rsch_crit_sect_prio_changed(rsch_prio_t prio)
{
    if (prio > RSCH_PRIO_IDLE)
    {
        cont_prec_approved();
    }
    else
    {
        cont_prec_denied();
        nrf_802154_rsch_continuous_ended();
    }
}

/***************************************************************************************************
 * @section RADIO interrupt handler
 **************************************************************************************************/

void nrf_802154_trx_receive_on_shr(trx_state_t state)
{
    switch (state)
    {
        case TRX_STATE_RXACK:
            assert(m_state == RADIO_STATE_RX_ACK);
            nrf_802154_core_hooks_rx_ack_started();
            break;

        default:
            assert(false);
    }
}

#if !NRF_802154_DISABLE_BCC_MATCHING
uint8_t nrf_802154_trx_receive_on_bcmatch(uint8_t bcc)
{
    uint8_t               prev_num_data_bytes;
    uint8_t               num_data_bytes;
    nrf_802154_rx_error_t filter_result;
    bool                  frame_accepted = true;

    num_data_bytes      = bcc;
    prev_num_data_bytes = num_data_bytes;

    assert(num_data_bytes >= PHR_SIZE + FCF_SIZE);
    assert(m_state == RADIO_STATE_RX);

    if (!m_flags.frame_filtered)
    {
        filter_result = nrf_802154_filter_frame_part(mp_current_rx_buffer->data,
                                                     &num_data_bytes);

        if (filter_result == NRF_802154_RX_ERROR_NONE)
        {
            if (num_data_bytes != prev_num_data_bytes)
            {
                bcc = num_data_bytes;
            }
            else
            {
                m_flags.frame_filtered = true;
            }
        }
        else if ((filter_result == NRF_802154_RX_ERROR_INVALID_LENGTH) ||
                 (!nrf_802154_pib_promiscuous_get()))
        {
            nrf_802154_trx_receive_frame_abort();
            rx_init(true);

            frame_accepted = false;

            if ((mp_current_rx_buffer->data[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) !=
                FRAME_TYPE_ACK)
            {
                receive_failed_notify(filter_result);
            }
        }
        else
        {
            // Promiscuous mode, allow incorrect frames. Nothing to do here.
        }
    }

    if ((!m_flags.rx_timeslot_requested) && (frame_accepted))
    {
        if (nrf_802154_rsch_timeslot_request(nrf_802154_rx_duration_get(
                                                 mp_current_rx_buffer->data[0],
                                                 ack_is_requested(mp_current_rx_buffer->data))))
        {
            m_flags.rx_timeslot_requested = true;

            receive_started_notify();
        }
        else
        {
            // Disable receiver and wait for a new timeslot.
            nrf_802154_trx_receive_frame_abort();

            // We should not leave trx in temporary state, let's receive then.
            // We avoid hard reset of radio during TX ACK phase due to timeslot end,
            // which could result in spurious RF emission.
            rx_init(false);

            nrf_802154_notify_receive_failed(NRF_802154_RX_ERROR_TIMESLOT_ENDED);
        }
    }

    return bcc;
}
#endif

void nrf_802154_trx_in_idle(void)
{
    sleep_init();
    state_set(RADIO_STATE_SLEEP);
}

static void on_bad_ack(void);

void nrf_802154_trx_receive_crcerror(trx_state_t state)
{
    switch (state)
    {
#if !NRF_802154_DISABLE_BCC_MATCHING || NRF_802154_NOTIFY_CRCERROR
        case TRX_STATE_RXFRAME:
            // We don't change receive buffer, receive will go to the same that was already used
#if !NRF_802154_DISABLE_BCC_MATCHING
            nrf_802154_trx_receive_frame(BCC_INIT / 8U);
#else
            // With BCC matching disabled trx module will re-arm automatically
#endif
#if NRF_802154_NOTIFY_CRCERROR
            receive_failed_notify(NRF_802154_RX_ERROR_INVALID_FCS);
#endif // NRF_802154_NOTIFY_CRCERROR
            break;
#endif
        case TRX_STATE_RXACK:
            assert(m_state == RADIO_STATE_RX_ACK);
            on_bad_ack();
            break;

        default:
            assert(false);
    }
}

static void on_trx_received_frame(void)
{
    uint8_t * p_received_data = mp_current_rx_buffer->data;

    m_flags.rssi_started = true;

#if NRF_802154_DISABLE_BCC_MATCHING
    uint8_t               num_data_bytes      = PHR_SIZE + FCF_SIZE;
    uint8_t               prev_num_data_bytes = 0;
    nrf_802154_rx_error_t filter_result;

    // Frame filtering
    while (num_data_bytes != prev_num_data_bytes)
    {
        prev_num_data_bytes = num_data_bytes;

        // Keep checking consecutive parts of the frame header.
        filter_result = nrf_802154_filter_frame_part(mp_current_rx_buffer->data, &num_data_bytes);

        if (filter_result == NRF_802154_RX_ERROR_NONE)
        {
            if (num_data_bytes == prev_num_data_bytes)
            {
                m_flags.frame_filtered = true;
            }
        }
        else
        {
            break;
        }
    }

    // Timeslot request
    if (m_flags.frame_filtered &&
        ack_is_requested(p_received_data) &&
        !nrf_802154_rsch_timeslot_request(nrf_802154_rx_duration_get(0, true)))
    {
        // Frame is destined to this node but there is no timeslot to transmit ACK.
        // Just disable receiver and wait for a new timeslot.
        nrf_802154_trx_receive_frame_abort();

        rx_flags_clear();

        // Filter out received ACK frame if promiscuous mode is disabled.
        if (((p_received_data[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK) ||
            nrf_802154_pib_promiscuous_get())
        {
            mp_current_rx_buffer->free = false;
            received_frame_notify_and_nesting_allow(p_received_data);
        }

        return;
    }
#endif // NRF_802154_DISABLE_BCC_MATCHING

    if (m_flags.frame_filtered || nrf_802154_pib_promiscuous_get())
    {
        bool send_ack = false;

        if (m_flags.frame_filtered &&
            ack_is_requested(mp_current_rx_buffer->data) &&
            nrf_802154_pib_auto_ack_get())
        {
            mp_ack = nrf_802154_ack_generator_create(mp_current_rx_buffer->data);
            if (NULL != mp_ack)
            {
                send_ack = true;
            }
        }

        if (send_ack)
        {
            // TODO: What about preconditions for coex here?
            if (nrf_802154_trx_transmit_ack(mp_ack, ACK_IFS))
            {
                state_set(RADIO_STATE_TX_ACK);
            }
            else
            {
                mp_current_rx_buffer->free = false;

                rx_init(true);

                received_frame_notify_and_nesting_allow(p_received_data);
            }
        }
        else
        {
            // Filter out received ACK frame if promiscuous mode is disabled.
            if (((p_received_data[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK) ||
                nrf_802154_pib_promiscuous_get())
            {
                // Current buffer will be passed to the application
                mp_current_rx_buffer->free = false;

                // Find new buffer
                rx_buffer_in_use_set(nrf_802154_rx_buffer_free_find());

                rx_init(true);

                received_frame_notify_and_nesting_allow(p_received_data);
            }
            else
            {
                // Receive to the same buffer
                rx_init(true);
            }
        }
    }
    else
    {
        // CRC is OK, but filtering operation did not end - it is invalid frame with valid CRC
        // or problem due to software latency (i.e. handled BCMATCH, CRCERROR, CRCOK from two
        // consecutively received frames).
        rx_init(true);

#if NRF_802154_DISABLE_BCC_MATCHING
        if ((p_received_data[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK)
        {
            receive_failed_notify(filter_result);
        }
#else // NRF_802154_DISABLE_BCC_MATCHING
        receive_failed_notify(NRF_802154_RX_ERROR_RUNTIME);
#endif  // NRF_802154_DISABLE_BCC_MATCHING
    }
}

static void on_trx_received_ack(void);

void nrf_802154_trx_receive_received(trx_state_t state)
{
    switch (state)
    {
        case TRX_STATE_RXFRAME:
            on_trx_received_frame();
            break;

        case TRX_STATE_RXACK:
            on_trx_received_ack();
            break;

        default:
            assert(false);
    }
}

void nrf_802154_trx_transmit_started(trx_state_t state)
{
    switch (state)
    {
#if NRF_802154_TX_STARTED_NOTIFY_ENABLED
        case TRX_STATE_TXFRAME:
            assert((m_state == RADIO_STATE_TX) || (m_state == RADIO_STATE_CCA_TX));
            transmit_started_notify();
            break;
#endif
        case TRX_STATE_TXACK:
            assert(m_state == RADIO_STATE_TX_ACK);
            nrf_802154_tx_ack_started(mp_ack);
            break;

        default:
            assert(false);
    }
}

static void on_trx_transmitted_ack(void)
{
    assert(m_state == RADIO_STATE_TX_ACK);

    uint8_t * p_received_data = mp_current_rx_buffer->data;

    // Current buffer used for receive operation will be passed to the application
    mp_current_rx_buffer->free = false;

    state_set(RADIO_STATE_RX);

    rx_init(true);

    received_frame_notify_and_nesting_allow(p_received_data);
}

static void on_trx_transmitted_frame(void)
{
    if (ack_is_requested(mp_tx_data))
    {
        state_set(RADIO_STATE_RX_ACK);

        bool     rx_buffer_free = rx_buffer_is_available();

        nrf_802154_trx_receive_buffer_set(rx_buffer_get());

        nrf_802154_trx_receive_ack();

        if (!rx_buffer_free)
        {
            rx_buffer_in_use_set(nrf_802154_rx_buffer_free_find());

            nrf_802154_trx_receive_buffer_set(rx_buffer_get());
        }
    }
    else
    {
        state_set(RADIO_STATE_RX);

        rx_init(true);

        transmitted_frame_notify(NULL, 0, 0);
    }
}

void nrf_802154_trx_transmit_transmitted(trx_state_t state)
{
    switch (state)
    {
        case TRX_STATE_TXACK:
            on_trx_transmitted_ack();
            break;

        case TRX_STATE_TXFRAME:
            on_trx_transmitted_frame();
            break;

        default:
            assert(false);
    }
}

static bool ack_match_check_version_not_2(const uint8_t * p_tx_data, const uint8_t * p_ack_data)
{
    // Frame Version != 2

    // Check: Phy length
    if (p_ack_data[PHR_OFFSET] != IMM_ACK_LENGTH)
    {
        return false;
    }

    // Check if Frame version is 0 or 1.
    switch (p_ack_data[FRAME_VERSION_OFFSET] & FRAME_VERSION_MASK)
    {
        case FRAME_VERSION_0:
        case FRAME_VERSION_1:
            break;
        default:
            return false;
    }

    // Check: Sequence number match
    if (p_ack_data[DSN_OFFSET] != p_tx_data[DSN_OFFSET])
    {
        return false;
    }

    return true;
}

static bool ack_match_check_version_2(const uint8_t * p_tx_data, const uint8_t * p_ack_data)
{
    if ((p_ack_data[FRAME_VERSION_OFFSET] & FRAME_VERSION_MASK) != FRAME_VERSION_2)
    {
        return false;
    }

    // Transmitted frame was Version 2
    // For frame version 2 sequence number bit may be suppressed and its check fails.
    // Verify ACK frame using its destination address.
    nrf_802154_frame_parser_mhr_data_t tx_mhr_data;
    nrf_802154_frame_parser_mhr_data_t ack_mhr_data;
    bool                               parse_result;

    parse_result = nrf_802154_frame_parser_mhr_parse(p_tx_data, &tx_mhr_data);
    assert(parse_result);
    parse_result = nrf_802154_frame_parser_mhr_parse(p_ack_data, &ack_mhr_data);

    // TOOD: Check PHR (frame length), may be complicated
    if (!parse_result ||
        (tx_mhr_data.p_src_addr == NULL) ||
        (ack_mhr_data.p_dst_addr == NULL) ||
        (tx_mhr_data.src_addr_size != ack_mhr_data.dst_addr_size) ||
        (0 != memcmp(tx_mhr_data.p_src_addr,
                     ack_mhr_data.p_dst_addr,
                     tx_mhr_data.src_addr_size)))
    {
        // Mismatch
        return false;
    }

    return true;
}

static bool ack_match_check(const uint8_t * p_tx_data, const uint8_t * p_ack_data)
{
    if ((p_tx_data == NULL) || (p_ack_data == NULL))
    {
        return false;
    }

    // Check: Frame Control Field -> Frame type
    if ((p_ack_data[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK) != FRAME_TYPE_ACK)
    {
        return false; // This is not an ACK frame
    }

    // Check: Frame Control Field -> Frame version
    if ((p_tx_data[FRAME_VERSION_OFFSET] & FRAME_VERSION_MASK) == FRAME_VERSION_2)
    {
        return ack_match_check_version_2(p_tx_data, p_ack_data);
    }

    return ack_match_check_version_not_2(p_tx_data, p_ack_data);
}

static void on_bad_ack(void)
{
    // We received either a frame with incorrect CRC or not an ACK frame or not matching ACK
    state_set(RADIO_STATE_RX);

    rx_init(true);

    transmit_failed_notify_and_nesting_allow(NRF_802154_TX_ERROR_INVALID_ACK);
}

static void on_trx_received_ack(void)
{
    // CRC of received frame is correct
    uint8_t     * p_ack_data   = mp_current_rx_buffer->data;

    if (ack_match_check(mp_tx_data, p_ack_data))
    {
        rx_buffer_t * p_ack_buffer = mp_current_rx_buffer;

        mp_current_rx_buffer->free = false;

        state_set(RADIO_STATE_RX);
        rx_init(true);

        transmitted_frame_notify(p_ack_buffer->data,           // phr + psdu
                                 rssi_last_measurement_get(),  // rssi
                                 lqi_get(p_ack_buffer->data)); // lqi;

    }
    else
    {
        on_bad_ack();
    }
}

void nrf_802154_trx_standalone_cca_finished(bool channel_was_idle)
{
    state_set(RADIO_STATE_RX);
    rx_init(true);

    cca_notify(channel_was_idle);
}

void nrf_802154_trx_transmit_ccabusy(void)
{
    state_set(RADIO_STATE_RX);
    rx_init(true);

    transmit_failed_notify_and_nesting_allow(NRF_802154_TX_ERROR_BUSY_CHANNEL);
}

void nrf_802154_trx_energy_detection_finished(uint8_t ed_sample)
{
    if (m_ed_result < ed_sample)
    {
        // Collect maximum value of samples provided by trx
        m_ed_result = ed_sample;
    }

    if (m_ed_time_left >= ED_ITER_DURATION)
    {
        uint32_t trx_ed_count = 0U;
        if (ed_iter_setup(&m_ed_time_left, &trx_ed_count))
        {
            nrf_802154_trx_energy_detection(trx_ed_count);
        }
        else
        {
            /* There is too little time in current timeslot, just wait for timeslot end.
             * Operation will be resumed in next timeslot
             */
        }
    }
    else
    {
        channel_set(nrf_802154_pib_channel_get());

        state_set(RADIO_STATE_RX);
        rx_init(true);

        energy_detected_notify(ed_result_get(m_ed_result));

    }
}

/***************************************************************************************************
 * @section API functions
 **************************************************************************************************/

void nrf_802154_core_init(void)
{
    m_state                    = RADIO_STATE_SLEEP;
    m_rsch_timeslot_is_granted = false;

    nrf_802154_trx_init();
    nrf_802154_ack_generator_init();
}

void nrf_802154_core_deinit(void)
{
    if (timeslot_is_granted())
    {
        nrf_802154_trx_disable();
    }

    nrf_802154_fal_cleanup();

    irq_deinit();
}

radio_state_t nrf_802154_core_state_get(void)
{
    return m_state;
}

bool nrf_802154_core_sleep(nrf_802154_term_t term_lvl)
{
    bool result = nrf_802154_critical_section_enter();

    if (result)
    {
        if ((m_state != RADIO_STATE_SLEEP) && (m_state != RADIO_STATE_FALLING_ASLEEP))
        {
            result = current_operation_terminate(term_lvl, REQ_ORIG_CORE, true);

            if (result)
            {
                state_set(RADIO_STATE_FALLING_ASLEEP);
                falling_asleep_init();
            }
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_receive(nrf_802154_term_t              term_lvl,
                             req_originator_t               req_orig,
                             nrf_802154_notification_func_t notify_function,
                             bool                           notify_abort)
{
    bool result = nrf_802154_critical_section_enter();

    if (result)
    {
        if ((m_state != RADIO_STATE_RX) && (m_state != RADIO_STATE_TX_ACK))
        {
            if (critical_section_can_be_processed_now())
            {
                result = current_operation_terminate(term_lvl, req_orig, notify_abort);

                if (result)
                {
                    state_set(RADIO_STATE_RX);
                    rx_init(true);
                }
            }
            else
            {
                result = false;
            }
        }

        if (notify_function != NULL)
        {
            notify_function(result);
        }

        nrf_802154_critical_section_exit();
    }
    else
    {
        if (notify_function != NULL)
        {
            notify_function(false);
        }
    }

    return result;
}

bool nrf_802154_core_transmit(nrf_802154_term_t              term_lvl,
                              req_originator_t               req_orig,
                              const uint8_t                * p_data,
                              bool                           cca,
                              bool                           immediate,
                              nrf_802154_notification_func_t notify_function)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        result = current_operation_terminate(term_lvl, req_orig, true);

        if (result)
        {
            // Set state to RX in case sleep terminate succeeded, but tx_init fails.
            state_set(RADIO_STATE_RX);

            mp_tx_data = p_data;
            result     = tx_init(p_data, cca, true);

            if (!immediate)
            {
                result = true;
            }
        }

        if (result)
        {
            state_set(cca ? RADIO_STATE_CCA_TX : RADIO_STATE_TX);
        }

        if (notify_function != NULL)
        {
            notify_function(result);
        }

        nrf_802154_critical_section_exit();
    }
    else
    {
        if (notify_function != NULL)
        {
            notify_function(false);
        }
    }

    return result;
}

bool nrf_802154_core_energy_detection(nrf_802154_term_t term_lvl, uint32_t time_us)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        result = current_operation_terminate(term_lvl, REQ_ORIG_CORE, true);

        if (result)
        {
            if (time_us < ED_ITER_DURATION)
            {
                time_us = ED_ITER_DURATION;
            }

            state_set(RADIO_STATE_ED);
            m_ed_time_left = time_us;
            m_ed_result    = 0;
            ed_init(true);
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_cca(nrf_802154_term_t term_lvl)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        result = current_operation_terminate(term_lvl, REQ_ORIG_CORE, true);

        if (result)
        {
            state_set(RADIO_STATE_CCA);
            cca_init(true);
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_continuous_carrier(nrf_802154_term_t term_lvl)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        result = current_operation_terminate(term_lvl, REQ_ORIG_CORE, true);

        if (result)
        {
            state_set(RADIO_STATE_CONTINUOUS_CARRIER);
            continuous_carrier_init(true);
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_notify_buffer_free(uint8_t * p_data)
{
    rx_buffer_t * p_buffer     = (rx_buffer_t *)p_data;
    bool          in_crit_sect = critical_section_enter_and_verify_timeslot_length();

    p_buffer->free = true;

    if (in_crit_sect)
    {
        if (timeslot_is_granted())
        {
            if (nrf_802154_trx_receive_is_buffer_missing())
            {
                rx_buffer_in_use_set(p_buffer);
                nrf_802154_trx_receive_buffer_set(rx_buffer_get());
            }
        }

        nrf_802154_critical_section_exit();
    }

    return true;
}

bool nrf_802154_core_channel_update(void)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        // TODO: rewrite this function to use trx
        if (timeslot_is_granted())
        {
            channel_set(nrf_802154_pib_channel_get());
        }

        switch (m_state)
        {
            case RADIO_STATE_RX:
                if (current_operation_terminate(NRF_802154_TERM_NONE, REQ_ORIG_CORE, true))
                {
                    rx_init(true);
                }

                break;

            case RADIO_STATE_CONTINUOUS_CARRIER:
                if (timeslot_is_granted())
                {
                    nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);
                }

                break;

            default:
                // Don't perform any additional action in any other state.
                break;
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_cca_cfg_update(void)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        if (timeslot_is_granted())
        {
            // TODO: rewrite this function to use trx
            cca_configuration_update();
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_rssi_measure(void)
{
    bool result = critical_section_enter_and_verify_timeslot_length();

    if (result)
    {
        if (timeslot_is_granted() && (m_state == RADIO_STATE_RX))
        {
            rssi_measure();
        }
        else
        {
            result = false;
        }

        nrf_802154_critical_section_exit();
    }

    return result;
}

bool nrf_802154_core_last_rssi_measurement_get(int8_t * p_rssi)
{
    bool result       = false;
    bool rssi_started = m_flags.rssi_started;
    bool in_crit_sect = false;

    if (rssi_started)
    {
        in_crit_sect = critical_section_enter_and_verify_timeslot_length();
    }

    if (rssi_started && in_crit_sect)
    {
        // Checking if a timeslot is granted is valid only in a critical section
        if (timeslot_is_granted())
        {
            rssi_measurement_wait();
            *p_rssi = rssi_last_measurement_get();
            result  = true;
        }
    }

    if (in_crit_sect)
    {
        nrf_802154_critical_section_exit();
    }

    return result;
}
