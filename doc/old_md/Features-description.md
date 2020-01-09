## Receiving frames and frames filtering

The driver can receive unicast and broadcast 802.15.4 frames on channels 11-26 from the channel page 0. The driver performs most of the received frame filtering procedure (IEEE 802.15.4-2006: 7.5.6.2). The filtering procedure may be performed during frame reception or after depending on driver configuration.
Filtering during frame reception:
 1. When FCF is received (first BCMATCH event), the driver checks if the length of the frame is valid and verifies the frame type and version.
 2. When destination address fields (PAN ID and address) are present and received (second BCMATCH event), the driver checks if the frame is destined to this node (broadcast or unicast).
 3. When the full frame is received (END event), the driver verifies if the FCS field contains a valid value.
Filtering after frame reception performs all above filtering steps in END event handler.
If all checks pass, the driver passes the received frame to the MAC layer.

Note that steps 1 and 2 may be bypassed in [promiscuous mode](Features-description#promiscuous-mode).

Received frame may optionally include a timestamp. The timestamp can be used to support synchronous communication like CSL or TSCH.

## Automatic sending of ACK frames

The MAC layer may configure the driver to automatically send ACK frames (enabled by default). The automatically created ACK frame is compliant with IEEE 802.15.4-2006: 7.2.2.3. This frame is sent exactly 192 us (aTurnaroundTime) after a data frame is received. The ACK frame is sent only if the received frame passes all steps of [the filter](Features-description#receiving-frames-and-frames-filtering) (even in promiscuous mode) and if the ACK request bit is present in FCF of the received frame.

The automatic ACK procedure uses a dedicated TIMER peripheral to keep valid inter-frame spacing regardless of ISR latency. The driver sets PPI and TIMER registers to end transmitter ramping up exactly 192 us (aTurnarondTime) after the frame is received. If any of the filter steps fails, the driver does not set PPI to start transmitter ramp up when TIMER fires. If all filter steps pass, the driver checks the version of the received frame and accordingly prepares an immediate acknowledgement (Imm-Ack) frame or an enhanced acknowledgement (Enh-Ack) frame. In the Imm-Ack, the driver sets the correct sequence number in the ACK frame, prepares PPIs to ramp up the transmitter, and optionally performs [automatic pending bit procedure](Features-description#setting-pending-bit-in-ack-frames). In the Enh-Ack, the driver fills all required fields (including [automatic pending bit procedure](Features-description#setting-pending-bit-in-ack-frames)) and optionally adds the last configured Information Elements to the acknowledgement frame. The ACK frame is sent automatically by TIMER, RADIO shorts, and PPIs.

## Setting pending bit in ACK frames

The MAC layer may configure the driver to automatically set a pending bit in automatically generated ACK frames (this feature is enabled by default). If this feature is disabled, pending bit in automatically generated ACK frames is always set (1). If this feature is enabled, the driver compares the source address of the data frame (the one that is being acknowledged) with an array of entries containing addresses of nodes for which the MAC layer has pending data. If the driver matches the source address with an entry in the array, the pending bit is set (1). If the array does not contain an address matching the source address, the pending bit is cleared (0). If the ACK frame is transmitted before the matching procedure is completed (i.e. due to too big array), the pending bit is set (1).

This procedure is performed in the DISABLED event handler when the receiver is disabled in order to enable the transmitter to automatically transmit an ACK frame. Due to DISABLED->TXEN short, this procedure takes place during transmitter ramp up time.

## Transmission of unicast and broadcast frames

The radio driver allows the MAC layer to transmit a frame containing any PSDU. The RADIO peripheral updates the FCS field of every frame automatically. A transmission procedure may be preceded by a CCA procedure. The driver automatically receives an ACK frame if requested.

## Automatic CCA procedure before transmission

The MAC layer may request the driver to perform a CCA procedure prepending transmission. If CCA procedure is requested, the driver performs a CCA procedure. If the channel is busy, the driver notifies the MAC layer and ends the transmission procedure. If the channel is idle, the driver starts transmission immediately after the CCA procedure ends.

## Automatic receiving of ACK frames

If FCF of the frame requested for transmission has the ACK request bit cleared, the driver ends the transmission procedure and notifies the MAC layer right after the RADIO peripheral ends transmission of the frame. If FCF of the frame has the ACK request bit set, the driver waits for ACK frame. Waiting may be interrupted by four events:
1. The driver receives the expected ACK frame. In this case, the driver resets the receiver, enters the 'receive' state, and notifies the MAC layer that the transmission succeeded.
2. The driver receives another frame than the expected ACK. In this case, the driver resets the receiver, enters the 'receive' state, and notifies the MAC layer that the transmission failed.
3. If the automatic ACK time-out feature is enabled and the ACK timer expires, the driver resets the receiver, enters the 'receive' state, and notifies the MAC layer that the transmission failed.
4. If the automatic ACK time-out feature is disabled and the ACK timer expires, the MAC layer must call the 'receive()' or the 'sleep()' request to stop waiting for an ACK frame. In this case, the driver resets the receiver and enters the 'receive' state. It does not notify the MAC layer.

## Stand-alone CCA procedure

The driver can perform a stand-alone CCA procedure. The MAC layer should request it when the driver is in 'receive' state. The driver notifies the MAC layer about the result of the CCA procedure by the cca_done() call. After the CCA procedure ends, the driver enters the 'receive' state.

## Low power mode

The MAC layer can request the driver to enter low power mode (sleep). In this mode, the RADIO peripheral cannot receive or transmit any frames, but power consumption is minimal.

## Energy detection

The driver can perform an energy detection procedure for the time given by the MAC layer. This procedure returns the maximal energy level detected during the procedure. The time given by the MAC layer is rounded up to multiplication of 128 us.

Note that the energy detection procedure in a multiprotocol configuration may take longer than the requested time. Energy detection is interrupted by another protocols' radio activity, but the total time of energy-detection periods is greater or equal to the time requested by the MAC layer.

## Promiscuous mode

In promiscuous mode, the driver reports to the MAC layer the received frames that either pass all [filter](Features-description#receiving-frames-and-frames-filtering) steps or fail steps 1 or 2. If any step of the filter fails, the driver does not [automatically transmit an ACK frame](Features-description#automatic-sending-of-ack-frames) in response to the received frame.

## Continuous carrier transmission

The driver can send a continuous carrier wave on a selected channel. This mode is intended for device testing and __should not__ be used in a product application. Continuous carrier transmission forces CCA (ED mode) to report a busy channel on nearby devices. The MAC layer should request entering the 'receive' state by the driver to stop continuous carrier transmission.

Continuous carrier is transmitted when the RADIO peripheral is in the TXIDLE state.

## CSMA-CA

There is an optional feature of the driver to perform CSMA-CA procedure followed by frame transmission. This feature can be configured compile-time. The MAC layer shall call 'csma_ca()' to initiate this procedure. The end of the procedure is notified by the 'tx_started()' or 'transmit_failed()' function. The driver [receives ACK frames](Features-description#automatic-receiving-of-ack-frames) like after any other transmission procedure.

## Delayed operations

The driver can transmit or receive a frame at a requested time, which provides support for synchronous communication. This feature can be used by a higher layer to support features like CSL, TSCH, or Zigbee GP Proxy.

Delayed transmission and reception operations are not exclusive which means that there can be one transmission and one reception scheduled at the same time. On the other hand, the driver does not check if the operations are conflicting. As a result, delayed operations may interrupt each other.

### Delayed reception

The delayed reception feature puts the driver to the RECEIVE state for a given period of time. If a start of frame is detected during the reception window, the window is automatically extended to be able to receive the whole frame. The window is kept open after the frame is received. The end of the window is notified to the MAC layer with the `rx_failed(RX_TIMEOUT)` notification. The driver does not automatically transit to the sleep state at the end of the reception window. It is a responsibility of the MAC layer to request the transition to the required state and request the next delayed reception operation.
