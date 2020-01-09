## FSM description

The main part of the driver is an FSM. From API perspective, it has six states. Most of these states contain sub-states in the implementation.

We can put states in two categories:
1. Stable states
    * Sleep
    * RX
    * Continuous Carrier
2. Temporary states
    * TX
    * ED
    * CCA

### Transitions

The driver is initialized in the Sleep state.

The driver may be in a stable state indefinitely. When the driver is in any of the stable
states, the MAC layer can request entering any other state, and this request should
be successful. If the driver is busy with any operation (i.e. request from another
execution context or receiving PSDU), the request to leave the stable state fails.

Each temporary state is left automatically when the associated procedure ends. Then the
driver transits to default state. Default state is RX. Also MAC layer may request to
leave any temporary state by requesting transition to Sleep or RX state. This request
should be successful. Requesting transition from any temporary state to other
states than Sleep or RX shall result in failure.

When a frame is received in the RX state, the driver notifies the higher layer by 
calling the received() function. This function is called after reception of a frame 
without the Ack Request bit set or after sending an ACK to a frame with the Ack Request bit set.
In promiscuous mode, the higher layer is notified about all of the received
frames, even if the frame was not destined to the receiving node.

To transmit a frame, the higher layer must call the transmit() function. If the 
channel is busy, the driver goes back to the default state and notifies the 
higher layer by calling the transmit_failed() function. If a frame without Ack Request
bit was transmitted, the driver goes back to the default state and notifies the higher 
layer by calling the transmitted() function. If a frame with Ack Request bit was 
transmitted and an ACK was received, the driver goes back to the default state and 
notifies the higher layer by calling the transmitted() function.

To perform an Energy Detection procedure, the higher layer must call the
energy_detection() function. When the procedure is completed, the driver automatically goes 
back to the default state and notifies the higher layer with the energy_detected() function.

To transmit carrier continuously, the higher layer must call the continuous_carrier() 
function. Note that the __Continuous carrier state should be used
only in test applications__. In Continuous carrier state, the driver transmits a
carrier wave until it is requested by the higher layer to transmit to another state.

To perform a stand-alone CCA procedure, the higher layer must call the cca()
function. When the procedure is completed, the driver automatically transits back
to the default state and notifies the higher layer with the cca_done() function.

The following image shows all possible states and their transitions.
<a href="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/state_diagram.png" target="_blank"><img src="https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/state_diagram.png" /></a>
(Click on image to enlarge)

### States

#### Sleep
In this state, the radio is in low power mode. It cannot transmit or receive any frames.

#### RX (receive)
In this state, the radio [receives 802.15.4 frames](Features-description#receiving-frames-and-frames-filtering). It filters out frames with invalid CRC, length, type, or destination address. 
If the driver receives a frame destined to the receiving node, it can [automatically transmit an ACK frame](Features-description#automatic-sending-of-ack-frames).

#### TX (transmit)
In this state, the radio can perform the [CCA procedure](Features-description#automatic-cca-procedure-before-transmission). If the channel is free, the radio [transmits the requested frame](Features-description#transmission-of-unicast-and-broadcast-frames). If an ACK was requested in the transmitted frame, the driver [automatically receives the ACK frame](Features-description#automatic-receiving-of-ack-frames) in this state.

#### ED (energy detection)
In this state, the radio performs the [Energy Detection procedure](Features-description#energy-detection). The end of this procedure is notified to the higher layer by a energy_detected() function call.

#### CCA
In this state, the radio performs the [stand-alone CCA procedure](Features-description#stand-alone-cca-procedure). The end of this procedure it notified to the higher layer by a cca_done() function call.

#### Continuous Carrier
In this state, the radio [emits a carrier wave continuously](Features-description#continuous-carrier-transmission). It is a stable state and shall be exited by a state transition request from the higher layer.