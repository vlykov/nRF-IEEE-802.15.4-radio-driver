## Logs

The driver implements a debug logging feature. Event logs are stored in RAM memory. The logs can be retrieved from RAM by a debugger and decoded to a sequence diagram with the provided `decoder.html` utility. To enable this feature, add the **ENABLE_DEBUG_LOG** define option while compiling the sources.

## Extracting driver logs using GDB
```
set print elements 0
set print repeats unlimited
set pagination off
set height unlimited

p/z nrf_802154_debug_log_buffer
p nrf_802154_debug_log_ptr
```

## Creating a sequence diagram based on the extracted logs
Copy the output of `nrf_802154_debug_log_buffer` to the GDB Log area in the _decoder.html_ utility. Then, copy the `nrf_802154_debug_log_ptr` value to the Index pointer field in decoder.html.

![Event decoder - input form](https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/event_decoder_input.png)

Then, click Generate sequence diagram. The sequence diagram will be automatically generated.

![Event decoder - sequence diagram](https://raw.githubusercontent.com/wiki/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/img/event_decoder_sequence_diagram.png)
