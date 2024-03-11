### e78-lora
A small header-only lib to control the Ebyte E78 LoRaWAN chip based on the ASR6601 LPWAN SoC. Feel free to modify necessary values like retry etc. Tested with platformio v6.1 and esp-idf 6.2.

### Usage
The library provides a class that's very easy to use:
```cpp
#include "ebyte-e78.hpp"
#define EX_UART_NUM UART_NUM_2

void cb_func(std::string data)
{
    // handle lorawan downlinks
}

extern "C" void app_main(void)
{
    E78LORAWAN e78{};
    e78.configure(EX_UART_NUM, 26, 27);
    e78.set_downstream_callback(cb_func);
    e78.set_app_eui("APPEUI");
    e78.set_app_key("APPKEY");
    e78.set_dev_eui("DEVEUI");
    e78.set_class(E78LORAWAN::CLASS_C);
    e78.set_tx_power(E78LORAWAN::TX_17_DBM);
    e78.set_confirmation_enable(true);
    e78.save();
    e78.join_network();
    while(1){
        // proceed
    }
}
```
The registered callback function will be called if there are downlink messages available (HEX, ASCII).
