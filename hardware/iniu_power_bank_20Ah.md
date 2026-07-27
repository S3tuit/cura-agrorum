Amazon link: https://www.amazon.it/dp/B07YPS5JC5.

## Measurements

### Setup
Powerbank OUT2 -> USB -> USB to DIP Adapter -> 5V/GND pins of the esp32 DevKit.
Multimeter in series between DIP Adapter and 5V.

esp32 used: ESP32-WROOM-32D DevKit

### Results

Mode, mA, seconds after powerbank turns off
vTaskDelay, 35.2, hasn't turned off
__asm__ __volatile__("nop"), 48.6, hasn't turned off
deepsleep, 6.4, 16.98

A combination of 35mV for 5s and 6.4mV for 10s makes the powerbank turns off in around 30 minutes.  
A combination of 35mV for 5s and 6.4mV for 5s doesn't seem to make the powerbank turns off. Tested for 2 days and it's still on.  

