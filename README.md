# MIDI_Merger

I wanted a simple and cheap solution to add an USB midi controller to a chain of old fashioned serial MIDI devices. I found none, cheapest one is about 30 euros, which is ridiculous, as an USB controller could be less than 10.

Then I found a project on Adafrut page, which is almost what I need, except that Adafruit modules were unavailable for me. I built a clone on the breadboard only to discover that original code fails, if you move several knobs simultaneously. When there are too many events USB stops working and module requries a restart. 

* Here is their repository: https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/main/USB_MIDI_Host_Messenger/USB_MIDI_Host_Messenger.ino (MIT License)
* And original page with electronic parts: https://learn.adafruit.com/usb-midi-host-messenger

So I modified their code significantly to avoid hanging and be more responsive MIDI-wise. 
And will add PCB part to make a compact soldered device.

## Hardware
* MPU: RP2040
* MIDI: 5-pin DIN or trs, 2 pieces
* USB: USB-A socket
* Display: 1306 OLED I2C
* Optocoupler: 6n137 (one that will work with 3.3v normally)

## Software

Arduino code. Plan to move to Platformio.
