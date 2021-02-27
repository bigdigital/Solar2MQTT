# What works:
- WiFi Ap with setup
- Web display Data for Inverters where support the QIPS Request
- Web Update, Reset, Restart
- send MQTT DATA

# ToDo:
- add mqtt refresh rate option
- store mqtt server settings in eprom
- some other useless things
- change webpages to dark theme
- add other inverters

**works with**
- pip / PCM devices
- i solar
- and many many others based on the chinese solar inverter with a rj45 jack and usb port, primary identified by the display


# Overview
Based around an ESP8266 WiFi microcontroller.
Software is built using [Arduino for ESP8266](https://github.com/esp8266/Arduino). It's the simplest way to get the toolchain.

**PROGRAMMING:** You need to program it before hooking it up to the TTL board! Im using HW Serial since its so buggy using Software Serial and Wifi reliable so its not even worth it.


**UART0:** Talks to the inverter at 2400 baud and for programming. DISCONNECT RX before programming

**UART1:** Used for debugging only. Just connect another serial adaptor to TX with 9600 baud.


**GPIO:** NOT IN USE Button on GPIO0, changes wifi mode, also used for programming when DTR pin isn't available.


**POWER:** Using a DC/DC or USB power currently. My gear dont have any power on the serial port

**WIFI:** The system defaults to AP mode on first setup. Surf to "SetSol" AP and then surf to http://192.168.4.1/ On this page configure WIFI and MQTT. Then you need to reboot the device either by the reboot botton or hard reboot. No changes are applied properly before reboot


# Parts required to build one

Most of the parts can be bought as modules, it's usually cheaper that way.

Approximate prices found on Ebay
- ESP8266 - Wemos D1 Mini ($2.66)
- MAX3232 module ($0.70) - it's just a max3232 with 5 capacitors on a tiny little board
- DC-DC buck module ($1.50) - 12-80v down to 5v
- LEDs ($0.50) - The leds are for status etc

Total cost: $5
