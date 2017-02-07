# ESP8266-Hobo-Clock
This clock has not RTC module and doesn't need to reach a NTP server to adjust its time. It uses WiFi access points openness in order to extract the date/time from the "Date" HTTP header that may be sent by some captive portals. So it will scan all nearby open WiFi AP until it finds a suitable response.
