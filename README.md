# ESP8266-Hobo-Clock


* The Hobo's Animated IoT Clock
  
  Manifesto: giving trust to unreliable time providers isn't worse than trusting potentially unreliable access points with personal credentials.
  
  This clock has not RTC module and doesn't need to reach a NTP server to adjust its time. It uses WiFi access points openness in order to extract the date/time from the "Date" HTTP header that may be sent by some captive portals. So it will scan all  nearby open WiFi AP until it finds a suitable response.
  
  Breakout:
  - Wemos Mini D1 (ESP8266)
  - SSD1306 OLED (128x64 Monochrome)
  - TP4056 LiPo Charger
  - 3.7v LiPo (270mAh, tiny size)
  
  Boot sequence:
  - Enumerate Open WiFi AP
  - Get an IP address
  - Connect to captive portal
  - Look for a "Date" HTTP header (and against common 
    sense, trust its value)
  - Adjust the clock accordingly (at 0:40)
  
  Expecting unknown networks to provide a HTTP header value and relying on it to estimate time is like counting on other people's wealth to survive, hence the Hobo name.
  
  The exclusive use of open access points removes the hassle of hardcoding SSID/password into the sketch but also compensates its lack of auth plus the fact that the optional NTP connexion attempt will always fail, unless the AP acts as.
  
  The Pong animation with a bouncing rotating cube is there to cut on the boringness of the clock but also to demonstrate how this tiny OLED can animate fast (nearly 60fps).
  
  Since it has trust issues, don't trust this clock more than you would trust a stranger's watch! The available space and power consumption won't let it run more than a couple of hours on the LiPo anyway.
  
  Ported to NodeMCU by tobozo (c+) Nov 2016
  
  [![The Hobo's Animated IoT Clock](https://img.youtube.com/vi/RZ90ruADrI4/0.jpg)](https://www.youtube.com/watch?v=RZ90ruADrI4)

