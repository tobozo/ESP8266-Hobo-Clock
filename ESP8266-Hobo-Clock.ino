/*************************************************** 
  The Hobo's Animated IoT Clock

  Manifesto: giving trust to unreliable time providers 
  isn't worse than trusting potentially unreliable 
  access points with personal credentials.

  This clock has not RTC module and doesn't need to 
  reach a NTP server to adjust its time. It uses WiFi 
  access points openness in order to extract the 
  date/time from the "Date" HTTP header that may be 
  sent by some captive portals. So it will scan all 
  nearby open WiFi AP until it finds a suitable response.

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

  Expecting unknown networks to provide a HTTP header 
  value and relying on it to estimate time is like 
  counting on other people's wealth to survive, hence 
  the Hobo name.

  The exclusive use of open access points removes the 
  hassle of hardcoding SSID/password into the sketch 
  but also compensates its lack of auth plus the fact 
  that the optional NTP connexion attempt will always 
  fail, unless the AP acts as.

  The Pong animation with a bouncing rotating cube is 
  there to cut on the boringness of the clock but also 
  to demonstrate how this tiny OLED can animate fast 
  (nearly 60fps).

  Since it has trust issues, don't trust this clock 
  more than you would trust a stranger's watch! The 
  available space and power consumption won't let it 
  run more than a couple of hours on the LiPo anyway.

  Ported to NodeMCU by tobozo (c+) Nov 2016

  https://youtu.be/RZ90ruADrI4
  
 ****************************************************/

// add some verbosity
#define DEBUG false
#define DEBUGGYRO false
#define DEBUGNTP false
#define DEBUGHTTP false
#define DEBUGWIFI false
#define DEBUGOTA false

// using a gyro for cube animation (enable this for wearable clock)
// 
#define USE_GYRO false

// hardcoding mac address, use this at home as if you're hosting your
// own hobo. NTP will be used in this case.
// Set to false if you encounter problems
#define HC_MAC true

#if USE_GYRO == true
  // https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
  #include "MPU6050_6Axis_MotionApps20.h"
#endif
#include <SSD1306.h>
#include <OLEDDisplayUi.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TimeLib.h>


// hardcoded mac case
#if HC_MAC == true
  extern "C" {
    #include "user_interface.h"
  }
  /* override init function to inject fake mac address */
  void initVariant() {
    // WARN: hardcoded mac address could create cross-device identity issues
    uint8_t mac[] = {0x60, 0x01, 0x94, 0x17, 0x8b, 0xb1};
    wifi_set_macaddr(STATION_IF, &mac[0]);
  }
  // also set hardcoded Hostname.
  const char* ESPName = "esp8266-178bb1";
  const char* ssid = "myhomewifi"; // Set this only if you're fool enough to hardcode it into an IoT
  const char* password = "myhomewifipassword"; // Set this only if you're fool enough to hardcode it into an IoT
#endif

// regular case
#if HC_MAC == false
  // automatic generation o
  const char* ESPName = "HOBO-CLOCK-" + String(ESP.getChipId(), HEX);
  const char* ssid = ""; // Set this only if you're fool enough to hardcode it into an IoT
  const char* password = ""; // Set this only if you're fool enough to hardcode it into an IoT
#endif


// OTA updatable as long as the bool is true
bool otaready = true;
int otawait = 20000; // how long to wait for OTA at boot (millisec)
int otanow; // time counter for OTA update


// Accel and gyro data
int16_t  ax, ay, az, gx, gy, gz;
double MMPI = 512*M_PI;

// used for cube animation loop
long int timeLast = -100, period = 1;
// Overall scale and perspective distance
uint8_t sZ = 4, scale = 6, scaleMax = 16;
// screen center
uint8_t centerX = 64;
uint8_t centerY = 32;
// Initialize cube point arrays
double C1[] = {  1,  1,  1 };
double C2[] = {  1,  1, -1 };
double C3[] = {  1, -1,  1 };
double C4[] = {  1, -1, -1 };
double C5[] = { -1,  1,  1 };
double C6[] = { -1,  1, -1 };
double C7[] = { -1, -1,  1 };
double C8[] = { -1, -1, -1 };
// Initialize cube points coords
uint8_t P1[] = { 0, 0 };
uint8_t P2[] = { 0, 0 };
uint8_t P3[] = { 0, 0 };
uint8_t P4[] = { 0, 0 };
uint8_t P5[] = { 0, 0 };
uint8_t P6[] = { 0, 0 };
uint8_t P7[] = { 0, 0 };
uint8_t P8[] = { 0, 0 };

// wifi scan progress bar
uint8_t progress;
int n = 0; // wifiscan index
int con = 0; // wifi connection indicator
// a host to visit in order to get the "Date:" HTTP header response
// in case the NTP query failed (most of the time it will)
const char* host = "google.com";
const char* pass = "";
String dateString = "";
bool dateprinted = false;


//Define Pins
#define OLED_RESET 4
#define BEEPER 3
#define CONTROL_A D1
#define CONTROL_B D2

//Define Visuals
#define FONT_SIZE 2
#define SCREEN_WIDTH 127  //real size minus 1, because coordinate system starts with 0
#define SCREEN_HEIGHT 63  //real size minus 1, because coordinate system starts with 0
#define PADDLE_WIDTH 4
#define PADDLE_HEIGHT 10
#define PADDLE_PADDING 10
#define BALL_SIZE 16
#define SCORE_PADDING 10

#define EFFECT_SPEED 1
#define MIN_Y_SPEED 1
#define MAX_Y_SPEED 2


int nstars=32;// Number of Stars
int ncrementer = 0;
double star[512][5];// Data structure to hold the position of all the stars
int w=127;// Width of the viewport (aka the body width)
int h=63;// Height of the viewport (aka the body height)
int x=64;// Center of the width of the viewport (width/2)
int y=32;// Center of the height of the viewport (height/2)
int z=5;// Hypothetical z-value representing where we are on the screen
int starRatio=8;// Just a constant effecting the way stars appear
int starSpeed=1;// The speed of the star. Yes, all star's have the same speed.
// Play around with the values for star speed, I noticed a cool effect if we made the star speed 0. Hence, I created a variable to save the star speed in those cases
int starSpeedPrev=0;
int opacity = 1;// Just a constant to hold the opacity
int cursorX=64;// Mouse Positions
int cursorY=32;// Mouse Positions


int paddleLocationA = 0;
int paddleLocationB = 0;

float ballX = SCREEN_WIDTH/2;
float ballY = SCREEN_HEIGHT/2;
float ballSpeedX = 2;
float ballSpeedY = 1;

int lastPaddleLocationA = 0;
int lastPaddleLocationB = 0;

int scoreA = 0;
int scoreB = 0;


// NTP variables
time_t prevDisplay = 0; // when the digital clock was displayed
int hours, minutes, seconds;  // for the results
int lastSecond = -1; // need an impossible value for comparison
int offset = 1; // timezone offset (relative to the ntp pool and geoposition)
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "fr.pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets


// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
SSD1306 display(0x3c, D3, D4);
OLEDDisplayUi ui ( &display );
#if USE_GYRO == true
  MPU6050 mpu;
#endif


void setstar(int i) {
 /* Initialize the stars.
  Since the ship is in the middle, we assume
  Each star has the following properties:
  1.[0] Actual X-coordinate of position in prespective of ship
  2.[1] Actual Y-coordinate of position in prespective of ship
  3.[2] Actual Z-coordinate of position in prespective of ship
  4.[3] Calculated X (represents X-coordinate on screen)
  5.[4] Calculated Y (represents Y-coordinate on screen)
  */
  star[i][0]=(random(100)*w*2-x*2)/100;
  star[i][1]=(random(100)*h*2-y*2)/100;
  star[i][2]=round(random(1)*z);
  star[i][3]=0;
  star[i][4]=0;
}


void starAnimate(){
  int mouseX=cursorX-x;
  int mouseY=cursorY-y;
  display.clear();

  if(ncrementer<nstars) {
    ncrementer++;
    setstar(ncrementer);
  }
  
  
  for(int i=0;i<ncrementer;i++){
    // Flag for if the star is offscreen (we don't want to draw it)

    //if(random(100) > 75) continue;
    
    bool test=true;
    /* Save the stars calculated position so we can use it for drawing */
    int starXPrev=star[i][3];
    int starYPrev=star[i][4];
    /* Update the Star */
    star[i][0]+=mouseX>>4;
    star[i][1]+=mouseY>>4;
    star[i][2]-=starSpeed;
    /* Check the boundary conditions to make sure stars aren't offscreen. */
    if(star[i][0]>x<<1){ 
      star[i][0]-=w<<1; 
      test=false; 
    } 
    if(star[i][0]<-x<<1){ 
      star[i][0]+=w<<1; 
      test=false;
    }
    if(star[i][1]>y<<1){ 
      star[i][1]-=h<<1; 
      test=false; 
    } 
    if(star[i][1]<-y<<1){ 
      star[i][1]+=h<<1; 
      test=false; 
    }
    if(star[i][2]>z){ 
      star[i][2]-=z; 
      test=false;
    } 
    if(star[i][2]<0){ 
      star[i][2]+=z; 
      test=false; 
    }
    // Our calculated position and where the star is going to be drawn on the screen
    star[i][3]=x + (star[i][0]/star[i][2]) * starRatio;
    star[i][4]=y + (star[i][1]/star[i][2]) * starRatio;
    // Actually draw the object, if the star isn't offscreen
    if( starXPrev>0 && starXPrev<w && starYPrev>0 && starYPrev<h 
     && star[i][3]>0 && star[i][4]<w && star[i][3]>0 && star[i][4]<h 
     && test ){
      // Note: all stars, even though appear the be dots, are actually drawn as lines
      // LineWidth is Calculated so that if the star is closer to the ship, make the star appear bigger
      // Draw a line from position 0 to position 1
      display.drawLine(starXPrev, starYPrev, star[i][3], star[i][4]);
    }
    
    if( starXPrev<0 || starXPrev>w || starYPrev<0 || starYPrev>h
     || star[i][3]<0 || star[i][4]>w || star[i][3]<0 || star[i][4]>h
      ) {
      setstar(i);
    }

  }

}


void swsetup() {

  // force stars out from screen center 
  //x=round(w/2);
  //y=round(h/2);
  
  z=(w+h)/2;
  //starColorRatio=1/z;
  // Initially we set the mouse to point to the middle of the viewport
  cursorX=x;
  cursorY=y;
  
  for(int i=0;i<nstars;i++){
    setstar(i);
  }
}


/* Vector Rotation calculations for x/y/z rotating cube points */
void vectRotXYZ(double angle, int axe) { 
  int8_t m1; // coords polarity
  uint8_t i1, i2; // coords index
  
  switch(axe) {
    case 1: // X
      i1 = 1; // y
      i2 = 2; // z
      m1 = -1;
    break;
    case 2: // Y
      i1 = 0; // x
      i2 = 2; // z
      m1 = 1;
    break;
    case 3: // Z
      i1 = 0; // x
      i2 = 1; // y
      m1 = 1;
    break;
  }

  double t1 = C1[i1];
  double t2 = C1[i2];
  C1[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C1[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);
  
  t1 = C2[i1]; 
  t2 = C2[i2];
  C2[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C2[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C3[i1]; 
  t2 = C3[i2];
  C3[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C3[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C4[i1]; 
  t2 = C4[i2];
  C4[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C4[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C5[i1]; 
  t2 = C5[i2];
  C5[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C5[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C6[i1]; 
  t2 = C6[i2];
  C6[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C6[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C7[i1]; 
  t2 = C7[i2];
  C7[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C7[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C8[i1]; 
  t2 = C8[i2];
  C8[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C8[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

}


void cubeloop() {

  period = (millis()- timeLast);
  timeLast = millis(); 
  // precalc
  double MMPI_TIME = MMPI*period;
  double interpolation = 1;

  #if USE_GYRO == true
    //Read gyro, apply calibration, ignore small values
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  #endif
  
  if(abs(gx)<20){
    gx = 800 * ballSpeedX;
  }
  if(abs(gy)<15){
    gy = 800 * ballSpeedY;
  }
  if(abs(gz)<30){
    gz = 400 * ballSpeedX*ballSpeedY;
  }
  
  // scale angles down, rotate
  vectRotXYZ((double)gy/MMPI_TIME, 1); // X
  vectRotXYZ((double)gx/MMPI_TIME, 2); // Y
  vectRotXYZ((double)-gz/MMPI_TIME, 3); // Z

  #if DEBUGGYRO == true
    Serial.print(scale);
    Serial.print("\t");  
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
  #endif

  // calculate each point coords
  P1[0] = centerX + scale/(1+C1[2]/sZ)*C1[0]; P1[1] = centerY + scale/(1+C1[2]/sZ)*C1[1];
  P2[0] = centerX + scale/(1+C2[2]/sZ)*C2[0]; P2[1] = centerY + scale/(1+C2[2]/sZ)*C2[1];
  P3[0] = centerX + scale/(1+C3[2]/sZ)*C3[0]; P3[1] = centerY + scale/(1+C3[2]/sZ)*C3[1];
  P4[0] = centerX + scale/(1+C4[2]/sZ)*C4[0]; P4[1] = centerY + scale/(1+C4[2]/sZ)*C4[1];
  P5[0] = centerX + scale/(1+C5[2]/sZ)*C5[0]; P5[1] = centerY + scale/(1+C5[2]/sZ)*C5[1];
  P6[0] = centerX + scale/(1+C6[2]/sZ)*C6[0]; P6[1] = centerY + scale/(1+C6[2]/sZ)*C6[1];
  P7[0] = centerX + scale/(1+C7[2]/sZ)*C7[0]; P7[1] = centerY + scale/(1+C7[2]/sZ)*C7[1];
  P8[0] = centerX + scale/(1+C8[2]/sZ)*C8[0]; P8[1] = centerY + scale/(1+C8[2]/sZ)*C8[1];

  // draw each cube edge
  display.drawLine(P1[0], P1[1], P2[0], P2[1]); //1-2
  display.drawLine(P1[0], P1[1], P3[0], P3[1]); //1-3
  display.drawLine(P1[0], P1[1], P5[0], P5[1]); //1-5
  display.drawLine(P2[0], P2[1], P4[0], P4[1]); //2-4
  display.drawLine(P2[0], P2[1], P6[0], P6[1]); //2-6
  display.drawLine(P3[0], P3[1], P4[0], P4[1]); //3-4
  display.drawLine(P3[0], P3[1], P7[0], P7[1]); //3-7
  display.drawLine(P4[0], P4[1], P8[0], P8[1]); //4-8
  display.drawLine(P5[0], P5[1], P6[0], P6[1]); //5-6
  display.drawLine(P5[0], P5[1], P7[0], P7[1]); //5-7
  display.drawLine(P6[0], P6[1], P8[0], P8[1]); //6-8
  display.drawLine(P7[0], P7[1], P8[0], P8[1]); //7-8

}


void addEffect(int paddleSpeed) {
  float oldBallSpeedY = ballSpeedY;

  //add effect to ball when paddle is moving while bouncing.
  //for every pixel of paddle movement, add or substact EFFECT_SPEED to ballspeed.
  for (int effect = 0; effect < abs(paddleSpeed); effect++) {
    if (paddleSpeed > 0) {
      ballSpeedY += EFFECT_SPEED;
    } else {
      ballSpeedY -= EFFECT_SPEED;
    }
  }

  //limit to minimum speed
  if (ballSpeedY < MIN_Y_SPEED && ballSpeedY > -MIN_Y_SPEED) {
    if (ballSpeedY > 0) ballSpeedY = MIN_Y_SPEED;
    if (ballSpeedY < 0) ballSpeedY = -MIN_Y_SPEED;
    if (ballSpeedY == 0) ballSpeedY = oldBallSpeedY;
  }

  //limit to maximum speed
  if (ballSpeedY > MAX_Y_SPEED) ballSpeedY = MAX_Y_SPEED;
  if (ballSpeedY < -MAX_Y_SPEED) ballSpeedY = -MAX_Y_SPEED;
}

void calculateMovement() {

  paddleLocationA = ballY - PADDLE_HEIGHT/2; //map(controlA, 0, 738, 0, SCREEN_HEIGHT - PADDLE_HEIGHT);
  paddleLocationB = ballY - PADDLE_HEIGHT/2; //map(controlB, 0, 738, 0, SCREEN_HEIGHT - PADDLE_HEIGHT);

  int paddleSpeedA = paddleLocationA - lastPaddleLocationA;
  int paddleSpeedB = paddleLocationB - lastPaddleLocationB;

  ballX += ballSpeedX;
  ballY += ballSpeedY;

  //bounce from top and bottom
  if (ballY >= SCREEN_HEIGHT - BALL_SIZE || ballY <= 0) {
    ballSpeedY *= -1;
  }

  //bounce from paddle A
  if (ballX >= PADDLE_PADDING && ballX <= PADDLE_PADDING+BALL_SIZE && ballSpeedX < 0) {
    if (ballY > paddleLocationA - BALL_SIZE && ballY < paddleLocationA + PADDLE_HEIGHT) {
      ballSpeedX *= -1;
      addEffect(paddleSpeedA);
    }
  }

  //bounce from paddle B
  if (ballX >= SCREEN_WIDTH-PADDLE_WIDTH-PADDLE_PADDING-BALL_SIZE && ballX <= SCREEN_WIDTH-PADDLE_PADDING-BALL_SIZE && ballSpeedX > 0) {
    if (ballY > paddleLocationB - BALL_SIZE && ballY < paddleLocationB + PADDLE_HEIGHT) {
      ballSpeedX *= -1;
      addEffect(paddleSpeedB);
    }
  }

  //score points if ball hits wall behind paddle
  if (ballX >= SCREEN_WIDTH - BALL_SIZE || ballX <= 0) {
    if (ballSpeedX > 0) {
      scoreA++;
      ballX = SCREEN_WIDTH / 4;
    }
    if (ballSpeedX < 0) {
      scoreB++;
      ballX = SCREEN_WIDTH / 4 * 3;
    }
  }

  //set last paddle locations
  lastPaddleLocationA = paddleLocationA;
  lastPaddleLocationB = paddleLocationB;  
}


// utility function for digital clock display: prints leading 0
String twoDigits(int digits) {
  if(digits < 10) {
    String i = '0'+String(digits);
    return i;
  }
  else {
    return String(digits);
  }
}


void draw() {
  display.clear(); 
  
  starAnimate();
  
  //draw paddle A
  display.fillRect(PADDLE_PADDING,paddleLocationA+ (scale/2),PADDLE_WIDTH,PADDLE_HEIGHT+ (scale/2));

  //draw paddle B
  display.fillRect(SCREEN_WIDTH-PADDLE_WIDTH-PADDLE_PADDING,paddleLocationB+ scale/2,PADDLE_WIDTH,PADDLE_HEIGHT+ (scale/2));

  //draw center line
  for (int i=0; i<SCREEN_HEIGHT; i+=4) {
    display.drawLine(SCREEN_WIDTH/2, i, SCREEN_WIDTH/2+2, i);
  }

  // ball/cube position
  centerX = ballX+(scale/2);
  centerY = ballY+(BALL_SIZE/2)+(scale/2);

  bool doblink = ((int)floor(millis()/100))%10<=5;

  if(progress!=0) {
    display.drawProgressBar(14, 27, 100, 10, progress);
  }

  if(dateString=="" && doblink) {
    // blink clock when not set
    cubeloop();
    display.display();
    return;  
  }

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, twoDigits(hour())+ "   " + twoDigits(minute()));

  if(!doblink) {
    // blink hh/mm separator
    display.fillCircle(64, 5, 1.5);
    display.fillCircle(64, 15, 1.5);
  }

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 45, (String)dayShortStr(weekday()) + ". " + day() + " " + monthShortStr(month()) + " " + year());
  cubeloop();
  display.display();
} 


void delayAnimate(int delay) {
  int now = millis();
  while( millis()-delay < now ) {
    calculateMovement();
    draw();
  }
}


void doScan() {
  n = WiFi.scanNetworks();
}


void tryWifiCon() {

  if(WiFi.encryptionType(n) != ENC_TYPE_NONE) {
    return; // only attempt to connect to Open AP
  }
  #if DEBUGWIFI==true
    Serial.print(n + 1);
    Serial.print(": ");
    Serial.print(WiFi.SSID(n));
    Serial.print(" (");
    Serial.print(WiFi.RSSI(n));
    Serial.print(")");
    Serial.print(" NOENC ");
  #endif
  WiFi.begin(WiFi.SSID(n).c_str(), pass);

  int comtimeout = 8000; // wait 8 sec max
  int now = millis();

  while (WiFi.status() != WL_CONNECTED) {
    int deltatime = millis()-now;
    progress = round(deltatime/80);
    delayAnimate(500);
    display.display();
    #if DEBUGWIFI==true
      Serial.print(".");
    #endif
    if(deltatime > comtimeout) {
      progress = 0;
      #if DEBUGWIFI==true
        Serial.print("*");
        Serial.println(" Timeout");
      #endif
      delayAnimate(10);
      return;
    }
  }
  progress = 0;
  con = n;
  #if DEBUGWIFI==true
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  #endif
  delayAnimate(10);
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  #if DEBUGNTP==true
    Serial.println("sending NTP packet...");
  #endif
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}


void tryHttpCon() {
  #if DEBUGHTTP==true
    Serial.print("connecting to ");
    Serial.println(host);
  #endif
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    #if DEBUGHTTP==true
      Serial.println("connection failed");
    #endif
    con = 0;
    return;
  }
  // We now create a URI for the request
  String url = "/";
  #if DEBUGHTTP==true
    Serial.print("Requesting URL: ");
    Serial.println(url);
  #endif
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      #if DEBUGHTTP==true
        Serial.println(">>> Client Timeout !");
      #endif
      client.stop();
      con = 0;
      return;
    }
  }
  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
    String line = client.readStringUntil('\r');

    if(line.indexOf("Date: ")>-1) {
      dateString = line;//.split("Date:");
      dateString.replace("Date: ", "");
    }
    #if DEBUGHTTP==true
      Serial.print(line);
    #endif
  }
  #if DEBUGHTTP==true
    Serial.println();
    Serial.println("closing connection");
  #endif
  con = 0;
}


time_t getNtpTime() {
  udp.begin(localPort);
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      #if DEBUGNTP==true
        Serial.println("Receive NTP Response");
      #endif
      udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + offset * SECS_PER_HOUR;
    }
    delayAnimate(1);
  }
  #if DEBUGNTP==true
    Serial.println("No NTP Response :-(");
  #endif
  return 0; // return 0 if unable to get the time
}


void tryNtpCon() {
  time_t ntpTime = getNtpTime();
  if(ntpTime!=0) {
    setTime((time_t) ntpTime);
    //adjustTime(offset * SECS_PER_HOUR);
    dateString = (String)dayShortStr(weekday()) + ". " + day() + " " + monthShortStr(month()) + " " + year();
    con = 0;
    dateprinted = true;
    #if DEBUGNTP==true
      Serial.print("[UDP] New Date: ");
      Serial.println(dateString);
    #endif
  }
}


void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10) {
    Serial.print('0');
  }
  Serial.print(digits);
}


int my_atoi(const char *s) {
  int sign=1;
  if(*s == '-')
    sign = -1;
  s++;
  int num=0;
  while(*s){
    num=((*s)-'0')+num*10;
    s++;   
  }
  return num*sign;
}


void adjustTimeFromHTTPHeaders() {
  // find the colons:
  int firstColon = dateString.indexOf(":");
  int secondColon= dateString.lastIndexOf(":");
  // get the substrings for hour, minute second:
  String hourString = dateString.substring(firstColon-2, firstColon);
  String minString = dateString.substring(firstColon+1, secondColon);
  String secString = dateString.substring(secondColon+1, secondColon+3);
  dateString = dateString.substring(0, firstColon-3)+" ";
  // convert to ints,saving the previous second:
  hours = hourString.toInt();
  minutes = minString.toInt();
  lastSecond = seconds;          // save to do a time comparison
  seconds = secString.toInt();
  // extract date info
  char str[] = "Sun. 10 nov 2016 "; // expected format
  static const char month_names[] = "ErrJanFebMarAprMayJunJulAugSepOctNovDec";
  char dlm[] = " "; // space delimited
  // convert to char array for easier splitting
  dateString.toCharArray(str, dateString.length()+1);
  // get date chunks
  int cnt = 0;
  char* tab[10] = { "ddd", "dd", "mmm", "yyyy" };
  // find chunks positions in array
  char *pch = strtok(str, dlm);
  // iterate and move chunks into tab
  while ( pch != NULL ) {
    if (cnt < 10) {
      tab[cnt++] = pch;
    } else {
      break;
    }
    pch = strtok(NULL, dlm);
  }
  // convert to ints
  int dday = ((String)tab[1]).toInt();
  int mmonth = (strstr(month_names, tab[2])-month_names)/3;
  int yyear = ((String)tab[3]).toInt();
  // set and adjust
  setTime(hours, minutes, seconds, dday, mmonth, yyear);
  adjustTime(offset * SECS_PER_HOUR);
  dateString = (String)dayShortStr(weekday()) + ". " + day() + " " + monthShortStr(month()) + " " + year();
  con = 0;
  dateprinted = true;
  #if DEBUGHTTP==true
    Serial.print("New Date: ");
    Serial.println(dateString);
  #endif
}



void setupOta() {
  Serial.begin(115200);

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "OTA Check");
  display.drawProgressBar(14, 27, 100, 10, 0);
  display.display();

  #if DEBUGOTA==true
    Serial.println("OTA Check");
    Serial.print("Setting Hostname: ");
    Serial.println(ESPName);
    Serial.print("MAC Address:");
    Serial.printf("%02x", WiFi.macAddress()[0]);
    Serial.print(":");
    Serial.printf("%02x", WiFi.macAddress()[1]);
    Serial.print(":");
    Serial.printf("%02x", WiFi.macAddress()[2]);
    Serial.print(":");
    Serial.printf("%02x", WiFi.macAddress()[3]);
    Serial.print(":");
    Serial.printf("%02x", WiFi.macAddress()[4]);
    Serial.print(":");
    Serial.printf("%02x", WiFi.macAddress()[5]);
    Serial.println();
  #endif
  
  //WiFi.hostname(ESPName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    #if DEBUGOTA==true
      Serial.println("Connection Failed! Aborting...");
    #endif
    display.clear();
    display.drawString(64, 0, "OTA Fail, aborting");
    display.display();
    delay(1000);
    otaready = false;
    return;
  }

  display.drawProgressBar(14, 27, 100, 10, 10);
  display.display();

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    #if DEBUGOTA==true
      Serial.println("Start updating " + type);
    #endif
  });
  ArduinoOTA.onEnd([]() {
    #if DEBUGOTA==true
      Serial.println("\nEnd");
    #endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #if DEBUGOTA==true
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
    display.clear();
    display.drawString(64, 0, "OTA Flashing");
    display.drawProgressBar(14, 27, 100, 10, 20);
    display.display();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    #if DEBUGOTA==true
      Serial.printf("Error[%u]: ", error);
    #endif
    display.clear();
    display.drawString(64, 0, "Error");
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    otaready = false;
  });

  ArduinoOTA.setHostname(ESPName);
  ArduinoOTA.begin();

  display.clear();
  display.drawProgressBar(14, 27, 100, 10, 20);
  display.drawString(64, 0, "OTA Ready");
  display.display();
  #if DEBUGOTA==true
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  #endif
  otanow = millis();
}



void wifiOff() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
}


void handleOta() {
  int then = millis();
  if(then - otawait < otanow) {
    int percent = 100-(ceil(then - otanow) / (otawait / 100))+1;
    display.drawProgressBar(14, 27, 100, 10, percent);
    display.display();
    ArduinoOTA.handle();
  } else {
    otaready = false;

    tryNtpCon(); // probably useless but who knows ?

    if (timeStatus()== timeNotSet) {

      tryHttpCon(); // only useful on captive portals (hoping they return Date: header)
      
      if(dateString!="" && !dateprinted) {
        // headers already received, no need to stay connected
        wifiOff();
        // Serial.println(dateString);
        adjustTimeFromHTTPHeaders();
      }
      
    } else {
      
      wifiOff();
      
    }
  }
}



void setup() {
  
  Serial.begin(115200);

  ui.setTargetFPS(60);
  display.init();
  display.clear();   // clears the screen and buffer
  display.display();   
  display.setColor(WHITE);
  display.clear(); 
  display.flipScreenVertically();
  #if USE_GYRO == true
    pinMode(A0, INPUT);
    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        #if DEBUGGYRO==true
          Serial.println("MPU initialized!");
        #endif
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(62);
        mpu.setYGyroOffset(100);
        mpu.setZGyroOffset(82);
        mpu.setZAccelOffset(-580); // 1688 factory default for my test chip
    }
  #endif
  
  delay(100);
  
  setupOta();
  swsetup();
  
  Serial.println("Setup done");
  
}


void loop() {

  if(otaready==true) {
    handleOta();
    return;
  }
  
  calculateMovement();
  draw();

  if (timeStatus()!= timeNotSet) {
    if (now() != prevDisplay) { //update the display only if the time has changed
      prevDisplay = now();
      // digital clock display of the time
      #if DEBUGNTP == true
        printDigits(hour());
        Serial.print(":");
        printDigits(minute());
        Serial.print(":");
        printDigits(second());
        Serial.print(" ");
        Serial.print(day());
        Serial.print(" ");
        Serial.print(month());
        Serial.print(" ");
        Serial.print(year());
        Serial.println();
      #endif 
    }
    // TODO: rescan every hour or so to calculate avg time leap
    return;
  }

  if(n<=0) {
    // get Wifi neigbourhood
    Serial.println("scan start");
    doScan();
    delayAnimate(5000);
    Serial.println("scan done");
    return;
  }

  while(con==0) {
    // iterate over Wifi neighbourhood
    tryWifiCon();
    n--;
    if(n>0) return;
  }

  if(con!=0) {
    // got and IP address!
    if(dateString=="") {
      tryNtpCon(); // probably useless
      if (timeStatus()== timeNotSet) {
        tryHttpCon(); // only useful on captive portals (hoping they return Date: header)
      }
    }
  }
  
  if(dateString!="" && !dateprinted) {
    // Date+Time HTTP headers have been received, no need to stay connected
    wifiOff();
    // Serial.println(dateString);
    adjustTimeFromHTTPHeaders();
  }

}

