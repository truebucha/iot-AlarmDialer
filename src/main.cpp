#include <Arduino.h>
#include <ArduinoOTA.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>

uint8_t ledPin = 2;

bool modemShouldRestart = true;
bool modemShouldConnect = true;
bool modemShouldHangup = false;
uint8_t modemConnectionAttemptsCount = 0;
uint8_t modemConnectionSequentalFailuresCount = 0;

String logStorage = String();

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

#define ussdBalance "*100#"
#define ussdNumber "*115#"

//#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon
#define CALL_TARGET "+375**45"
#define callDelaySec 30
// #define SMS_TARGET "+375**45"

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "YourAPN";
const char user[] = "";
const char pass[] = "";

/**************************************************************
 *
 * TinyGSM Getting Started guide:
 *   http://tiny.cc/tiny-gsm-readme
 *
 * NOTE:
 * Some of the functions may be unavailable for your modem.
 * Just comment them out.
 *
 **************************************************************/

// Select your modem:
#define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial1

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial

// or Software Serial on Uno, Nano
//#include <SoftwareSerial.h>
//SoftwareSerial SerialAT(2, 3); // RX, TX

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// ============================================

void cutLog() {

  if (logStorage.length() > 1000) {

    logStorage = logStorage.substring(100);
  }
}

void LOG(const String &s) {

  logStorage += s;
  cutLog();
}

void LOG(const char message[]) {

  logStorage += String(message);
  cutLog();
}

void LOG(char charOne) {

  logStorage += String(charOne);
  cutLog();
}

void LOG(int num) {

  logStorage += String(num);
  cutLog();
}

void LOG(long num) {

  logStorage += String(num);
  cutLog();
}

void LOG(double num) {

  logStorage += String(num);
  cutLog();
}

template <typename T>
void LOG(T t) {
  logStorage += String(t);
  cutLog();
}

template<typename T, typename... Args>
void LOG(T t, Args... args) {// recursive variadic function

    logStorage += String(t);
    cutLog();

    LOG(args...) ;
}

// =================================================================

template<class T>
int serialGSMAutoBaud(T& SerialAT,
                      bool swap = false,
                      uint32_t minimum = 9600,
                      uint32_t maximum = 115200) {

  static uint32_t rates[] = { 115200, 57600, 38400, 19200, 9600, 74400, 74880, 230400, 460800, 2400, 4800, 14400, 28800 };

  for (unsigned i = 0; i < sizeof(rates)/sizeof(rates[0]); i++) {
    uint32_t rate = rates[i];
    if (rate < minimum || rate > maximum) continue;

    LOG("\nTrying baud rate ", rate, "...");
    SerialAT.begin(rate);
    if (swap) {
      LOG("\nSwapping...");
      SerialAT.swap();
    }
    delay(10);
    for (int i=0; i<3; i++) {
      SerialAT.print("AT\r\n");
      String input = SerialAT.readString();
      if (input.indexOf("OK") >= 0) {
        LOG("\nModem responded at rate ", rate);
        return rate;
      }
    }
  }
  return 0;
}

// ====================================

int modemDelayCountdown = 0;

void applyModemDelay(int delay) {

  modemDelayCountdown = delay;
}

bool processingModemDelay() {

  delay(10);
  modemDelayCountdown -= 10;
 
  if (modemDelayCountdown < 0) {

    modemDelayCountdown = 0;
  }

  if (modemDelayCountdown == 0) {

    return false;
  }

  return true;
}

//========================

void hangup() {
   
    bool res = modem.callHangup();
    LOG("Hang up:", res ? "OK" : "fail");
    modemShouldHangup = false;
}

void call() {

  if (modemShouldHangup) {

    hangup();
  }

  #if defined(CALL_TARGET)

    bool res;

    LOG("Calling:", CALL_TARGET);

    // This is NOT supported on M590
    res = modem.callNumber(CALL_TARGET);

    LOG("Call:", res ? "OK" : "fail");

    if (res == false) {
      return;
    }

    applyModemDelay(callDelaySec * 1000);
    modemShouldHangup = true;

    // // Play DTMF A, duration 1000ms
    // modem.dtmfSend('A', 1000);

    // // Play DTMF 0..4, default duration (100ms)
      
    // for (char tone='0'; tone<='4'; tone++) {
    //   modem.dtmfSend(tone);
    // }
    
  #endif
}

void sms() {

 #if defined(SMS_TARGET)

  bool res = modem.sendSMS(SMS_TARGET, String("Hello from "));
  LOG("SMS:", res ? "OK" : "fail");

  // // This is only supported on SIMxxx series
  // res = modem.sendSMS_UTF16(SMS_TARGET, u"Привіііт!", 9);
  // DBG("UTF16 SMS:", res ? "OK" : "fail");

  #endif
}

// ============

String collectNetworkInfo() {
  String result = String();
  
  result += "\nCCID: ";
  result += modem.getSimCCID();

  result += "\nIMEI: ";
  result += modem.getIMEI();

  result += "\nOperator: ";
  result += modem.getOperator();
  
  result += "\nLocal IP: ";
  result += modem.getLocalIP();

  result += "\nSignal quality: ";
  result += modem.getSignalQuality();

  // This is NOT supported on M590
  result += "\nBattery lavel:";
  result += modem.getBattPercent();

  // This is only supported on SIMxxx series
  result += "\nBattery voltage:";
  result += String(modem.getBattVoltage() / 1000.0F);

  // This is only supported on SIMxxx series
  result += "\nGSM location:";
  result += modem.getGsmLocation();

  // result += "\nBalance (USSD): ";
  result += modem.sendUSSD(ussdBalance);

  // result += "\n Phone num: ";
  // result += modem.sendUSSD(ussdNumber);

  return result;
}

String stateString() {
  
  String result = String();

  result += (F("\ndevice config\n"));

  result += ("\nSketch size: ");
  result += String(ESP.getSketchSize());

  result += (F("\nFree size: "));
  result += String(ESP.getFreeSketchSpace());

  result += (F("\nFree Heap: "));
  result += String(ESP.getFreeHeap());

  #ifdef CALL_TARGET
  result += (F("\nAlarm Number: "));
  result += String(CALL_TARGET);
  #endif

  #ifdef SMS_TARGET
  result += (F("\nAlarm SMS: "));
  result += String(SMS_TARGET);
  #endif

  return result;
}

// =============================


void connectGSM();
void toggleLED();
void toggleAlarm();
void respondWithState();
void respondWithLog();
void toggleCall();
void toggleSMS();

// - setup -

void connectWifi() {
  
  WiFi.begin("puntodeacceso", "***");

  LOG("Connecting");
  int ms = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    ms += 500;
    LOG(".");
  }

  LOG("\nConnected after ");
  LOG(ms);
  LOG("ms, IP address: ");
  LOG(WiFi.localIP().toString());
}

void setupServer() {

  MDNS.begin("esp");

  httpUpdater.setup(&httpServer, "/update");

  // httpServer.on("/",[](){httpServer.send(200,"text/plain","Hello World!");});

  httpServer.on("/", respondWithState);
  httpServer.on("/log", respondWithLog);

  httpServer.on("/call", toggleCall);
  httpServer.on("/sms", toggleSMS);
  httpServer.on("/led", toggleLED);
  httpServer.on("/alarm", toggleAlarm);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
}

// - gsm -

void gsmRestart() {

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  LOG("\nRestartinging modem...");

  if (!modem.restart()) {
    applyModemDelay(10000);
    return;
  }
}

void gsmCouldConnect() {

  LOG("\nTrying Connect to Network...");
  if (!modem.waitForNetwork()) {
    
    LOG("\nWaiting for network...");
    applyModemDelay(10000);
    return;
  }
  
  delay(100);

  if (modem.isNetworkConnected()) {
    LOG("\nNetwork connected");
  } else {
    LOG("\nNetwork not connected");
  }
}

void connectApn () {

  LOG("\nConnecting to", apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    applyModemDelay(10000);
    return;
  }
}

void connectGSM() {

  modemConnectionAttemptsCount += 1;

  String modemInfo = modem.getModemInfo();
  LOG("\nModem: ", modemInfo);

  gsmCouldConnect();
  // connectApn();

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");
  modemShouldConnect = false;
}

void modemLogic() {

#if defined(TINY_GSM_MODEM_SIM808)
  modem.enableGPS();
  String gps_raw = modem.getGPSraw();
  modem.disableGPS();
  DBG("GPS raw data:", gps_raw);
#endif

  modem.gprsDisconnect();
  if (!modem.isGprsConnected()) {
    LOG("GPRS disconnected");
  } else {
    LOG("GPRS disconnect: Failed.");
  }

  // Try to power-off (modem may decide to restart automatically)
  // To turn off modem completely, please use Reset/Enable pins
  modem.poweroff();
  LOG("Poweroff.");

  // // Do nothing forevermore
  // while (true) {
  //   modem.maintain();
  // }
}

void setupPins() {

  pinMode(ledPin, OUTPUT);
}

// - life cycle -

void setup() {

  setupPins();
  setupServer();
  connectWifi();

  applyModemDelay(3000);
}

void gsmStart() {

  modemConnectionAttemptsCount = 0;
  modemConnectionSequentalFailuresCount = 0;

  // Set GSM module baud rate
  // int rate = serialGSMAutoBaud(SerialAT, true);
  // if (rate == 0) {
  //   LOG((F("\nGSM serial connection failed.")));
  // } else {
  //   LOG((F("\nGSM serial connected with baud rate = ")), rate);
  //   gsmRestart();
  //   modemShouldRestart = false;
  // }

  SerialAT.begin(9600);
  SerialAT.swap();

  SerialAT.print("AT\r\n");
  String input = SerialAT.readString();
  if (input.indexOf("OK") >= 0) {
    LOG((F("\nGSM serial connected with baud rate = 9600")));
    gsmRestart();
    modemShouldRestart = false;
  } else {
    LOG((F("\nGSM serial connection failed.")));
  }
}

void loop() {
  
  httpServer.handleClient();

  
  if (processingModemDelay()) {

    return;
  }  

  if (modemShouldHangup) {

    hangup();
    return;
  }

  if (modemShouldRestart) {

      gsmStart();
      return;
  }

  if (modemShouldConnect) {
 
    modemConnectionSequentalFailuresCount = 0;
    connectGSM();
    return;
  }

  if (modemConnectionAttemptsCount >= 10) {

    modemShouldRestart = true;
    return;
  }

  if (modemConnectionSequentalFailuresCount >= 10) {

    modemShouldConnect = true;    
    return;
  }

  RegStatus status = modem.getRegistrationStatus();
  LOG(status);
  
  if (modem.isNetworkConnected() == false) {

    modemConnectionSequentalFailuresCount += 1;
  } else {

    modemConnectionSequentalFailuresCount = 0;
    return;
  }
}

// - logic -



// - responds -

void respondWithState() {

  String response = stateString();

  if (modem.isNetworkConnected()) {
    response += "\n";
    response += collectNetworkInfo();
  }

  httpServer.send(200, (F("text/plain")), response.c_str() );
}

void respondWithLog() {

  String response = String(F("<html><body><p>"));
  response += String(F("<br/>modemDelay: "));
  response += String(modemDelayCountdown);
  response += String(F("<br/>Modem Should Restart: "));
  response += String(modemShouldRestart ? (F("YES")):(F("NO")));
  response += String(F("<br/>Modem Should Connect: "));
  response += String(modemShouldConnect ? (F("YES")):(F("NO")));
  response += String(F("<br/>Modem Should Hunup: "));
  response += String(modemShouldHangup ? (F("YES")):(F("NO")));
  response += String(F("<br/>Modem Network Connection Attempts: "));
  response += String(modemConnectionAttemptsCount);
  response += String(F("<br/>Modem Network Connection Sequental Failures Count: "));
  response += String(modemConnectionSequentalFailuresCount);
  response += "</p><pre>";
  response += logStorage;
  response += String("</pre></body></html>\n\r");
  
  httpServer.send(200, (F("text/html")), response.c_str());
}

void toggleSMS() {

  LOG("\n SMS initiated");

  sms();

  httpServer.send(200, (F("text/plain")), logStorage.c_str());
}

void toggleCall() {

  LOG("\n Call initiated");

  call(); 
  httpServer.send(200, (F("text/plain")), logStorage.c_str());
}

void toggleLED() {

  LOG(F("\ntoggle Led"));
  digitalWrite(ledPin,!digitalRead(ledPin));
  httpServer.send(204);
}

void toggleAlarm() {

  LOG(F("\ntoggle ALARM"));

  if (modemShouldHangup) {

    LOG((F("Already calling")));
    return;
  }

  sms();
  call();

  httpServer.send(204, "Ok");
}

