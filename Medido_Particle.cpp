/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/home/davidmcq/particle/Medido_Particle_Peripheral/src/Medido_Particle.ino"
/*
 * Project Medido_Particle
 * Description: Control code for Medido Pump .. Particle Argon
 * Author: D. McQueeney
 * Date: 14-May-2020
 * Modified to be BLE Central 25-Nov-2020
 */

#include <Particle.h>
#include "Adafruit_SSD1306.h"
#include "Encoder.h"
#include "math.h"

void powerDownTimeout();
int manPumpSwitch();
void loop();
void resetOLED();
void setup();
void lineLCD0();
void lineLCD(int line, String text);
void lineLCDf(int line, String text, float val, String fmt, String sfx);
void lineLCDd(int line, String text, int val, String fmt, String sfx);
void showLCD();
void timeFmt(char* tstr, int tt);
double slope();
void gpioCBFill();
void gpioCBEmpty();
void gpioCBStop();
void setRunSpeed(int pw);
void setPumpSpeed(float ps);
void setPumpFwd();
void setPumpRev();
void sendSPI(String str, float val);
float adcVolts(float vRaw);
void timerCB();
void onLinefeed(String msg);
void execKwd(String k, String v);
void execCmd(String k, String v);
#line 14 "/home/davidmcq/particle/Medido_Particle_Peripheral/src/Medido_Particle.ino"
PRODUCT_ID(14118);
PRODUCT_VERSION(2);
//SYSTEM_MODE(MANUAL);
STARTUP(resetOLED());
SYSTEM_THREAD(ENABLED);

Encoder spdEnc(D2, D3);

#define OLED_RESET D6
#define PWM_FREQ 500

long spdEncPos;
long spdEncPosLast;
float spdEncPsi = 5.0;
bool manTouched = true; //false; this version is manual control only

int manPumpState;
int lastManPumpRead;
int priorPumpState;
int manPumpRead;
unsigned long manPumpTime;
unsigned long manPumpBounce = 5;
bool pumpEverZero = false;

int clrState;
int lastClrRead;
int clrRead;
unsigned long clrTime;

float PIDpGain = 0.0;
float PIDiGain = 1.0;
float PIDpTerm = 0;
float PIDiTerm = 0;
float MINpress = 0;
float MAXpress = 10;
float pressLimit = (MAXpress + MINpress) / 2;
float pressMult = 1.1; //10% overpressure to shut off
float lowBattery = 8.0; //suitable for LiFePo
float pulsePerOzFill = 84.3;
float pulsePerOzEmpty = 84.3;
int dispRstPin = D6;
int flowMeterPinFill = A2;  //A2   //D3;
int flowMeterPinEmpty = A3; // A3  //D2;
int flowMeterPinStop = A4;
int powerDownPin = D5;
int pwmPumpPin = D7;
int flowDirPin = D8;
volatile int pulseCountFill = 0;
volatile int pulseCountEmpty = 0;
volatile int pulseCountStop = 0;
volatile int pulseCountBad = 0;
volatile bool enablePump = false;
int pulseStop = 10; // set # pulses to stop pump on piss tank sensor @77 pulses/oz this is 3.5 ml
float flowRate = 0;
float integVolts = 0;
float integAmps = 0;
float integRPM = 0;
int lastPulseCountFill = 0;
int lastPulseCountEmpty = 0;
int lastPulseCountBad = 0;
int lastFlowTime = 0;
float pressZero;
float pressScale = 3.75;
float currentZero = 0.0;
int minPWM = 15;
int maxPWM = 255;
int opPWM = maxPWM;
int pumpPWM = 0;
int runPWM = 0;
float revSpd = 0;
float revSpdMax = 100;
float pressPSI = 0.0;
bool pumpFwd = true;
int saveSetSpeed = 100;
unsigned long pumpStartTime = 0;
unsigned long pumpStopTime = 0;
unsigned long lastShowDisplay = 0;
float runningTime = 0.0;
float flowCount = 0;
int pumpTimer = 0;
int watchTimer = 0;
int powerOffMins = 30;
unsigned long bootTime = 0;
int adcZero = 0;
float adcAvg = 0.0;
float adcScale = 1240.9; // 4095 / 3.3
float adcDiv = 1.666;    // resistive divider in front of Argon board adc from pressure sensor
int dw = 128;
int dh = 64;
int seq = 0;
unsigned long lastLoop = 0;
unsigned long loopMinTime = 20;
unsigned long lastTimerCB = 0;
unsigned long minTimerCB = 100; // was 200

const int maxSlopePoints = 10;
volatile int currSlopePoints = 0;
volatile unsigned long xx[maxSlopePoints];
volatile unsigned int yy[maxSlopePoints];
double dxx[maxSlopePoints];
double dyy[maxSlopePoints];

volatile int xyIdx = 0;

bool medidoEnabled = false;
bool haveDisplay = false;

bool imperial = true;
unsigned long lastmicros = 0;

const size_t UART_TX_BUF_SIZE = 100;  //20;
const size_t SCAN_RESULT_COUNT = 100; //20;

BleScanResult scanResults[SCAN_RESULT_COUNT];

const unsigned long SCAN_PERIOD_MS = 200; // was 2000

unsigned long lastScan = 0;

const unsigned long PRT_PERIOD_MS = 100; // was 1000
unsigned long lastPrt = 0;

BleCharacteristic peerMxCharacteristic;

uint8_t txBuf[UART_TX_BUF_SIZE];
size_t txLen = 0;

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context);
String textacc = "";
#define DISP_BUF_LEN 128
char textLCD[5][DISP_BUF_LEN];

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.

const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE, rxUuid, serviceUuid, onDataReceived, NULL);

float loopTimAvg = 0;
unsigned long loopTimLast = 0;
unsigned long loopDelta = 0;
int kk;
size_t mxch;
uint8_t mxcmd[1];

struct NVM
{
  uint8_t version;
  float CalF;
  float CalE;
  float Prs;
  float saveSetSpeed;
  float lowBattery;
  float pressMult;
  bool imperial;
  int pMAX;
};

NVM medidoNVM;

void powerDownTimeout()
{
  sendSPI("PowerDown", 0.0);
  lineLCD(1, "Timeout powerdown");
  showLCD();
  delay(2s); // wait for message to get to ios
  //Serial.println("timeout!");
  digitalWrite(powerDownPin, HIGH);
}

Timer powerTimer(1000 * 60 * powerOffMins, powerDownTimeout);

int manPumpSwitch()
{
  int manPumpFwd;
  int manPumpRev;
  if (digitalRead(D12) == HIGH)
  {
    manPumpFwd = 1;
  }
  else
  {
    manPumpFwd = 0;
  }

  if (digitalRead(D11) == HIGH)
  {
    manPumpRev = 1;
  }
  else
  {
    manPumpRev = 0;
  }
  return (manPumpRev + 2 * manPumpFwd);
}

int loopcnt = 0;
unsigned long sampleTime = 0;

// loop() runs over and over again, as quickly as it can execute.

void loop()
{
  loopcnt++;


  Particle.process();


  if (true) //(!BLE.connected())
  { // check manual controls only if BLE not connected

    clrRead = digitalRead(D4);

    if (clrRead != lastClrRead)
    {
      clrTime = millis();
    }

    lastClrRead = clrRead;

    if (millis() - clrTime > 50)
    {
      clrState = clrRead;
    }

    if (clrState == 0)
    {
      //lineLCD(1, "Clear");
      sendSPI("CLEAR!", 0.0);
      execCmd("Clear", "");
    }

    manPumpRead = manPumpSwitch();

    if (manPumpRead != lastManPumpRead)
    {
      manPumpTime = millis();
    }

    lastManPumpRead = manPumpRead;

    if ((millis() - manPumpTime) > manPumpBounce)
    {
      if (manPumpRead != manPumpState)
      {
        manPumpState = manPumpRead;
      }
    }

    if (manPumpState != priorPumpState)
    {
      if (priorPumpState == -1)
      {
        manPumpTime = 0;
        priorPumpState = 0;
      }
      if (manPumpState == 0)
      {
        //lineLCD(1, "Off");
        execCmd("Off", ""); 
        pumpEverZero = true;
      }
      else if (pumpEverZero and manPumpState == 1 and priorPumpState == 0)
      { // Off to Empty
        lineLCD(1, "Empty");
        char str[16];
        snprintf(str, sizeof(str), "%d", saveSetSpeed);
        execCmd("Spd", str);
        execCmd("Empty", "");
      }
      else if (pumpEverZero and manPumpState == 2 and priorPumpState == 0)
      { // Off to Fill
        char str[16];
        snprintf(str, sizeof(str), "%d", saveSetSpeed);
        execCmd("Spd", str);
        execCmd("Fill", "");
      }
      else
      {
        if (Serial.available())
        {
          Serial.printlnf("else: %d %d", manPumpState, priorPumpState);
        }
      }
    }

    priorPumpState = manPumpState;

    spdEncPos = spdEnc.read();
    if (spdEncPos != spdEncPosLast)
    {
      if (millis() - bootTime < 15000)
      { // if manual controls touched in first 15 secs, don't start BLE
        manTouched = true;
      }
      spdEncPsi = spdEncPsi + (float)(spdEncPosLast - spdEncPos) / 40.0;
      if (spdEncPsi > 15.0)
      {
        spdEncPsi = 15.0;
      }
      if (spdEncPsi < 0.0)
      {
        spdEncPsi = 0.0;
      }
      pressLimit = spdEncPsi;
      //PIDpGain = spdEncPsi * 10;
      lineLCDf(1, "Press Set ", spdEncPsi, "%.1f", " psi");
      powerTimer.start();     
      //showLCD();
    }
    spdEncPosLast = spdEncPos;
    //showLCD();
  }

  if (medidoEnabled)
  {
    if (loopTimLast != 0)
    {
      loopDelta = micros() - loopTimLast;
      loopTimAvg = loopTimAvg + (float)(loopDelta - loopTimAvg) / 10.0;
    }
    loopTimLast = micros();

    if (millis() - lastLoop > loopMinTime)
    {
      timerCB();
      lastLoop = millis();
      kk = 0;
    }
    else
    {
      kk = kk + 1;
    }
  }
}

Adafruit_SSD1306 display(OLED_RESET);

void resetOLED()
{
  // Setup a pin to reset the OLED display and do a clean hw reset
  pinMode(dispRstPin, OUTPUT);
  digitalWrite(dispRstPin, HIGH);
  delayMicroseconds(2000);
  digitalWrite(dispRstPin, LOW);
  delayMicroseconds(2000);
  digitalWrite(dispRstPin, HIGH);
}

double ddisp;

// setup() runs once, when the device is first turned on.
void setup()
{
  //bool haveDisplay = false;

  unsigned long tdisp;
  float fdisp;

  bootTime = millis();

  noInterrupts();
  currSlopePoints = 0;
  xyIdx = 0;
  interrupts();

  Serial.begin(115200);
  //Serial.begin(38400);

  //waitFor(Serial.isConnected, 10000); // comment this line out for production so it does not delay startup .. uncomment for debugging so we don't miss messages

  Serial.println("starting setup");
  String addrTxt = "";

  Serial.println("EEPROM");

  EEPROM.get(10, medidoNVM);

  Serial.print("Version: ");
  Serial.println(medidoNVM.version);
  Serial.print("pMAX: ");
  Serial.println(medidoNVM.pMAX);
  
  pinMode(D4, INPUT_PULLUP);
  bool startupReset = false;

  if (digitalRead(D4) == LOW)
  {
    startupReset = true;
  }

  //blank EEPROM is filled with 0xFF .. initialize if so

  //startupReset = true; // force reset for development .. comment out or remove

  if (startupReset == true) {
    medidoNVM.version = 0xFF; 
  }

  if (medidoNVM.version == 0xFF)
  {
    medidoNVM.version = 1;
    medidoNVM.CalF = pulsePerOzFill;
    medidoNVM.CalE = pulsePerOzEmpty;
    medidoNVM.lowBattery = lowBattery;
    medidoNVM.pressMult = pressMult;
    medidoNVM.Prs = pressLimit;
    medidoNVM.saveSetSpeed = saveSetSpeed;
    medidoNVM.imperial = true;
    medidoNVM.pMAX = maxPWM;
  }

  if (startupReset == true) {
    EEPROM.put(10, medidoNVM); 
  }

  // here if we have a valid version on read, or have initialized a blank
  // set up operational variables in case we will be runnnig manual

  pulsePerOzFill = medidoNVM.CalF;
  pulsePerOzEmpty = medidoNVM.CalE;
  lowBattery = medidoNVM.lowBattery;
  pressMult = medidoNVM.pressMult;
  pressLimit = medidoNVM.Prs;
  saveSetSpeed = medidoNVM.saveSetSpeed;
  imperial = medidoNVM.imperial;
  opPWM = medidoNVM.pMAX;

  
  tdisp = micros();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();                    // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  fdisp = (float)(micros() - tdisp);
  ddisp = (double)fdisp;

  // since there is no easy way to get display status from the adafruit api, time how long the init takes
  // typically it's a little over 20,000 us .. so if over 2x that we must not have a display

  if (fdisp < 40000.)
  {
    haveDisplay = true;
  }

  lineLCD0();
  lineLCD(1,  "Pump Ready");
  lineLCDf(2, "FillCal ", pulsePerOzFill, "%.1f", " ppo");
  lineLCDf(3, "EmptyCal ", pulsePerOzEmpty, "%.1f", " ppo");
  if (startupReset == true) {
    lineLCD(4,  "EEPROM reset");
  } else {
    lineLCD(4,  "BLE P: Fuel Pump");
  }
  lineLCD(5,  "V 0.1 04/24/24 DFM");
  showLCD();

  lastShowDisplay = millis();

  pinMode(D4, INPUT_PULLUP);    // pushbutton on rotary switch
  pinMode(D11, INPUT_PULLDOWN); // fwd/rev switch for manual operation (MISO, MOSI also D11,D12)
  pinMode(D12, INPUT_PULLDOWN);

  clrState = digitalRead(D4);
  lastClrRead = clrState;
  clrTime = 0;

  manPumpState = manPumpSwitch(); // read actual switch position
  lastManPumpRead = -1;
  priorPumpState = -1;
  manPumpTime = 0;

  powerTimer.start();

  char devname[80];
  String devID = System.deviceID();
  String devshort = devID.substring(devID.length() - 6);
  sprintf(devname, "Fuel Pump %s", devshort.c_str());
  BLE.setDeviceName(devname);
  BLE.on();
  BLE.selectAntenna(BleAntennaType::EXTERNAL);
  BLE.addCharacteristic(txCharacteristic);
  BLE.addCharacteristic(rxCharacteristic);

  BleAdvertisingData data;
  data.appendServiceUUID(serviceUuid);
  BLE.advertise(&data);

  //Currently we are using D7 for PWM .. this is a bad choice since its limited to 500Hz and 0-255

  pinMode(pwmPumpPin, OUTPUT);
  analogWriteResolution(pwmPumpPin, 8);
  analogWrite(pwmPumpPin, 0, PWM_FREQ);

  pinMode(flowDirPin, OUTPUT);

  // Preset pump speed to 0, set FWD direction

  setRunSpeed(0);
  setPumpSpeed(0);

  // Get a zero cal point on the current sensor

  for (int i = 1; i <= 10; i++)
  {
    currentZero = currentZero + (float)analogRead(A5);
  }

  currentZero = currentZero / 10.0;

  // Get a zero cal point on the pressure transducer

  int arsum = 0;
  int ar = 0;
  for (int i = 1; i <= 50; i++)
  {
    ar = analogRead(A0);
    arsum = arsum + ar;
  }
  pressZero = adcVolts((float)arsum / 50.0);

  // Set up interrupts to catch pulses from the flowmeters
  // experiment .. only attachInterrupt when about to run

  pinMode(flowMeterPinFill, INPUT);
  attachInterrupt(flowMeterPinFill, gpioCBFill, FALLING);

  pinMode(flowMeterPinEmpty, INPUT);

  pinMode(flowMeterPinStop, INPUT);

  // Setup power down pin, taking it high turns off the pump

  pinMode(powerDownPin, OUTPUT);
  digitalWrite(powerDownPin, LOW);

  sendSPI("Init", 0.0);
  sendSPI("rPWM", 0.0);

  // mainLoop.start();

  medidoEnabled = true;
}

void lineLCD0()
{
  for (int i = 0; i <= 4; i++)
  {
    strncpy(textLCD[i], "", DISP_BUF_LEN-1);
  }
}

void lineLCD(int line, String text)
{
  strcpy(textLCD[line - 1], text.c_str());
}

void lineLCDf(int line, String text, float val, String fmt, String sfx)
{
  sprintf(textLCD[line - 1], text + fmt + sfx, val);
}

void lineLCDd(int line, String text, int val, String fmt, String sfx)
{
  sprintf(textLCD[line - 1], text + fmt + sfx, val);
}

void showLCD()
{
  if (!haveDisplay)
  {
    return;
  }
  if (millis() - lastShowDisplay < 100)
  {
    return;
  }
  lastShowDisplay = millis();

  display.clearDisplay();
  display.setTextSize(1);
  //noInterrupts();
  for (int i = 0; i <= 4; i++)
  {
    display.setCursor(0, i * 13);
    display.println(textLCD[i]);
    delayMicroseconds(1000);
  }
  display.display();
  //interrupts();
  }


void timeFmt(char* tstr, int tt)
{

  int min;
  int sec;
  if (tt < 60)
  {
    sprintf(tstr, "Running Time %d sec", tt);
  }
  else
  {
    min = tt / 60;
    sec = tt - 60 * min;
    sprintf(tstr, "Running Time %2d:%02d", min, sec);
  }
  return;
}

double slope()
{
  double xbar = 0.0;
  double ybar = 0.0;
  double sxy = 0.0;
  double sx2 = 0.0;
  int csp = 0;
  
  noInterrupts(); // take the risk of missing a pulse to ensure data integrity - copy to local doubles

  if (currSlopePoints < 2) // can't make a line 
  {
    //sendSPI("currSlopePoints < 2", 0.0);

    interrupts();
    return (0.0);
  }
  csp = currSlopePoints;
  interrupts();

  for (int i = 0; i < csp; i++) // copy to nonvolatile variables and cast to double
  {
    dxx[i] = (double)xx[i];
    dyy[i] = (double)yy[i];
  }

  for (int i = 0; i < csp; i++)
  {
    if (micros() - dxx[i] > 1.E+6)
    { // defend against old points (>1s) stuck in buffer
      noInterrupts();
      currSlopePoints = 0;
      xyIdx = 0;
      interrupts();
      return (0.0);
    }
    xbar = xbar + dxx[i];
    ybar = ybar + dyy[i];
  }
  xbar = xbar / (double)csp;
  ybar = ybar / (double)csp;

  for (int i = 0; i < csp; i++)
  {
    sxy = sxy + ((dxx[i] - xbar) * (dyy[i] - ybar));
    sx2 = sx2 + ((dxx[i] - xbar) * (dxx[i] - xbar));
  }
  if (sx2 < 1.E-6)
  {
    sx2 = 1.E-6;
  }

  return (sxy / sx2);
}

void gpioCBFill()
{
  unsigned long mic = micros();

  // experiment: look for spurious pulses that occur too soon and ignore them
  // 16000 usec is about 60 oz/min
  // so in normal execution it should always be greater than that...
  if (mic - lastmicros < 16000) {
    return;
  }
  lastmicros = mic;

  if (pumpFwd)
  {
    pulseCountFill += 1;
    if (currSlopePoints < maxSlopePoints)
    {
      currSlopePoints += 1;
    }
    xx[xyIdx] = micros();
    yy[xyIdx] = pulseCountFill;
    if (xyIdx + 1 >= maxSlopePoints)
    {
      xyIdx = 0;
    }
    else
    {
      xyIdx += 1;
    }
  }
  else
  {
    //pulseCountBad += 1;
    pulseCountEmpty += 1;
    if (currSlopePoints < maxSlopePoints)
    {
      currSlopePoints += 1;
    }
    if (xyIdx + 1 >= maxSlopePoints)
    {
      xyIdx = 0;
    }
    else
    {
      xyIdx += 1;
    }
    xx[xyIdx] = micros();
    yy[xyIdx] = -pulseCountEmpty;
  }
  //interrupts(); // not requiured in ISR
}

void gpioCBEmpty()
{
  if (!pumpFwd)
  {
    noInterrupts();
    pulseCountEmpty += 1;
    if (currSlopePoints < maxSlopePoints)
    {
      currSlopePoints += 1;
    }
    if (xyIdx + 1 >= maxSlopePoints)
    {
      xyIdx = 0;
    }
    else
    {
      xyIdx += 1;
    }
    xx[xyIdx] = micros();
    yy[xyIdx] = -pulseCountEmpty;
  }
  else
  {
    pulseCountBad += 1;
  }
  interrupts();
}

void gpioCBStop()
{
  // code to stop goes here
}

void setRunSpeed(int pw)
{ // careful .. this is in pwm units 0 .. 1023 ..  not %
  pulseCountBad = 0;
  lastPulseCountBad = 0;

  analogWrite(pwmPumpPin, pw, PWM_FREQ);

  runPWM = pw;
}

int oldspd = 0;

void setPumpSpeed(float ps)
{                                      // this is ps units // 0 to 100%
  pumpPWM = (int)(ps * (float)opPWM / 100.0); //math.floor(ps*opPWM/100)
  if (pumpPWM < minPWM)
  {
    pumpPWM = 0;
  }
  if (pumpPWM > maxPWM)
  {
    pumpPWM = maxPWM;
  }
  
  oldspd = pumpPWM;
}

void setPumpFwd()
{
  digitalWrite(flowDirPin, LOW);
  pumpFwd = true;
  PIDiTerm = saveSetSpeed * minPWM / maxPWM; // setting to 1.0x saveSetSpeed caused overshoot
  pulseCountStop = 0;                        // reset piss tank flow sensor
  if (pumpPWM > 0)
  { // just turned on .. note startime
    pumpStartTime = millis();
  }
  //start the pump at min speed .. let the PID ramp it up
  setRunSpeed(minPWM);
}

void setPumpRev()
{
  digitalWrite(flowDirPin, HIGH);
  pumpFwd = false;
  PIDiTerm = 0;
  pulseCountStop = 0; // reset piss tank flow sensor
  revSpd = 0.0;
  if (pumpPWM > 0)
  { // just turned on .. note startime
    pumpStartTime = millis();
  }
  setRunSpeed(minPWM);
}

//
void sendSPI(String str, float val)
{
  if (BLE.connected())
  {
    {
      size_t nch;
      char buf[100];
      uint8_t txbuf[100];

      nch = sprintf(buf, "(" + str + ":%4.2f)", val);
      for (size_t ii = 0; ii < nch; ii++)
      {
        txbuf[ii] = (int)buf[ii];
      }
      // //peerRxCharacteristic.setValue(txbuf, nch);
      txCharacteristic.setValue(txbuf, nch);
    }
  }
}

float adcVolts(float vRaw)
{
  float v;
  v = adcDiv * (vRaw - (float)adcZero) / adcScale;
  return v;
}

void timerCB()
{

  float pSpd;
  float errsig;
  unsigned long now;
  unsigned long dtms;
  float vbatt;
  bool temp;
  static float current = 0;

  if (millis() - lastTimerCB < minTimerCB)
  {
    return;
  }

  lastTimerCB = millis();

  float loopD;
  if (loopDelta < 200) {
    loopD = (float)loopDelta;
  } else {
    loopD = 120;
  }

  vbatt = 7.504 * (float)analogRead(A1) * 0.5644 * 3.3 / 4096.;
  
  if (vbatt < lowBattery) {
    lineLCD(1, "Low Battery - pwr off");
    showLCD();
    sendSPI("power down", 0.0);
    delay(5s);
    execCmd("PwrOff", "");
  }


  if (pulseCountStop > pulseStop)
  {
    sendSPI("pSTP", (float)pulseCountStop);
    pulseCountStop = 0;
  }

  adcAvg = adcAvg - (adcAvg - (float)analogRead(A0)) / 2.0; // running avg of raw adc readings
  pressPSI = (adcVolts(adcAvg) - pressZero) * (pressScale);
  if (false) { 
    sendSPI("pPSI", pressPSI);
  }
  int a5 = 0;
  for(int i=0; i < 10; i++) {
    a5 += analogRead(A5);
  }
  a5 = a5 / 10;

  // current sense port is 20mV/A, 50 is 1/0.020
  // observered fudge factor of 0.7 (Pololu spec sheet says "about 20mV/A"...)
  
  current = current + (0.7 * 50 * ((3.3 / 4096.0) * ((float)a5 - currentZero)) - current) / 5;

  if (runPWM > 0)
  {
    if (pumpFwd)
    {
      integAmps += ((loopD / 1.0E6) / 3600) * 1000 * current;
      integRPM += (loopD / 1.0E6) * (((float)runPWM/(float)maxPWM) * vbatt - 0 * current);
    }
    else
    {
      integAmps -= ((loopD / 1.0E6) / 3600) * 1000 * current;
      integRPM -= (loopD / 1.0E6) * (((float)runPWM/(float)maxPWM) * vbatt - 0 * current);
    }
  }

  if (pressPSI > pressLimit * pressMult) {
    execCmd("Off", "");
    lineLCDf(1, "Pump Off - ", pressPSI, "%.1f", " psi");    
  }


  if ((pumpFwd && (runPWM > 0)) && (PIDiGain != 0 or PIDpGain != 0))
  {
    errsig = pressLimit - pressPSI;
    if (errsig < 1.0 && errsig > -1.0) { 
      PIDpTerm = errsig * PIDpGain;
    } else {
      PIDpTerm = 0.0;
    }

    PIDiTerm = PIDiTerm + errsig * PIDiGain;
    if (PIDiTerm < 0.0)
    {
      PIDiTerm = 0.0;
    }
    if (PIDiTerm > 100.0)
    {
      PIDiTerm = 100.0;
    }
    float rsec = (float)(millis() - pumpStartTime) / 1000;

    //soft start and let pump get a change to prime itself without slamming on

    if (rsec < 20.0) {
      sendSPI("startup", rsec);
      if (PIDiTerm > rsec * 4 + 20) {
        PIDiTerm = rsec * 4 + 20;
        sendSPI("LIMIT", PIDiTerm);
      }
    }

    pSpd = PIDpTerm + PIDiTerm;
    if (pSpd < 0.0)
    {
      pSpd = 0.0;
    }
    if (pSpd > saveSetSpeed)
    {
      pSpd = saveSetSpeed;
    }

    noInterrupts(); //enablePump is volatile
    temp = enablePump;
    interrupts();
    if (not temp)
    { // in case cmd came in asynch
      setPumpSpeed(0.0);
      setRunSpeed(0);
    }
    else
    {
      if (runPWM != 0)
      {
        setPumpSpeed(pSpd);   // side effect: sets pumpPWM within bounds
        setRunSpeed(pumpPWM); // side effect: sets runPWM
      }
    }
  }

  flowCount = ((float)pulseCountFill / pulsePerOzFill) - ((float)pulseCountEmpty / pulsePerOzEmpty);
  
  //loopDelta seems to average around 120us but .. takes large excursions presumably when doing particle tasks
  //presumably we need to use the actual amount since the voltage spends that long at the value

  if (runPWM > 0)
  {
    if (pumpFwd)
    {
      integVolts += (loopD / 1.0E6) * ((float)runPWM) * vbatt / 100.0;
    }
    else
    {
      integVolts -= (loopD / 1.0E6) * ((float)runPWM) * vbatt / 100.0;
    }
  }

  now = millis();

  if (pumpStartTime == 0)
  {
    pumpStartTime = now;
  }

  dtms = now - lastFlowTime;
  dtms = dtms;
  //  if needed port this line: if dtms < 0 then dtus = dtus + 2147483647 end // in case of rollover
  if (dtms > 1000)
  {
    if (pulseCountBad != lastPulseCountBad)
    {
      sendSPI("cBAD", (float)pulseCountBad);
      lastPulseCountBad = pulseCountBad;
    }

    if (pumpFwd)
    {
      flowRate = slope() * 1.E+6 * 60.0 / pulsePerOzFill; // convert from counts per microsecond (oz/m)in
    }
    else
    {
      flowRate = slope() * 1.E+6 * 60.0 / pulsePerOzEmpty; // convert from counts per microsecond oz/min
    }

    lastPulseCountFill = pulseCountFill;
    lastPulseCountEmpty = pulseCountEmpty;
    lastFlowTime = now;
  }

  if (runPWM > 0)
  {
    if (now > pumpStartTime)
    {                                                      //sometimes can have few 100 ms skew that makes now - pumpStartTime negative .. just set to zero
      runningTime = (float)(now - pumpStartTime) / 1000.0; // secs
    }
    else
    {
      runningTime = 0.0;
    }
    if (runningTime > 100000.0 || runningTime < 0.0)
    {
      //Serial.printlnf("runPWM > 0: runPWM=%d, now=%lu, pumpStartTime=%lu, runningTime=%f", runPWM, now, pumpStartTime, runningTime);
    }
  }
  else
  {
    if (pumpStopTime > pumpStartTime)
    {
      runningTime = (float)(pumpStopTime - pumpStartTime) / 1000.0;
    }
    else
    {
      runningTime = 0.0;
    }
  }

  // dont send time if pump not on .. confuses graph on iPhone

  if (false) // //(runPWM != 0)
  {
    sendSPI("rTIM", runningTime);
  }

  if (!pumpFwd && (runPWM > 0))
  {
    revSpd = revSpd + (revSpdMax - revSpd) / 6.0; //soft start... don't slam on
    if (runPWM != 0)
    {
      noInterrupts(); // enablePump is volatile
      temp = enablePump;
      interrupts();

      if (temp)
      {
        setPumpSpeed(revSpd);
        setRunSpeed(pumpPWM);
      }
      else
      {
        setPumpSpeed(0.0);
        setRunSpeed(0);
      }
    }
  }

  seq = seq + 1;

  if (millis() - bootTime > 4000)
  {
    if (imperial)
    {
      lineLCDf(2, "Flowrate ", flowRate, "%.1f", " oz/m");
      lineLCDf(3, "Flow ", flowCount, "%.1f", " oz");
    }
    else
    {
      lineLCDf(2, "Flowrate ", flowRate * 0.02957, "%.2f", " L/m");
      lineLCDf(3, "Flow ", flowCount * 0.02957, "%.2f", " L");
    }

    char tf[40];

    if (runPWM == 0)
    {
      timeFmt(tf, 0);
      lineLCD(4, tf);
    }
    else
    {
      timeFmt(tf, runningTime);
      lineLCD(4, tf);
    }
    
    if (runPWM > 0) {
      lineLCDf(5, "Pressure ", pressPSI, "%2.1f", " psi"); 
    } else {
      lineLCDf(5, "Battery ", vbatt, "%2.1f", " V"); 
    }
    
    if (true) { //(seq % 2 == 0) {
      showLCD();
    }
  }
}

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context)
{
  for (size_t ii = 0; ii < len; ii++)
  {
    textacc.concat(char(data[ii]));
    if (data[ii] == '\n')
    {
      onLinefeed(textacc);
    }
  }
}

void onLinefeed(String msg)

{
  int lparen;
  int rparen;
  int colon;
  String key;
  String val;
  bool isKwd = false;

  lparen = msg.indexOf("(");
  if (lparen < 0)
  {
    isKwd = true;
    rparen = msg.length() - 1;
  }
  else
  {
    rparen = msg.indexOf(")");
  }
  colon = msg.lastIndexOf(":");

  if (colon < 0)
  {
    key = msg.substring(lparen + 1, rparen);
    val = "";
  }
  else
  {
    key = msg.substring(lparen + 1, colon);
    val = msg.substring(colon + 1, rparen);
  }

  textacc = "";

  if (isKwd)
  {
    execKwd(key, val);
  }
  else
  {
    execCmd(key, val);
  }
}

String wifissid = "";
String wifipwd = "";
String wifihost = "";
String wifidir = "";
String wifiimage = "";

void execKwd(String k, String v)
{
  String tmp = "";

  if (k == "ssid")
  {
    wifissid = v.c_str();
  }
  else if (k == "pwd")
  {
    wifipwd = v.c_str();
  }
  else if (k == "host")
  {
    wifihost.concat(v);
  }
  else if (k == "dir")
  {
    wifidir.concat(v);
  }
  else if (k == "image")
  {
    wifiimage.concat(v);
  }
  else if (k == "stop")
  {
    //Serial.println("Stop cmd received");
  }
  else if (k == "unpair")
  {
    //Serial.println("unpair cmd");
    lineLCD(1, "Unpair");
  }
  else if (k == "update")
  {
    Serial.println("Update command received");
    lineLCD(1, "OTA update request");
    sendSPI("OTA", 1); // Starting WiFi
    if (wifissid != "" && wifipwd != "")
    {
      Serial.println("got some nonblank credentials");
      Serial.println(wifissid);
      Serial.println(wifipwd);
      WiFi.setCredentials(wifissid, wifipwd);
          if (WiFi.hasCredentials()){
        Serial.println("device reports it has credentials");
      }
    }
    else // perhaps device has stored credentials we can use
    {
      Serial.println("got blank credentials");
    }

    WiFi.on();
    WiFi.connect();

    int wLoops = 0;
    while (!WiFi.ready() && wLoops < 1200)
    { //timeout if no wifi or bad creds
      delay(100);
      wLoops = wLoops + 1;
    }
    if (wLoops >= 1200)
    {
      lineLCD(1, "No WiFi Connection");
      sendSPI("OTA", -1); //No WiFi connection

      return;
    }
    lineLCD(1, "WiFi Connected");
    sendSPI("OTA", 2); // WiFi connected
    Particle.connect();
    wLoops = 0;
    while (!Particle.connected() && wLoops < 300)
    {
      Particle.process();
      delay(100);
      wLoops = wLoops + 1;
    }
    if (wLoops >= 300)
    {
      lineLCD(1, "No Cloud Connection");
      sendSPI("OTA", -30); // No Particle Cloud Connection
      return;
    }
    if (Particle.connected()) {
      lineLCD(1, "Cloud Connected");
      sendSPI("OTA", 30); // particle cloud connected
      System.enableUpdates();
    }
    if (System.updatesPending()) {
      lineLCD(1, "Update available");
      sendSPI("OTA", 40);
    } else {
      lineLCD(1, "No Update Avail");
    }
  }
}

void execCmd(String k, String v)
{

  // each time we get any command, restart the poweroff timer

  float vv = atof(v.c_str());

  powerTimer.start(); // this will reset it if already running

  if (k == "Spd")
  { // what change was it?
    saveSetSpeed = vv;
    setPumpSpeed(saveSetSpeed);
    if (saveSetSpeed == 0)
    {
      lineLCD(1, "Pump Off");
    }
    medidoNVM.saveSetSpeed = saveSetSpeed;
    // // // showLCD();
  }
  else if (k == "Fill")
  {
    noInterrupts(); // these three are volatile
    enablePump = true;
    currSlopePoints = 0;
    xyIdx = 0;
    interrupts();
    setPumpFwd();
    lineLCD0();
    lineLCDf(1, "Fill - Limit ", pressLimit * pressMult, "%.1f", " psi");    
  }
  else if (k == "Off")
  {
    noInterrupts();
    enablePump = false;
    interrupts();
    setPumpSpeed(0.0);
    setRunSpeed(0);
    pumpStopTime = millis();

    lineLCD(1, "Pump Off");
  }
  else if (k == "Empty")
  {
    revSpdMax = saveSetSpeed;
    noInterrupts();
    currSlopePoints = 0;
    xyIdx = 0;
    enablePump = true;
    interrupts();
    setPumpRev();
    lineLCD0();
    lineLCD(1, "Empty");
  }
  else if (k == "Clear")
  {
    noInterrupts();
    pulseCountFill = 0;
    pulseCountEmpty = 0;
    pulseCountBad = 0;
    interrupts();

    lastPulseCountFill = 0;
    lastPulseCountEmpty = 0;
    lastPulseCountBad = 0;

    flowCount = 0;
    integVolts = 0;
    integAmps = 0;
    integRPM = 0;
    runningTime = 0;
    pumpStartTime = 0;
    pumpStopTime = 0;
    //Serial.println("Clear");
    sendSPI("rTIM", runningTime);
    lineLCD(1, "Clear");
  }
  else if (k == "CalF" && vv > 0.0)
  {
    pulsePerOzFill = vv;
    medidoNVM.CalF = pulsePerOzFill;
  }
  else if (k == "CalE" && vv > 0.0)
  {
    pulsePerOzEmpty = vv;
    medidoNVM.CalE = pulsePerOzEmpty;
  }
  else if (k == "LowB" && vv > 0.0)
  {
    lowBattery = vv;
    medidoNVM.lowBattery = lowBattery;
  }
  else if (k == "Pmult" && vv > 0.0)
  {
    pressMult = vv;
    medidoNVM.pressMult = pressMult;

  }
  else if (k == "Prs" && vv > 0.0)
  {
    pressLimit = vv;
    if (pressLimit > MAXpress)
    {
      pressLimit = MAXpress;
    }
    if (pressLimit < MINpress)
    {
      pressLimit = MINpress;
    }
    medidoNVM.Prs = pressLimit;
  }
  else if (k == "PwrOff")
  {
    digitalWrite(powerDownPin, HIGH);
  }
  else if (k == "pMAX")
  {
    opPWM = atof(v.c_str());
    if (opPWM > maxPWM)
    {
      opPWM = maxPWM;
    }
    if (opPWM < minPWM)
    {
      opPWM = minPWM;
    }
    medidoNVM.pMAX = opPWM;
  }
  else if (k == "Imp")
  {
    imperial = true;
    medidoNVM.imperial = true;
  }
  else if (k == "Met")
  {
    imperial = false;
    medidoNVM.imperial = false;
  }
  else if (k == "Sav")
  {
    EEPROM.put(10, medidoNVM);
    lineLCD(1, "EEPROM saved");
  }
  else
  {
    sendSPI("CalF CalE LowB Pmult Prs Imp Met Sav", 0.0);
    Serial.printlnf("execCmd cmd error: k,v = %s,%s", k.c_str(), v.c_str());
  }
}
