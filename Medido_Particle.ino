/*
 * Project Medido_Particle
 * Description: Control code for Medido Pump .. Particle Argon
 * Author: D. McQueeney
 * Date: 14-May-2020
 * Modified to be BLE Central 25-Nov-2020
 */

#include <Particle.h>
#include "Adafruit_SSD1306.h"

#define OLED_RESET D6

SYSTEM_MODE(MANUAL);
STARTUP(resetOLED());

//SYSTEM_THREAD(ENABLED);

float PIDpGain = 0.0;
float PIDiGain = 0.5;
float PIDpTerm = 0;
float PIDiTerm = 0;
float MINpress = 0;
float MAXpress = 10;
float pressLimit = (MAXpress + MINpress) / 2;
float pulsePerOzFill = 80.0;
float pulsePerOzEmpty = 80.0;
int dispRstPin = D6;
int flowMeterPinFill = D3;  //A2;
int flowMeterPinEmpty = D2; // A3;
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
int lastPulseCountFill = 0;
int lastPulseCountEmpty = 0;
int lastPulseCountBad = 0;
int lastFlowTime = 0;
float pressZero;
float pressScale = 3.75;
float currentZero = 0.0;
int minPWM = 50;
int maxPWM = 1023;
int opPWM = maxPWM;
int pumpPWM = 0;
int runPWM = 0;
float revSpd = 0;
float revSpdMax = 100;
float pressPSI = 0.0;
bool pumpFwd = true;
int saveSetSpeed = 0;
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
volatile unsigned long lastFillMicro = 0;
volatile unsigned long lastEmptyMicro = 0;
volatile int lastFillShort = 0;
volatile int lastEmptyShort = 0;

bool medidoEnabled = false;
bool haveDisplay = false;

const size_t UART_TX_BUF_SIZE = 100;  //20;
const size_t SCAN_RESULT_COUNT = 100; //20;

BleScanResult scanResults[SCAN_RESULT_COUNT];

const unsigned long SCAN_PERIOD_MS = 200; // was 2000
unsigned long lastScan = 0;

const unsigned long PRT_PERIOD_MS = 100; // was 1000
unsigned long lastPrt = 0;

BleCharacteristic peerTxCharacteristic;
BleCharacteristic peerRxCharacteristic;
BleCharacteristic peerMxCharacteristic;

//BleCharacteristic peertxUuidharacteristic;

BlePeerDevice peer;

uint8_t txBuf[UART_TX_BUF_SIZE];
size_t txLen = 0;

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context);
String textacc = "";
#define DISP_BUF_LEN 64
char textLCD[5][DISP_BUF_LEN];

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.

const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

//const BleUuid serviceUuid("331A36F5-2459-45EA-9D95-6142F0C4B307");
//const BleUuid rxUuid("A9DA6040-0823-4995-94EC-9CE41CA28833");
//const BleUuid txUuid("A73E9A10-628F-4494-A099-12EFAF72258F");
//const BleUuid mxUuid("75a9f022-af03-4e41-b4bc-9de90a47d50b");
//                      a73e9a10-628f-4494-a099-12efaf72258f


BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE, rxUuid, serviceUuid, onDataReceived, NULL);
//BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, serviceUuid, onDataReceived, NULL);
//BleCharacteristic mxCharacteristic("mx", BleCharacteristicProperty::WRITE, mxUuid, serviceUuid);

float loopTimAvg = 0;
unsigned long loopTimLast = 0;
unsigned long loopDelta = 0;
int kk;
size_t mxch;
uint8_t mxcmd[1];

uint8_t connBLE[6];

void powerDownTimeout()
{
  sendSPI("PowerDown", 0.0);
  delay(1s); // wait for message to get to ios
  //Serial.println("timeout!");
  digitalWrite(powerDownPin, HIGH);
}

Timer powerTimer(1000 * 60 * powerOffMins, powerDownTimeout);

//Timer mainLoop(200, timerCB);

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  //if (Particle.connected()) {
    Particle.process(); 
  //}
  

  if (medidoEnabled)
  {
    if (loopTimLast != 0)
    {
      loopDelta = micros() - loopTimLast;
      //Serial.println(loopDelta);
      loopTimAvg = loopTimAvg + (float)(loopDelta - loopTimAvg) / 10.0;
      //Serial.println(loopTimAvg);
    }
    loopTimLast = micros();

    //Serial.print(millis());
    //Serial.print(lastLoop);
    //Serial.println(millis() - lastLoop);

    if ((int)((unsigned long)millis() - lastLoop) > (int)loopMinTime)
    {

      //Serial.print("Loop avg time: ");
      //Serial.println((float)loopTimAvg);
      //Serial.print(millis());
      //Serial.print(" ");
      //Serial.println((unsigned long)millis() - lastLoop);
      timerCB();

      lastLoop = millis();
      //Serial.print("kk: ");
      //Serial.println(kk);
      kk = 0;
    }
    else
    {
      kk = kk + 1;
    }

    if (BLE.connected() && medidoEnabled)
    {
      //Serial.println("BLE connected");
      //if (millis() - lastPrt >= PRT_PERIOD_MS)
      //{
        //for (size_t ii = 0; ii < 1; ii++)
        //{
        //  txBuf[ii] = 65;
        //}
        //txLen = 1;
        //peerRxCharacteristic.setValue(txBuf, txLen);
        //Serial.println("PRT!");
        //sendSPI("FOO", 137.0);
        //lastPrt = millis();
        //txLen = 0;
      //}
      //while (Serial.available() && txLen < UART_TX_BUF_SIZE)
      //{
      //  txBuf[txLen++] = Serial.read();
      //  Serial.write(txBuf[txLen - 1]);
      //}
      //Serial.print("txLen: ");
      //Serial.println(txLen);
      //if (txLen > 0)
      //{
      // Transmit the data to the BLE peripheral
      //Serial.print("txLen: ");
      //Serial.println(txLen);
      //peerRxCharacteristic.setValue(txBuf, txLen);

      //txLen = 0;
      //Serial.println(txLen);
      //}
      //uint8_t txBuf[UART_TX_BUF_SIZE];
      //size_t txLen = 0;

      //while (Serial.available() && txLen < UART_TX_BUF_SIZE)
      //{
      //  txBuf[txLen++] = Serial.read();
      //  Serial.write(txBuf[txLen - 1]);
      //}
      //if (txLen > 0)
      //{
      //  txCharacteristic.setValue(txBuf, txLen);
      //}
    }
    else
    {
      if (millis() - lastScan >= SCAN_PERIOD_MS)
      {
        // Time to scan
        lastScan = millis();

        size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);
        //Serial.printlnf("Scan Count: %d", count);

        if (count > 0)
        {
          for (uint8_t ii = 0; ii < count; ii++)
          {
            // Our serial peripheral only supports one service, so we only look for one here.
            // In some cases, you may want to get all of the service UUIDs and scan the list
            // looking to see if the serviceUuid is anywhere in the list.
            BleUuid foundServiceUuid;
            size_t svcCount = scanResults[ii].advertisingData.serviceUUID(&foundServiceUuid, 1);
            //Serial.print("svcCount: ");
            //Serial.println(svcCount);

            bool matchAddr = true;
            bool allFF = true;
            for(int i = 0; i <= 5; i++){
              if (connBLE[i] != scanResults[ii].address[i]) {
                matchAddr = false;
              }
              if (connBLE[i] != 0xFF) {
                allFF = false;
              }
            }

            if (matchAddr) {
              Serial.println("Addr Match");
            } else {
              Serial.println("No Addr Match");
            }

            if ( (matchAddr || allFF) && svcCount > 0 && foundServiceUuid == serviceUuid)
            {

              peer = BLE.connect(scanResults[ii].address);
              //Serial.print("peer: ");
              //Serial.println(peer);
              if (peer.connected())
              {
                if (allFF) {
                  for(int i = 0; i <= 5; i++){
                    connBLE[i] = scanResults[ii].address[i];
                  }
                  EEPROM.put(10, connBLE);
                }
                Serial.printlnf("successfully connected %02X:%02X:%02X:%02X:%02X:%02X!",
                                scanResults[ii].address[0], scanResults[ii].address[1], scanResults[ii].address[2],
                                scanResults[ii].address[3], scanResults[ii].address[4], scanResults[ii].address[5]);
                peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);
                //Serial.print("txUuid");
                //Serial.println(txUuid);
                peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);
                //Serial.print("rxUuid");
                //Serial.println(rxUuid);

                // Could do this instead, but since the names are not as standardized, UUIDs are better
                
                //bool txResult = peer.getCharacteristicByDescription(peerTxCharacteristic, "tx");
                //if (txResult) {
                //  Serial.println("tx result true");
                //}
                //bool rxResult = peer.getCharacteristicByDescription(peerRxCharacteristic, "rx");
                //if (rxResult) {
                //  Serial.println("rx result true");
                //}

                //peer.getCharacteristicByDescription(peerMxCharacteristic, "mx");
                //mxcmd[0] = 1; // send BGX220 into stream mode
                //mxch = 1;

                //bool mxResult = peerMxCharacteristic.setValue(mxcmd, mxch);
                //if (mxResult) {
                //  Serial.println("mx result true");
                //}
              } else {
                Serial.println("Connect failed!!!!");
              }
              break;
            }
          }
        }
      }
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

  //Serial.begin(115200);
  Serial.begin(38400);
  
  waitFor(Serial.isConnected, 10000);

  Serial.println("starting setup");
  String addrTxt = "";
  EEPROM.get(10, connBLE);
  Serial.println("EEPROM");
  for (int i = 0; i <= 5; i++){
    addrTxt.concat(String::format("%02X:", connBLE[i]));
    Serial.printlnf("%02X", connBLE[i]);
  } 
  unsigned int ll = addrTxt.length();
  addrTxt.remove(ll-1);
  Serial.println(addrTxt);
  tdisp = micros();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();                    // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  fdisp = (float)(micros() - tdisp);
  ddisp = (double)fdisp;
  //Particle.variable("DispTime", &ddisp, DOUBLE);
  // since there is no easy way to get display status from the adafruit api, time how long the init takes
  // typically it's a little over 20,000 us .. so if over 2x that we must not have a display
  if (fdisp < 40000.)
  {
    haveDisplay = true;
  }

  lineLCD0();
  lineLCD(1, "Pump Ready");
  lineLCD(2, "Bluetooth Central");
  lineLCD(3, "Paired with:");
  lineLCD(4, addrTxt.c_str());
  lineLCD(5, "V 0.9 03/2021 DFM");
  Serial.println("about to showLCD");
  showLCD();

  lastShowDisplay = millis();
  
  powerTimer.start();
  Serial.println("about to turn BLE on");
  int bleOnRet;
  bleOnRet = BLE.on();
  BLE.selectAntenna(BleAntennaType::EXTERNAL);

  Serial.print("ble On Ret: ");
  Serial.println(bleOnRet);
  peerTxCharacteristic.onDataReceived(onDataReceived, &peerTxCharacteristic);

  //BLE.addCharacteristic(txCharacteristic);
  //BLE.addCharacteristic(rxCharacteristic);

  //BleAdvertisingData data;
  //data.appendServiceUUID(serviceUuid);
  //BLE.advertise(&data);

  pinMode(pwmPumpPin, OUTPUT);
  analogWriteResolution(pwmPumpPin, 10);
  analogWrite(pwmPumpPin, 0, 10000);

  pinMode(flowDirPin, OUTPUT);

  // Preset pump speed to 0, set FWD direction

  setRunSpeed(0);
  setPumpSpeed(0);
  //setPumpFwd();

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
    //Serial.println(ar);
    //Serial.println((float)ar * 3.3 / 4096.);
    arsum = arsum + ar;
  }
  //Serial.printlnf("arsum: %d", arsum);
  //Serial.printlnf("(float)arsum / 50.0: %f", (float)arsum / 50.0);
  pressZero = adcVolts((float)arsum / 50.0);
  //Serial.printlnf("pressZero: %f", pressZero);

  // Set up interrupts to catch pulses from the flowmeters

  pinMode(flowMeterPinFill, INPUT);
  attachInterrupt(flowMeterPinFill, gpioCBFill, FALLING);

  pinMode(flowMeterPinEmpty, INPUT);
  attachInterrupt(flowMeterPinEmpty, gpioCBEmpty, FALLING);

  pinMode(flowMeterPinStop, INPUT);
  attachInterrupt(flowMeterPinStop, gpioCBStop, FALLING);

  // Setup power down pin, taking it high turns off the pump

  pinMode(powerDownPin, OUTPUT);
  digitalWrite(powerDownPin, LOW);

  sendSPI("Init", 0.0);
  sendSPI("rPWM", 0.0);

  // mainLoop.start();

  Serial.println("init done");
  medidoEnabled = true;
}

void lineLCD0()
{
  //display.clearDisplay();
  for (int i = 0; i <= 4; i++)
  {
    strncpy(textLCD[i], "", DISP_BUF_LEN);
  }
}

void lineLCD(int line, String text)
{
  strcpy(textLCD[line - 1], text.c_str());
}

void lineLCDf(int line, String text, float val, String fmt, String sfx)
{
  //Serial.println(" ");
  //Serial.print("line:");
  //Serial.println(line);
  //Serial.print("text:");
  //Serial.println(text);
  //Serial.print("val:");
  //Serial.println(val);
  //Serial.print("fmt:");
  //Serial.println(fmt);
  //Serial.print("sfx:");
  //Serial.println(sfx);

  sprintf(textLCD[line - 1], text + fmt + sfx, val);
}

void lineLCDd(int line, String text, int val, String fmt, String sfx)
{

  //Serial.println(" ");
  //Serial.print("line:");
  //Serial.println(line);
  //Serial.print("text:");
  //Serial.println(text);
  //Serial.print("val:");
  //Serial.println(val);
  //Serial.print("fmt:");
  //Serial.println(fmt);
  //Serial.print("sfx:");
  //Serial.println(sfx);

  sprintf(textLCD[line - 1], text + fmt + sfx, val);
}

void showLCD()
{
  //Serial.println("showLCD");
  //Serial.println(millis());
  //Serial.println(lastShowDisplay);
  //Serial.println(haveDisplay);
  if (!haveDisplay)
  {
    return;
  }
  if (millis() - lastShowDisplay < 200)
  {
    return;
  }
  lastShowDisplay = millis();
  
  display.clearDisplay();
  for (int i = 0; i <= 4; i++)
  {
    //Serial.print("i=");
    //Serial.println(i);
    //Serial.print("line: ");
    //Serial.println(textLCD[i]);
    display.setCursor(0, i * 13);
    display.println(textLCD[i]);
  }
  display.display();
}
char tstr[32];
char *timeFmt(int tt)
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
  return tstr;
}

void gpioCBFill()
{
  if (micros() - lastFillMicro < 1000)
  {
    lastFillShort = lastFillShort + 1;
  }
  lastFillMicro = micros();
  if (pumpFwd)
  {
    pulseCountFill += 1;
  }
  else
  {
    pulseCountBad += 1;
  }
}

void gpioCBEmpty()
{
  if (micros() - lastEmptyMicro < 1000)
  {
    lastEmptyShort = lastEmptyShort + 1;
  }
  lastEmptyMicro = micros();
  if (!pumpFwd)
  {
    pulseCountEmpty += 1;
  }
  else
  {
    pulseCountBad += 1;
  }
}

void gpioCBStop()
{
  // code to stop goes here
  pulseCountStop += 1;
  //Serial.print("number of stop pulses: ");
  //Serial.println(pulseCountStop);
}

void setRunSpeed(int pw)
{ // careful .. this is in pwm units 0 .. 1023 ..  not %
  pulseCountBad = 0;
  lastPulseCountBad = 0;

  sendSPI("rPWM", (float)pw);

  analogWrite(pwmPumpPin, pw);
  Serial.print("just sent pw: ");
  Serial.println(pw);

  runPWM = pw;
}

int oldspd = 0;

void setPumpSpeed(float ps)
{                                      // this is ps units // 0 to 100%
  pumpPWM = (int)(ps * opPWM / 100.0); //math.floor(ps*opPWM/100)
  if (pumpPWM < minPWM)
  {
    pumpPWM = 0;
  }
  if (pumpPWM > maxPWM)
  {
    pumpPWM = maxPWM;
  }
  sendSPI("pPWM", pumpPWM);
  if (pumpPWM != oldspd)
  {
    //print("pump speed set to", pumpPWM)
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
  //setRunSpeed(pumpPWM)
  //if (pumpPWM < minPWM)
  //{
  //  setRunSpeed(pumpPWM);
  //}
  //else
  //{
  setRunSpeed(minPWM);
  //}
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
  //Serial.print("string: ");
  //Serial.println(str);
  //Serial.print("val: ");
  //Serial.println(val);
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
      peerRxCharacteristic.setValue(txbuf, nch);
      //txCharacteristic.setValue(txbuf, nch);
    }
  }
}

float adcVolts(float vRaw)
{
  float v;
  v = adcDiv * (vRaw - (float)adcZero) / adcScale;
  //Serial.print("vRaw: ");
  //Serial.print(vRaw);
  //Serial.print(" comp volts: ");
  //Serial.println(v);

  return v;
}

void timerCB()
{

  float pSpd;
  float errsig;
  float deltaFFill;
  float deltaFEmpty;
  float deltaF;
  unsigned long now;
  float deltaT;
  unsigned long dtms;
  float vbatt;

  // send the app a message if we get #pulses > pulseStop  at piss tank sensor

  if (millis() - lastTimerCB < minTimerCB)
  {
    return;
  }

  lastTimerCB = millis();
  if (pulseCountStop > pulseStop)
  {
    sendSPI("pSTP", (float)pulseCountStop);
    pulseCountStop = 0;
  }

  adcAvg = adcAvg - (adcAvg - (float)analogRead(A0)) / 2.0; // running avg of raw adc readings
  pressPSI = (adcVolts(adcAvg) - pressZero) * (pressScale);
  sendSPI("pPSI", pressPSI);

  //if pressPSI < 0 then pressPSI = 0 end

  flowCount = ((float)pulseCountFill / pulsePerOzFill) - ((float)pulseCountEmpty / pulsePerOzEmpty);

  if (runPWM != 0)
  {
    //Serial.printlnf("fCNT, eCNT, flwc, lFS, lES: %d %d %.2f %d %d", pulseCountFill, pulseCountEmpty, flowCount, lastFillShort, lastEmptyShort);
  }

  now = millis();

  if (pumpStartTime == 0)
  {
    pumpStartTime = now;
  }

  dtms = now - lastFlowTime;
  //  if needed port this line: if dtms < 0 then dtus = dtus + 2147483647 end // in case of rollover
  if (dtms > 1000)
  {
    sendSPI("fCNT", (float)pulseCountFill);
    sendSPI("eCNT", (float)pulseCountEmpty);
    if (pulseCountBad != lastPulseCountBad)
    {
      sendSPI("cBAD", (float)pulseCountBad);
      //Serial.printlnf("cBAD: last %d %d", pulseCountBad, lastPulseCountBad);
      lastPulseCountBad = pulseCountBad;
    }
    deltaT = (float)dtms / (1000. * 60.); // mins
    deltaFFill = (float)(pulseCountFill - lastPulseCountFill) / pulsePerOzFill;
    deltaFEmpty = (float)(pulseCountEmpty - lastPulseCountEmpty) / pulsePerOzEmpty;
    deltaF = deltaFFill - deltaFEmpty;
    flowRate = flowRate - (flowRate - (deltaF / deltaT)) / 1.2;
    sendSPI("fRAT", flowRate); //does the app still use fRAT now that we send fCNT and eCNT?
    //sendSPI("fCLK", (float)(millis() - bootTime));
    //sendSPI("fDEL", deltaF);
    //sendSPI("fDET", deltaT);
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
    if (runningTime > 100000.0 || runningTime < 0.0)
    {
      //Serial.printlnf("runPWM <= 0: runPWM=%d, pumpStopTime=%lu, pumpStartTime=%lu, runningTime=%f", runPWM, pumpStopTime, pumpStartTime, runningTime);
    }
  }

  // dont send time if pump not on .. confuses graph on iPhone

  if (runPWM != 0)
  {
    sendSPI("rTIM", runningTime);
    //Serial.print("Running Time: ");
    //Serial.println(runningTime);
  }

  if (!pumpFwd && (runPWM > 0))
  {
    revSpd = revSpd + (revSpdMax - revSpd) / 6.0; //soft start... don't slam on
    if (runPWM != 0)
    { 
      if (enablePump) {
        setPumpSpeed(revSpd);
        setRunSpeed(pumpPWM);
      } else {
        setPumpSpeed(0.0);
        setRunSpeed(0);
      }
    }
  }
  if ((pumpFwd && (runPWM > 0)) && (PIDiGain != 0 or PIDpGain != 0))
  {
    errsig = pressLimit - pressPSI;
    PIDpTerm = errsig * PIDpGain;
    PIDiTerm = PIDiTerm + errsig * PIDiGain;
    if (PIDiTerm < 0.0)
    {
      PIDiTerm = 0.0;
    }
    if (PIDiTerm > 100.0)
    {
      PIDiTerm = 100.0;
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
    //if (( runPWM != 0) && (abs(millis() - pumpStopTime) > 1000) ){ // guard against asynch off in last sec
    if (not enablePump)
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
 
  seq = seq + 1;
  //if ((seq % 5 == 0) && (millis() - bootTime > 5000))
  if (millis() - bootTime > 3000)
  {
    lineLCDf(2, "Flow Rate ", flowRate, "%2.1f", " oz/s");
    lineLCDf(3, "Volume ", flowCount, "%4.1f", " oz");
    if (runPWM == 0)
    {
      lineLCD(4, timeFmt(0));
    }
    else
    {
      lineLCD(4, timeFmt(runningTime));
    }
    //Serial.print("seq:");
    //Serial.println(seq);
    //Serial.print(millis() / 1000.0);
    showLCD();

    if (seq % 10 == 0)
    {
      vbatt = (float)analogRead(A1) * 0.5644 * 3.3 / 4096.;
      sendSPI("Batt", vbatt); // kludge: 8266 version has 7.504:1 divider, Argon version is 4.235 .. correct for that
                              //should just send volts and change 8266 code and ios app..

      lineLCDf(5, "Pump Battery ", vbatt * 7.504, "%2.2f", "V"); // 7.504 is the resistive divider .. ios app applies it in swift .. should make uniform...
      //sendSPI("Batt", 12.3/7.5);
    }
    if (seq % 10 == 6)
    {
      sendSPI("Curr", 100 * ((3.3 / 4096.0) * ((float)analogRead(A5) - currentZero)));
    }
  }
}

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context)
{
  //Log.trace("Received data from: %02X:%02X:%02X:%02X:%02X:%02X:", peer.address()[0], peer.address()[1], peer.address()[2], peer.address()[3], peer.address()[4], peer.address()[5]);
  //Serial.print(len);

  //Serial.println("in onDataReceived");

  for (size_t ii = 0; ii < len; ii++)
  {
    //Serial.print(data[ii]);
    //Serial.print(char(data[ii]));
    textacc.concat(char(data[ii]));
    //Serial.write(data[ii]);
    //Serial.print("_");
    //Serial.print(ii);
    //Serial.print(">>" + textacc);
    //Serial.print("\r\n");
    if (data[ii] == '\n')
    {
      //onLinefeed(textacc.c_str());
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

  //Serial.println("in onLinefeed");
  //Serial.println(msg);
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
  //Serial.print(lparen);
  //Serial.print(rparen);
  //Serial.print(colon);
  //Serial.println("%");

  //if ( (lparen < 0 || rparen < 0) && (!isKwd) ) {
  //  Serial.println("lparen and rparen < 0 returning");
  //  return;
  //}

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

  //Serial.print(key);
  //Serial.print("+++");
  //Serial.print(val);
  //Serial.println("###");

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

  Serial.printlnf("execKwd: k,v = %s,%s", k.c_str(), v.c_str());
  if (k == "ssid")
  {
    wifissid = v.c_str();
  }
  else if (k == "pwd")
  {
    wifipwd = v.c_str();
    Serial.printlnf("set wifipwd to %s", wifipwd);
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
    Serial.println("Stop cmd received");
  }
  else if (k == "unpair") {
    Serial.println("unpair cmd");
    for(int i=0; i <= 5; i++) {
      connBLE[i] = 0xFF;
    }
    EEPROM.put(10, connBLE);
  }
  else if (k == "update")
  {
    Serial.println("Update command received");
    //medidoEnabled = false;
    sendSPI("OTA", 1); // Starting WiFi

    Serial.print("final ssid: ");
    Serial.println(wifissid);
    Serial.print("final pwd: ");
    Serial.println(wifipwd);
    //Serial.print("host: ");
    //Serial.println(wifihost);
    //Serial.print("dir: ");
    //Serial.println(wifidir);
    //Serial.print("image: ");
    //Serial.println(wifiimage);

    //WiFi.clearCredentials();
    if (wifissid != "" && wifipwd != "") { 
      Serial.println("got some nonblank credentials");
      WiFi.setCredentials(wifissid, wifipwd);
    } else {
      Serial.println("got blank credentials");
    }

    WiFi.on();
    WiFi.connect();

    int wLoops = 0;
    while (!WiFi.ready() && wLoops < 600)
    { //timeout 30 seconds if no wifi or bad creds
      delay(100);
      wLoops = wLoops + 1;
    }
    if (wLoops >= 600)
    {
      Serial.println("No Wifi Connection");
      sendSPI("OTA", -1); //No WiFi connection
      return;
    }
    Serial.println("Wifi connected");
    sendSPI("OTA", 2); // WiFi connected
    //System.enterSafeMode();
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
      Serial.println("No particle cloud connection");
      sendSPI("OTA", -30); // No Particle Cloud Connection
      return;
    }
    Serial.println("Particle cloud connected");
    sendSPI("OTA", 30); // particle cloud connected
  }
}

void execCmd(String k, String v)
{

  // each time we get any command, restart the poweroff timer

  powerTimer.start(); // this will reset it if already running

  //Serial.printlnf("execCmd: k,v = %s,%s", k.c_str(), v.c_str());

  if (k == "Spd")
  { // what change was it?
    saveSetSpeed = atof(v.c_str());
    Serial.print("+Spd: ");
    Serial.println(saveSetSpeed);
    setPumpSpeed(saveSetSpeed);
    if (saveSetSpeed == 0)
    {
      lineLCD(1, "Pump Off");
    }
    //lineLCDd(2, "Set Spd", saveSetSpeed, "%d", "%");
    showLCD();
  }
  else if (k == "Fill")
  {
    enablePump = true;
    setPumpFwd();
    lineLCD0();
    lineLCD(1, "Fill");
    Serial.println("Fill");
    //lineLCDd(2, "Set Spd", saveSetSpeed, "%d", "%");
    showLCD();
    lastFillShort = 0;
    lastEmptyShort = 0;
  }
  else if (k == "Off")
  {
    Serial.println("pump stop k=off");
    enablePump = false;
    setPumpSpeed(0.0);
    setRunSpeed(0);
    pumpStopTime = millis();
    Serial.println("Off");
    Serial.printlnf("OFF cmd: runPWM, pumpPWM, pumpFwd: %d, %d", runPWM, pumpPWM);

    lineLCD(1, "Pump Off");
  }
  else if (k == "Empty")
  {
    revSpdMax = saveSetSpeed;
    enablePump = true;
    setPumpRev();
    lineLCD0();
    lineLCD(1, "Empty");
    Serial.println("Empty");
  }
  else if (k == "Clear")
  {
    pulseCountFill = 0;
    pulseCountEmpty = 0;
    pulseCountBad = 0;

    lastPulseCountFill = 0;
    lastPulseCountEmpty = 0;
    lastPulseCountBad = 0;

    flowCount = 0;
    runningTime = 0;
    pumpStartTime = 0;
    pumpStopTime = 0;
    Serial.println("Clear");
    sendSPI("rTIM", runningTime);
    lineLCD(1, "Clear");
  }
  else if (k == "CalF")
  {
    Serial.printlnf("CalFactFill passed in: %s", v.c_str());
    pulsePerOzFill = atof(v.c_str());
  }
  else if (k == "CalE")
  {
    Serial.printlnf("CalFactEmpty passed in: %s", v.c_str());
    pulsePerOzEmpty = atof(v.c_str());
  }
  else if (k == "Prs")
  {
    pressLimit = atof(v.c_str()) / 10.0;
    if (pressLimit > MAXpress)
    {
      pressLimit = MAXpress;
    }
    if (pressLimit < MINpress)
    {
      pressLimit = MINpress;
    }
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
  }
  else
  {
    Serial.printlnf("execCmd cmd error: k,v = %s,%s", k.c_str(), v.c_str());
  }
}
