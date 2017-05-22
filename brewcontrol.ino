#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#include "badge.h"
#include "lcd5110.h"

const int mpin = 3;
const int oneWirePin = 2;

Badge badge;
LCD5110 display;

OneWire oneWire(oneWirePin);
DallasTemperature temp(&oneWire);

uint8_t probeAddr[8];

struct {
  double setPoint=140;
  double kp=600, ki=25, kd=0;
} params;
double currentTemp, dutyCycle;

PID *pid;

const int cycleSize=2000; // each cycle is 2s long
unsigned long cycleStartTime;
bool powerOn=false;

enum {
  TEMP,
  PCT
} target;

enum PROGRAM {
  P_NONE,
  P_KP,
  P_KI,
  P_KD,
} program;

void setup() {
  pinMode(mpin, OUTPUT);
  Serial.begin(9600);
  
  temp.begin();
  temp.getAddress(probeAddr, 0);

  EEPROM.get(0, params);
  target=TEMP;
  pid=new PID(&currentTemp, &dutyCycle, &params.setPoint, params.kp, params.ki, params.kd, DIRECT);

  pid->SetOutputLimits(0, cycleSize);
  pid->SetSampleTime(500);
  cycleStartTime = millis();  
}

const int btnDelay = 50;

void loop() {
  unsigned long tstart=millis();
  temp.requestTemperaturesByAddress(probeAddr);
  currentTemp = temp.getTempF(probeAddr);
  Serial.print("temp took ");
  Serial.print(millis() - tstart);
  Serial.println("");
  pid->SetMode((powerOn && target == TEMP) ? AUTOMATIC : MANUAL);
  pid->Compute();

  unsigned long now = millis();
  if (now - cycleStartTime > cycleSize)
    cycleStartTime += cycleSize;
  bool on = dutyCycle > now - cycleStartTime;
  if (!powerOn) on=false;
  digitalWrite(mpin, on ? HIGH : LOW);
  
  drawDisplay();

  if (millis() > 500) {
    // avoid phantom input during startup by delaying 500ms
    if (badge.Button0(btnDelay)) powerOn = !powerOn;
    if (badge.Button1(50)) target = (target == TEMP) ? PCT : TEMP;
    if (badge.JoyCenter(btnDelay)) program = (PROGRAM)(program+1);
    if (program > P_KD) program=P_NONE;
    if (target == TEMP) {
      double* tgt;
      double delta=0.1;
      switch(program) {
        case P_KP: tgt=&params.kp; break;
        case P_KD: tgt=&params.kd; break;
        case P_KI: tgt=&params.ki; break;
        default: tgt=&params.setPoint; delta=1;
      }

      bool change=false;
      if (badge.JoyUp(btnDelay, true)) { change=true; *tgt += delta; }
      if (badge.JoyDown(btnDelay, true)) { change=true; *tgt -= delta; }
      if (badge.JoyRight(btnDelay, true)) { change=true; *tgt += delta*10; }
      if (badge.JoyLeft(btnDelay, true)) { change=true; *tgt -= delta*10; }
      if (*tgt < 0) *tgt = 0;
      if (params.setPoint > 230) params.setPoint = 230;
      if (change) {
        pid->SetTunings(params.kp, params.ki, params.kd);
        EEPROM.put(0, params);
      }
    } else {
      if (badge.JoyUp(btnDelay, true)) dutyCycle += cycleSize / 100.0;
      if (badge.JoyDown(btnDelay, true)) dutyCycle -= cycleSize / 100.0;
      if (dutyCycle < 0) dutyCycle = 0;
      if (dutyCycle >= cycleSize) dutyCycle = cycleSize;
    }
  }
}

void drawDisplay() {
  display.Clear();
  
  display.WriteString((String(currentTemp, 2) + String("F")).c_str(), 1, 0);
  int cyclePct = (int)(dutyCycle / cycleSize * 100);
  if (!powerOn) cyclePct = 0;
  display.WriteString((String(cyclePct) + String("%")).c_str(), 56, 0);
  display.WriteString("Set:", 1, 15);
  if (target == TEMP) {
    display.WriteString((String(params.setPoint, 2) + String("F")).c_str(), 35, 15);
  } else {
    display.WriteString((String((int)(dutyCycle / cycleSize * 100)) + String("%")).c_str(), 35, 15);
  }
  display.WriteString(powerOn ? "on" : "off", 1, 40);

  if (target == TEMP) {
    if (program == P_KP) {
      display.WriteString((String("kp: ")+String(params.kp, 2)).c_str(), 1, 25);
    } else if (program == P_KD) {
      display.WriteString((String("kd: ")+String(params.kd, 2)).c_str(), 1, 25);
    } else if (program == P_KI) {
      display.WriteString((String("ki: ")+String(params.ki, 2)).c_str(), 1, 25);
    }
  }

  display.Redraw();
}

