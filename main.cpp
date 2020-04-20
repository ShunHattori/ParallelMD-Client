#include <Arduino.h>
#include "SoftwareSerial.h"
#include "PWMfrequency.h"
#include "DebounceSwitch.h"

//ref 03 https://docs.rs-online.com/bd24/0900766b814225ff.pdf
#define DIPPin1 A5
#define DIPPin2 A4
#define DIPPin4 A3
#define DIPPin8 A2

//#PWM pin
#define PWM1 11
#define PWM2 10
#define SHUTDOWN 12

#define SS_RX 8
#define SS_TX 9

SoftwareSerial MD_Slave_Bus(SS_RX, SS_TX);
DebounceSwitch DIP1(DIPPin1, DebounceSwitch::NONE);
DebounceSwitch DIP2(DIPPin2, DebounceSwitch::NONE);
DebounceSwitch DIP4(DIPPin4, DebounceSwitch::NONE);
DebounceSwitch DIP8(DIPPin8, DebounceSwitch::NONE);

uint8_t recNum, pwm[2];

int receivePacket();
void applyPWM();
void initDIPSwitch();
void readDIPSwitch();
void changePWMFreqency();
void testCase_init();

void setup()
{
  // put your setup code here, to run once:
  changePWMFreqency();
  initDIPSwitch();
  readDIPSwitch();
  MD_Slave_Bus.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!receivePacket())
    applyPWM();
  else
  {
    //エラー検証用デバッグメッセージとかここに書く
  }
}

int receivePacket()
{
  //#datas arrival checking
  MD_Slave_Bus.listen();
  if (!(MD_Slave_Bus.available()))
    return 1; //data not ready

  //#storing datas
  uint8_t dataBuffer[5];
  dataBuffer[0] = MD_Slave_Bus.read(); //receive front byte((must have been storing recNum))

  //#recNum checking
  //@desc: check recNum before reading main data frame
  if (dataBuffer[0] != recNum)
    return 2; //host sent datas to other MD slave

  while (!(MD_Slave_Bus.available() > 3)) //waiting ready for main frame arrival
    continue;

  for (uint8_t dataIndex = 1; dataIndex < 5; dataIndex++) //read main data frame
  {
    dataBuffer[dataIndex] = MD_Slave_Bus.read();
  }

  // Serial.print(dataBuffer[0]);
  // Serial.print('\t');
  // Serial.print(dataBuffer[1]);
  // Serial.print('\t');
  // Serial.print(dataBuffer[2]);
  // Serial.print('\t');
  // Serial.print(dataBuffer[3]);
  // Serial.print('\t');
  // Serial.print(dataBuffer[4]);
  // Serial.println("");

  //#CheckSum checking
  uint8_t bufferCheckSum1 = dataBuffer[0] ^ dataBuffer[1] ^ dataBuffer[2];
  if (bufferCheckSum1 != dataBuffer[3])
    return 3; //CheckSum(1) Error
  uint8_t bufferCheckSum2 = dataBuffer[0] + dataBuffer[1] + dataBuffer[2];
  if (bufferCheckSum2 != dataBuffer[4])
    return 4; //CheckSum(2) Error

  //#storing pwm datas
  pwm[0] = dataBuffer[1];
  pwm[1] = dataBuffer[2];
  return 0; //successful
}

void applyPWM()
{
  analogWrite(PWM1, pwm[0]);
  analogWrite(PWM2, pwm[1]);
  //Serial.println("went through");
  // Serial.print(pwm[0]);
  // Serial.print('\t');
  // Serial.print(pwm[1]);
  // Serial.print("\r\n");
}

void initDIPSwitch()
{
  for (size_t i = 0; i < 5; i++)
  {
    delayMicroseconds(5);
    DIP1.update();
    DIP2.update();
    DIP4.update();
    DIP8.update();
  }
}

void readDIPSwitch()
{
  recNum = DIP8.stats() << 3 ||
           DIP4.stats() << 2 ||
           DIP2.stats() << 1 ||
           DIP1.stats();
}

void changePWMFreqency()
{
  setPwmFrequencyMEGA2560(PWM1, 1);
  setPwmFrequencyMEGA2560(PWM2, 1);
}
