#include <Arduino.h>
#include "SoftwareSerial.h"

SoftwareSerial MD_Slave_Bus(52, 53);

//ref 03 https://docs.rs-online.com/bd24/0900766b814225ff.pdf
#define DIPPin1 0
#define DIPPin2 0
#define DIPPin4 0
#define DIPPin8 0

//#PWM pin
#define pin1 6
#define pin2 7

uint8_t recNum, pwm[2];

int receivePacket();
void applyPWM();
void initDIPSwitch();
void readDIPSwitch();
void changePWMFreqency();
void testCase_init();

void setup()
{
  MD_Slave_Bus.begin(9600);
  // put your setup code here, to run once:
  //initDIPSwitch();
  //readDIPSwitch();
  Serial.begin(256000);
  recNum = 2;
  changePWMFreqency();
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

  Serial.print(dataBuffer[0]);
  Serial.print('\t');
  Serial.print(dataBuffer[1]);
  Serial.print('\t');
  Serial.print(dataBuffer[2]);
  Serial.print('\t');
  Serial.print(dataBuffer[3]);
  Serial.print('\t');
  Serial.print(dataBuffer[4]);
  Serial.println("");

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
  analogWrite(pin1, pwm[0]);
  analogWrite(pin2, pwm[1]);
  Serial.println("went through");
  // Serial.print(pwm[0]);
  // Serial.print('\t');
  // Serial.print(pwm[1]);
  // Serial.print("\r\n");
}

void initDIPSwitch()
{
  pinMode(DIPPin1, INPUT);
  pinMode(DIPPin2, INPUT);
  pinMode(DIPPin4, INPUT);
  pinMode(DIPPin8, INPUT);
}

void readDIPSwitch()
{
  recNum = int(digitalRead(DIPPin8)) << 3 ||
           int(digitalRead(DIPPin4)) << 2 ||
           int(digitalRead(DIPPin2)) << 1 ||
           int(digitalRead(DIPPin1));
}

void changePWMFreqency()
{
}
