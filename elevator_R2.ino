// M1 = solenoid    || Left
// M2 = Pully Motor || Right

//#include "DualVNH5019MotorShield.h"
#include <HX711_ADC.h>
#include <EEPROM.h>

#define MAX_SPEED  20000
#define SPEED  20000
#define MAX_POSITION  -6000
#define DOWN  1
#define UP  2
#define BREAK  3
#define STOP  0
#define DEBUG 4

//DualVNH5019MotorShield md;

char inputChar = ' ';
unsigned long t = 0;
volatile long motor_position = 0,last_position=MAX_POSITION/10;
int sysState=0;
unsigned long currentTime=0,op_timestamp=0,op_timeout=15000;
float load_cell_data=0,startWieght=0,basketWieght=1000;

//pins:
const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin
int EndStopSwitch = 12;
int Solenoid = 11;
int Motor_INA = 2;
int Motor_INB = 4;
int Motor_PWM = 9;
int Motor_CS = A0;
int Motor_ENCA = 3;
int Motor_ENCB = 10;

void closeSolenoid();
void openSolenoid();
void set_motor(int dir, int PWM);

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

void setup()
{
  Serial.begin(115200);
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 89.28; //696.0; // uncomment this if you want to set the calibration value in the sketch
  unsigned long stabilizingtime = 100; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    LoadCell.setSamplesInUse(5);
    Serial.println("Startup is complete");
    Serial.print("Number of samples: ");
    Serial.println(LoadCell.getSamplesInUse());
  }
    
  pinMode(EndStopSwitch,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Solenoid, OUTPUT);
  pinMode(Motor_INA,OUTPUT);
  pinMode(Motor_INB,OUTPUT);
  pinMode(Motor_PWM,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Motor_ENCA), updateEncoder, RISING);
  
}


void loop()
{
  long startTime=0;
  int error=0;
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; //increase value to slow down serial print activity

  if (LoadCell.update()) load_cell_data = LoadCell.getData();
  currentTime=millis();
  
  switch(sysState) {
    case DOWN:
          if(currentTime-op_timestamp<20) set_motor(UP, 100);
          else if(currentTime-op_timestamp<30) set_motor(BREAK, 255);
          else set_motor(DOWN, 120); 
          if(motor_position<last_position+1000) set_motor(DOWN, 10);  
          if(motor_position<MAX_POSITION || load_cell_data<startWieght-5) 
          {
            sysState=STOP;
            set_motor(UP, 255);
            startTime=millis();
            while(millis()<startTime+10) ;
            set_motor(BREAK,255);
            if(motor_position<last_position/2) last_position=motor_position;
            closeSolenoid();
          }
          break;
    case UP:
          if(motor_position>-700) set_motor(UP, 50);
          else set_motor(UP, 150);
          
          if(digitalRead(EndStopSwitch)==0) 
          {
            sysState=STOP;
            set_motor(DOWN, 255);
            startTime=millis();
            while(millis()<startTime+10) ;
            set_motor(BREAK,255);
            closeSolenoid();
            LoadCell.tare();
          }
          if(load_cell_data>1000 && sysState==UP)
          {
            sysState=STOP;
            set_motor(DOWN, 255);
            startTime=millis();
            while(millis()<startTime+300) ;
            set_motor(BREAK,255);
            Serial.println("overwieght emergency stop");
          }
          break;
    case STOP:
          set_motor(BREAK,255);
          closeSolenoid();
          break;
  }
  if(millis()>op_timestamp+op_timeout && sysState!=STOP) {
    sysState=STOP;
    Serial.println("operation timeout");
  }
  
  if(digitalRead(EndStopSwitch)==0)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    motor_position = 0;
  }
  else digitalWrite(LED_BUILTIN,LOW);

  if (millis() > t + serialPrintInterval) {
      Serial.print("state: ");
      Serial.print(sysState);
      Serial.print("\tLoad_cell: ");
      Serial.print(load_cell_data);
      Serial.print("\tposition: ");
      Serial.print(motor_position);
      Serial.print("\t last pos: ");
      Serial.println(last_position);
      t = millis();
  }
}



void serialEvent()
{
  long startTime=0;
  if(Serial.available())
  {
    inputChar = Serial.read();
    
    if (inputChar == 'w')
    {
      Serial.println("go up");
      startWieght=load_cell_data;
      sysState=UP;
      op_timestamp=millis();
      openSolenoid();
    }
    else if (inputChar == 's')
    {
      Serial.println("go down");
      startWieght=load_cell_data;
      sysState=DOWN;
      op_timestamp=millis();
      openSolenoid();
    }
    else if (inputChar == 'e')
    {
      op_timestamp=millis();
      sysState=DEBUG;
      openSolenoid();
    }
    else if (inputChar == 'd')
    {
      op_timestamp=millis();
      sysState=DEBUG;
      closeSolenoid();
    }
    else if (inputChar == 't')
    {
      LoadCell.tareNoDelay();
      Serial.println("tare");
    }
    else
    {
      sysState=STOP;
      Serial.println("STOP");
      op_timestamp=millis();
      set_motor(BREAK, 255);
      closeSolenoid();
    }
  }
}

void updateEncoder() {
  // Increment value for each pulse from encoder
  int b = digitalRead(Motor_ENCB);
  if(b > 0){
    motor_position++;
  }
  else{
    motor_position--;
  }
}

void openSolenoid()
{
  digitalWrite(Solenoid,HIGH);
  //Serial.println("solenoid open");
}

void closeSolenoid()
{
  digitalWrite(Solenoid,LOW);
  //Serial.println("solenoid closed");
}

void set_motor(int dir, int PWM)
{
  switch(dir) {
    case DOWN:
      digitalWrite(Motor_INA,HIGH);
      digitalWrite(Motor_INB,LOW);
      analogWrite(Motor_PWM,PWM);
      break;
    case UP:
      digitalWrite(Motor_INA,LOW);
      digitalWrite(Motor_INB,HIGH);
      analogWrite(Motor_PWM,PWM);
      break;
    case BREAK:
      digitalWrite(Motor_INA,LOW);
      digitalWrite(Motor_INB,LOW);
      analogWrite(Motor_PWM,PWM);
      break;
    default:
      digitalWrite(Motor_INA,LOW);
      digitalWrite(Motor_INB,LOW);
      analogWrite(Motor_PWM,0);
      break;    
  }
}
