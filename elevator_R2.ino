// M1 = solenoid    || Left
// M2 = Pully Motor || Right

//#include "DualVNH5019MotorShield.h"
#include <HX711_ADC.h>
#include <EEPROM.h>

#define MAX_SPEED  20000
#define SPEED  20000
#define DOWN  1
#define UP  2
#define BREAK  3
#define STOP  0

//DualVNH5019MotorShield md;

char inputChar = ' ';
bool stringComplete = false;
unsigned long t = 0;
long motor_position = 0;

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
  calibrationValue = 600; //696.0; // uncomment this if you want to set the calibration value in the sketch
  unsigned long stabilizingtime = 10; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
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
  float load_cell_data=0;
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; //increase value to slow down serial print activity
  //Serial.println(digitalRead(EndStopSwitch));motor_position
  Serial.println(motor_position);
  set_motor(STOP, 0);
  if (LoadCell.update()) {
    if (millis() > t + serialPrintInterval) {
      load_cell_data = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(load_cell_data);
      t = millis();
    }
  }
  if(digitalRead(EndStopSwitch)==0)
  {
    digitalWrite(LED_BUILTIN,HIGH);
  }
  else digitalWrite(LED_BUILTIN,LOW);
  if (stringComplete)
  {
    if (inputChar == 'w')
    {
      Serial.println("go up");
      openSolenoid();
      set_motor(UP, 80);
      startTime=millis();
      while(millis()<startTime+500 && digitalRead(EndStopSwitch)==1) ;
      set_motor(DOWN, 255);
      startTime=millis();
      while(millis()<startTime+10) ;
      set_motor(BREAK,255);
      closeSolenoid();
    }
    if (inputChar == 's')
    {
      Serial.println("go down");
      openSolenoid();
      set_motor(UP, 100);
      startTime=millis();
      while(millis()<startTime+20) ;
      set_motor(BREAK, 255);
      startTime=millis();
      while(millis()<startTime+10) ;
      set_motor(DOWN, 120);
      startTime=millis();
      while(millis()<startTime+500) ;
      set_motor(UP, 255);
      startTime=millis();
      while(millis()<startTime+10) ;
      set_motor(BREAK, 255);
      closeSolenoid();
      
    }
    else if (inputChar == 'e')
    {
      openSolenoid();
    }
    else if (inputChar == 'd')
    {
      closeSolenoid();
    }
    else if (inputChar == 't')
    {
      LoadCell.tareNoDelay();
      Serial.println("tare");
    }
    else
    {
      Serial.println("STOP");
      set_motor(STOP, 0);
      closeSolenoid();
    }
  }
  stringComplete = false;
//  Serial.println(md.getM2CurrentMilliamps());
  delay(10);
}



void serialEvent()
{
  if(Serial.available())
  {
    inputChar = Serial.read();
    stringComplete = true;
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
  Serial.println("solenoid open");
}

void closeSolenoid()
{
  digitalWrite(Solenoid,LOW);
  Serial.println("solenoid closed");
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
