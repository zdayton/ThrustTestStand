


#include <EEPROM.h>
#include <Servo.h> 

//IO pins
int datPin = 9;
int clkPin=8;
int voltagePin=A0;
int currentPin=A1;
int potentiometerPin=A2;
int potHigh=7;
int potLow=6;
int ESCPin=12;
int probePowerHigh=5;
int probeVOfsPin=A3;

Servo ESC;
char character;
String input;
String calibrationWeight;
float tare=0;
float calibration;
int calibrationmass;
float throttle;
unsigned long startTime;

union f_bytes
{
  byte b[4];
  float fval;
}u;

float readFloatFromEEPROM(int address)
{
  for(char i=0;i<4;i++)
  {
    u.b[i]=EEPROM.read(address+i);
  }
  //if the read flaot is nan, clear the eeprom
  if(u.fval!=u.fval)
  {
    EEPROM.write(address,0);
    EEPROM.write(address+1,0);
    EEPROM.write(address+2,0);
    EEPROM.write(address+3,0);
  }
  return u.fval;
}
void saveFloatToEEPROM(float toSave,int address)
{
  u.fval=toSave;
  for(char i=0;i<4;i++)
  {
    EEPROM.write(address+i,u.b[i]);
  }
  
}
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  // make the pushbutton's pin an input:
  pinMode(datPin, INPUT);
  pinMode(clkPin,OUTPUT);
  pinMode(potHigh,OUTPUT);
  pinMode(potLow,OUTPUT);
  pinMode(probePowerHigh,OUTPUT);
  //attach ESC servo output
  ESC.attach(ESCPin,1050,1990);
  
  //high and low legs of the pot, because there aren't enough 5v and gnd pins on the uno
  digitalWrite(potHigh,1);
  digitalWrite(potLow,0);
  digitalWrite(probePowerHigh,1);
  
  tare = readFloatFromEEPROM(0);
  calibration=readFloatFromEEPROM(4);  
}

float readPot()
{
  return (float)analogRead(potentiometerPin)/1024.0f;
}
void setThrottle()
{
  ESC.write((int)(throttle*180.0f));
}

float readLoadCell()
{
  unsigned long load=0;
  float loadOut;
  while(digitalRead(datPin));
  for(char i=0;i<24;i++)
  {
    digitalWrite(clkPin,1);
    load=load<<1;
    digitalWrite(clkPin,0);
    if(digitalRead(datPin))
      load++;
  }
  digitalWrite(clkPin,1);
  load=load^0x800000;
  loadOut = (float)load/128.0f;
  digitalWrite(clkPin,0);
  return loadOut;
}

//voltage scaling of AttoPilot 90A sensor is 63.69mV/V
//scaling factor is 0.07666529282
//Multiply by 1.04828 because arduino input impedence is a little low and dividing the voltage a little
float readVoltage()
{
  int sensorIn=analogRead(voltagePin)-analogRead(probeVOfsPin);
  return (float)sensorIn*0.076665f*1.04828f;
}

//current scaling of AttoPilot 90A sensor is 36.60mV/Amp
//scaling factor is 0.13341017759
float readCurrent()
{
  int sensorIn=analogRead(voltagePin)-analogRead(probeVOfsPin);
  return (float)sensorIn*0.133410f;
}

// the loop routine runs over and over again forever:
void loop() {
  ESC.write(0);
  Serial.println("Type Tare, Calibrate, Start, or Free");
  input="";
  while(!Serial.available());
  while(Serial.available())
  {
      character = Serial.read();
      input.concat(character);
      delay(1);
  }
  if(input=="Tare")
  {
    input="";
    Serial.println("Taring");
    tare=readLoadCell();
    saveFloatToEEPROM(tare,0);
  }
  if(input=="Calibrate")
  {
    input="";
    Serial.println("You must Tare before calibrating.  To exit calibration without saving a new value, type Exit. Otherwise enter the calibration mass in grams, without units");
    while(!Serial.available());
    while(Serial.available())
    {
        character = Serial.read();
        input.concat(character);
        delay(1);
    }
    if(input!="Exit")
    {
      calibrationmass=input.toInt();
      Serial.print("Calibration mass: ");
      Serial.println(calibrationmass);
      calibration=(float)calibrationmass/(readLoadCell()-tare);
      saveFloatToEEPROM(calibration,4);
      Serial.print("New measured mass: ");
      Serial.println((readLoadCell()-tare)*calibration);
    }
  }
  if(input=="Free")
  {
    input="";
    Serial.println("Begining free run, press any key to exit");
    Serial.println("Thrust(g),Throttle(%),Time(ms)");
    startTime=millis();
    while(!Serial.available())
    {
      throttle=readPot();
      setThrottle();
      Serial.print((readLoadCell()-tare)*calibration);
      Serial.print(",");
      //Serial.print(readVoltage());
      //Serial.print(",");
      //Serial.print(readCurrent());
      //Serial.print(",");
      Serial.print(throttle);
      Serial.print(",");
      Serial.println(millis()-startTime);
    }
    while(Serial.available())
    {
        character = Serial.read();
        delay(1);
    }
  }
  if(input=="Start")
  {
    input="";
    Serial.println("Begining automated test, press any key to exit");
    Serial.println("Thrust(g),Throttle(%),Time(ms)");
    startTime=millis();
    while(!Serial.available() && (millis()-startTime)<18000)
    {
      if((millis()-startTime)<2000)
        throttle=0.25;
      else if((millis()-startTime)<4000)
        throttle=0.1;
      else if((millis()-startTime)<6000)
        throttle=0.50;
      else if((millis()-startTime)<8000)
        throttle=0.1;
      else if((millis()-startTime)<10000)
        throttle=1.0;
      else if((millis()-startTime)<12000)
        throttle=0.0;
      else if((millis()-startTime)<18000)
        throttle=(float)(millis()-startTime-12000)/6000.0;
      else
        throttle=0.0;
      setThrottle();
      Serial.print((readLoadCell()-tare)*calibration);
      Serial.print(",");
      //Serial.print(readVoltage());
      //Serial.print(",");
      //Serial.print(readCurrent());
      //Serial.print(",");
      Serial.print(throttle);
      Serial.print(",");
      Serial.println(millis()-startTime);
    }
    while(Serial.available())
    {
        character = Serial.read();
        delay(1);
    }
  }
  delay(1);        // delay in between reads for stability
}

