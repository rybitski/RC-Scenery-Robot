/* 
 Developed by Chris Rybitski 
  User accepts all risk and liability
  
   Connections:
   Sabertooth > Arduino Mega
           S1 > TX3 (Pin 14)
     Connect a 10K resistor from 5V to RX3
     This pulls the serial port HIGH so that it doesn't read spurious data should the receiver fail.

  SyRen  > Arduino
      S1 > TX1 (Pin 18)

   IBUS  > Arduino
    IBUS > RX2 (pin 17)


  Brake relay > pin 9
  Estop signal > pin 8

  Configuration Jumpers
                       10K
     5v----o o----41--/\/\/---GND
                       10K
     5v----o o----39--/\/\/---GND
                       10K
     5v----o o----37--/\/\/---GND

*/
#include "FlySkyIBus.h"
#include <Sabertooth.h>


Sabertooth ST(128, Serial3); // The Sabertooth is on address 128. We'll name its object ST.
Sabertooth SyRen(129, Serial1); //SyRen for linear actuarot


const int brake = 9;   //brake is on pin 9
const int estop = 8;   //input from wireless estop
const int set1 = 41;  //jumper settings
const int set2 = 39;  //jumper settings
const int set3 = 37;  //jumper settings
int steering;
int right;
int left;
int knob1;
int knob2;
int lift;
int toggle;

//values for ramping configuration
int rampMin;
int rampMax;

int heartBeat = 0;                       //counter to detect if IBUS is transmitting data
const int heartBeatTimeout = 100;              //counts required to trigger loss of IBUS routine

int counter = 0;                       //used for auto brake
const int autoBrakeThreshold = 700;    //counts requred to activate auto brake when ramping is set to low 700 is the shortest time before the motor stops
bool autoBrake = true;                 //stores the state of the brake

bool mixedMode = false;                //mixed mode variable for 1 or 2 joystick control

//RX Variables
char rightPower;    //stroes mapped value for sabertooth motor
char leftPower;     //stroes mapped value for sabertooth motor
char maxSpeed;      //stroes mapped value for max speed of motor
char ramp;          //stroes mapped value rampping curve on sabertooth
char oldRamp = 0;   //stroes mapped value for sabertooth motor
char liftSpeed;


void setup()
{
  Serial.begin(115200);     //for debugging

  IBus.begin(Serial2);      //for radio

  Serial3.begin(115200);      // 9600 is the default baud rate for Sabertooth packet serial.

  Serial1.begin(9600);
  Sabertooth::autobaud(Serial1); // Autobaud is for the whole serial line

  // ST.setBaudRate(9600);
  ST.setTimeout(100);     //turn off motors if no serial data for more than 100ms
  ST.setRamping(0);        //turn ramping off by default

  pinMode(brake, OUTPUT);
  pinMode(estop, INPUT);
  pinMode(set1, INPUT);
  pinMode(set2, INPUT);
  pinMode(set3, INPUT);


  //check the configuration jumpers
  //Jumper #1
  if (digitalRead(set1) == HIGH) {
    rampMin = 21;
    rampMax = 80;        //21-80 = intermediate ramp
  }
  else {
    rampMin = 11;     //11-20 = slow ramp
    rampMax = 20;
  }
  Serial.print("rampMin=");
  Serial.println(rampMin);
  Serial.print("rampMax=");
  Serial.println(rampMax);
  //Jumper #2
  if (digitalRead(set2) == HIGH) {
    mixedMode = true;
  }
  //Jumper #3



  // This setting does not persist between power cycles.
  // See the Packet Serial section of the documentation for what values to use
  // for the minimum voltage command. It may vary between Sabertooth models
  // (2x25, 2x60, etc.).
  //
  // On a Sabertooth 2x25, the value is (Desired Volts - 6) X 5.
  // So, in this sample, we'll make the low battery cutoff 12V: (12 - 6) X 5 = 30.
  //  ST.setMinVoltage(79);     //23.8v is fully discharged value of 79 for 24v battery bank
  //-----------------was not working at last test--------------------------------

}

void loop()
{
  IBus.loop();    //capture IBUS from receiver
  RX();           //extract IBUS data

  //IBUS has been missing for too long cut the motors
  if (heartBeat > heartBeatTimeout) {
    failsafe();
  }
  //only run drive method if IBUS is alive
  else {


    //Detect Failsafe. Radio is setup to send 990 as falisafe.
    //check to see if estop is still normal if not failsafe.

    if (digitalRead(estop) == HIGH && IBus.readChannel(6) >= 995) {

      if (mixedMode == false) {
        standardDrive();
      }
      else {
        mixedDrive();
      }

    }
    else {
      failsafe();

    }
  }

  //check to see if IBUS is alive
  if (Serial2.available()) {
    heartBeat = 0;
  }
  else {
    if (heartBeat < heartBeatTimeout + 10) { // don't overflow the counter
      heartBeat++;
    }
  }
}

void RX()
{

  //degub stuff

//  Serial.print(F("Knob1:"));
//  Serial.print(knob1);
//  Serial.print(F("    Turn:"));
//  Serial.print(steering);
//  Serial.print(F("    Knob2:"));
//  Serial.print(knob2);
//  Serial.print(F("    Left:"));
//  Serial.print(left);
//  Serial.print(F("    Right:"));
//  Serial.print(right);
//  Serial.print(F("    Switch:"));
//  Serial.print(toggle);
//  Serial.print(F("  "));
//  Serial.print(counter);
//  Serial.println();

  steering = IBus.readChannel(0);
  right = IBus.readChannel(1);
  left = IBus.readChannel(2);
  knob1 = IBus.readChannel(4);
  knob2 = IBus.readChannel(5);
  lift = IBus.readChannel(3);
  toggle = IBus.readChannel(6);

  if (toggle == 2000) {
    autoBrake = false;
  }
  else {
    autoBrake = true;
  }

  ramp = map(knob1, 1000, 2000, rampMin, rampMax);    //11-20 = slow ramp, 21-80 = fast ramp
  maxSpeed = map(knob2, 1000, 2000, 0, 127);

  if (oldRamp != ramp) {      //Only set ramping if the value has changed
    ST.setRamping(ramp);      //The Sabertooth saves this to EEPROM
    oldRamp = ramp;
  }
}


//standard drive for two joystick control
void standardDrive()
{
  rightPower = map(right, 1000, 2000, maxSpeed * (-1) , maxSpeed);
  leftPower = map(left, 1000, 2000, maxSpeed * (-1), maxSpeed);

  liftSpeed = map(lift, 1000, 2000, -127, 127);


  if (autoBrake == true) {
    if (rightPower < 5 && rightPower > -5 && leftPower < 5 && leftPower > -5) {
      if (counter < autoBrakeThreshold + 10) {
        counter++;    //increment counter
      }
      if (counter > autoBrakeThreshold) {
        digitalWrite(brake, LOW);   //turn on brake
      }
    }
    else {
      digitalWrite(brake, HIGH);  //turn brake off
      counter = 0;
    }

  }
  else {
    digitalWrite(brake, HIGH);  //turn brake off
  }

  if (toggle == 1500) {         //if lift is enabled don't drive the wheels
    SyRen.motor(liftSpeed);
  }
  else {                        //lift is disabled so drive the wheels and turn off the lift
    ST.motor(1, rightPower);
    ST.motor(2, leftPower);
    SyRen.motor(0);              //disable lift
  }
}

//mixed drive for single joystick control
void mixedDrive() {

  rightPower = map(right, 1000, 2000, maxSpeed * (-1) , maxSpeed);
  leftPower = map(steering, 1000, 2000, maxSpeed * (-1), maxSpeed);

  liftSpeed = map(lift, 1000, 2000, -127, 127);


  if (autoBrake == true) {
    if (rightPower < 5 && rightPower > -5 && leftPower < 5 && leftPower > -5) {
      if (counter < autoBrakeThreshold + 10) {
        counter++;    //increment counter
      }
      if (counter > autoBrakeThreshold) {
        digitalWrite(brake, LOW);   //turn on brake
      }
    }
    else {
      digitalWrite(brake, HIGH);  //turn brake off
      counter = 0;
    }

  }
  else {
    digitalWrite(brake, HIGH);  //turn brake off
  }

  if (toggle == 1500) {         //if lift is enabled don't drive the wheels
    SyRen.motor(liftSpeed);
  }
  else {                        //lift is disabled so drive the wheels and turn off the lift
    ST.turn(rightPower);
    ST.drive(leftPower);
    SyRen.motor(0);             //disable lift
  } 





}


//disables motors and turns on brake
void failsafe()
{
  ST.motor(1, 0);              //stop both motors
  ST.motor(2, 0);

  SyRen.motor(0);              //disable lift

  digitalWrite(brake, LOW);   //turn brake on

  Serial.println("Failsafe");
}
