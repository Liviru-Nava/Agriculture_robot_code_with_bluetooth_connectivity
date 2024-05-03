#include <Servo.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define IR_SENSOR_RIGHT 52
#define IR_SENSOR_LEFT 53
#define IR_FRONT_SENSOR 51
#define IR_PLANT_STOP_SENSOR 50
#define MOTOR_SPEED 60
#define LED 13
#define BUZZER 12
#define WATER_PUMP 9
#define DHTTYPE DHT11
#define DHTPIN 48
#define MOISTURE_SENSOR A0

DHT dht(DHTPIN, DHTTYPE);

Servo drill_base;
Servo drill;
Servo seed;
Servo moisture;

//Right motor
int enableRightMotor = 6;
int rightMotorPin1 = 34;
int rightMotorPin2 = 35;

//Left motor
  int enableLeftMotor = 5;
int leftMotorPin1 = 30;
int leftMotorPin2 = 31;

int plantSensorValue;
int rightIRSensorValue;
int leftIRSensorValue;
int frontSensorValue;

int min_analog_value = 550;
int max_analog_value = 1000;
int percentage;

int drill_count = 0;

float temperature=0;
float humidity=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_FRONT_SENSOR, INPUT);

  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(IR_PLANT_STOP_SENSOR, INPUT);

  pinMode(WATER_PUMP, OUTPUT);
  digitalWrite(WATER_PUMP, HIGH);
  

  pinMode(MOISTURE_SENSOR, INPUT);

  rotateMotor(0, 0);

  drill_base.attach(2);
  drill.attach(3);
  seed.attach(4);
  moisture.attach(8);
  seed.write(180);
  moisture.write(90); 
  drill_base.write(60);
  drill.write(0);

  lcd.init();
  lcd.backlight();
  dht.begin();

  
}

void loop() {
  // put your main code here, to run repeatedly:

  //call functions
  readAndDisplayTemperatureToLCD();   //this worked, displaying both temperature and humidity

  //read IR sensor values
  rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  frontSensorValue = digitalRead(IR_FRONT_SENSOR);
  plantSensorValue = digitalRead(IR_PLANT_STOP_SENSOR);
  delay(10);

  if(frontSensorValue == LOW)   //this also worked, when the front sensor detects an object, the buzzer and the led are on
  {
    stopRobot();
    alarmBuzzerLED();
  }
  else if(plantSensorValue == LOW && drill_count != 2)  //this also worked, plant sensor detected, function 1 is complete
  {
    stopRobot();
    alarmBuzzerLEDOff();
    
    startDrill();
    drill_count++;
    delay(500);
    startSeeding();
    delay(500);
    if(plantSensorValue == LOW)
    {
      startWaterPump();
      delay(500);
      lcd.clear();
      readAndDisplayTemperatureToLCD();   //this worked, displaying both temperature and humidity
    }
    moveRobot();
  }
  else if(plantSensorValue == LOW && drill_count == 2)  //this worked, only issue is the servo and because of the servo the pump doesnt work
  {
    readAndDisplayTemperatureToLCD();   //this worked, displaying both temperature and humidity
    alarmBuzzerLEDOff();
    stopRobot();
    //digitalWrite(LED, HIGH);
    startMoistureReading();
    delay(2000);
    if(plantSensorValue == LOW)
    {
      if(percentage < 20)
      {
        startWaterPump();
      }
    }
    delay(1000);
    moveRobot();
  }
  else if(rightIRSensorValue == HIGH && leftIRSensorValue == HIGH)  //this worked, the robot stopped when black line detected by both IR
  {
    readAndDisplayTemperatureToLCD();   //this worked, displaying both temperature and humidity
    stopRobot();
    alarmBuzzerLEDOff();
    readTemperature();
    if(temperature >= 33.00)
    {
      moveRobot();
      startLineFollowing();
    }
  }
  else  //this also worked
  {
    readAndDisplayTemperatureToLCD();   //this worked, displaying both temperature and humidity
    alarmBuzzerLEDOff();
    startLineFollowing();
  }
}

//functions readAndDisplayTemperatureToLCD()
void readAndDisplayTemperatureToLCD()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  //display temperature to lcd
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
}

//function stopRobot()
void stopRobot()
{
  rotateMotor(0, 0);
}
void moveRobot()
{
  rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  delay(900);
  rotateMotor(0, 0);
}

//function alarmBuzzerLED()
void alarmBuzzerLED()
{
  digitalWrite(LED, HIGH);
  digitalWrite(BUZZER, HIGH);
}
void alarmBuzzerLEDOff()
{
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
}

//function startDrill()
void startDrill()
{
  for(int i = 60; i<=120; i++)
  {
    drill_base.write(i);
    delay(50);
  }
  delay(100);
  for(int i = 0; i<3; i++)
  {
    drill.write(180);
    delay(500);
    drill.write(0);
    delay(500);
  }
  delay(100);
  for(int i = 120; i>=60; i--)
  {
    drill_base.write(i);
    delay(50);
  }
}

//function startSeeding()
void startSeeding()
{
  for(int i =0; i<2; i++)
  {
    seed.write(180);
    delay(500);
    seed.write(170);
    delay(500);
    seed.write(180);
  }
}

//function startWaterPump()
void startWaterPump()
{
  digitalWrite(WATER_PUMP, LOW);
  delay(480);
  digitalWrite(WATER_PUMP, HIGH);
}

//function startMoistureReading()
void startMoistureReading()
{
  moisture.write(0);

  //read moisture data
  int moisture_value = analogRead(MOISTURE_SENSOR);
  percentage = map(moisture_value, min_analog_value, max_analog_value, 100, 0);
  lcd.clear();
  lcd.print("Moisture: ");
  lcd.print(percentage);
  lcd.print(" %");
  delay(3000);
  moisture.write(90);
}

//function readTemperature()
void readTemperature()
{
  temperature = dht.readTemperature();
}

//function startLineFollowing()
void startLineFollowing()
{
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) 
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) 
  {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  }
  //If left sensor detects black line, then turn left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) 
  {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  //If both the sensors detect black line, then stop
  else 
  {
    rotateMotor(0, 0);
  }
}

//function rotateMotor()
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {

  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}