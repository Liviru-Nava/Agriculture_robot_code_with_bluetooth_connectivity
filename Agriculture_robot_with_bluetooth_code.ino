/*CODE FOR EXHIBITION*/
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//right motor pins
int enableRightMotor = 2;
int rightMotorPin1 = 23;
int rightMotorPin2 = 22;

//left motor pins
int enableLeftMotor = 3;
int leftMotorPin1 = 25;
int leftMotorPin2 = 24;

#define FORWARD_SPEED 90 //motor speed
#define TURN_SPEED 130 

String readString;  //store the concatenated characters of the string received from mobile app

#define IR_PLANT_SENSOR 26  //IR sensor to detect plant plot
int plant_sensor_value;
#define WATER_SENSOR_PIN A0
int water_sensor_value;
#define WATER_PUMP_PIN 6

unsigned long previousMillis = 0;
const long interval = 2000;   // Check water level every 2000 milliseconds (2 second)

Servo drill_base_s1;
Servo drill_s2;
Servo seed_servo;

#define LED_OPERATIONS 28
#define LED_MOVEMENT 30

void setup()
{

  //pinmodes for the right motor
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  //pinmodes for the left motor
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  //pinmode for IR sensor for plant plot detection
  pinMode(IR_PLANT_SENSOR, INPUT);

  pinMode(WATER_PUMP_PIN, OUTPUT);
  digitalWrite(WATER_PUMP_PIN, HIGH);

  pinMode(WATER_SENSOR_PIN, INPUT);
  //serial monitor initialisation
  Serial.begin(9600);
  //Inbuilt LED output pinmode
  pinMode(LED_MOVEMENT, OUTPUT);
  pinMode(LED_OPERATIONS, OUTPUT);

  drill_base_s1.attach(5);
  drill_base_s1.write(150);
  drill_s2.attach(4);
  seed_servo.attach(8);

  lcd.init();
  lcd.backlight();
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    int water_sensor_value = analogRead(WATER_SENSOR_PIN);
    int water_percentage = map(water_sensor_value, 250, 560, 0, 100);
    Serial.println(water_sensor_value);
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Water Level: ");
    lcd.print(water_percentage);
    lcd.print("% ");
    // Check water level status
    if (water_sensor_value < 200) 
    {
      Serial.println("Water level is low");
      // Add actions when water level is low
    } else if (water_sensor_value > 500) 
    {
      Serial.println("Water level is full");
      // Add actions when water level is full
    }
  }
  int plant_sensor_value = digitalRead(IR_PLANT_SENSOR);
  if(plant_sensor_value == LOW)
  {
    digitalWrite(LED_OPERATIONS, HIGH);
    digitalWrite(LED_MOVEMENT, LOW);
    stopRobot();
    delay(500);
    performAutomaticPlantingOperations();
    delay(500);
    moveForward();
    delay(350);
    stopRobot();
    digitalWrite(LED_OPERATIONS, LOW);
    digitalWrite(LED_MOVEMENT, HIGH);
  }
  else
  {
    digitalWrite(LED_MOVEMENT, HIGH);
    while(Serial.available()) //while a value is being sent from the mobile application
    {
      delay(50);
      char c=Serial.read(); 
      readString+=c;  //retreive the characters from the string and store into the String variable
    }
    if(readString.length()>0)
    {
      Serial.println(readString);
      if (readString =="FORWARD")
      {     
        digitalWrite(LED_MOVEMENT, HIGH);
        moveForward();
      }
      if (readString =="BACKWARD")
      {     
        // MOVE BACKWARD IF BACKWARD BUTTON IS PRESSED
        moveBackward();
        digitalWrite(LED_MOVEMENT, HIGH);
      }
      if (readString =="LEFT")
      {     // MOVE LEFT SIDE
        moveLeft();
        digitalWrite(LED_MOVEMENT, HIGH);
      }
      if (readString =="RIGHT")
      {     // MOVE RIGHT SIDE
        moveRight();
        digitalWrite(LED_MOVEMENT, HIGH);
      }
      if (readString =="STOP")
      {     // STOP
        Serial.println("Stop command received");
        stopRobot();
        digitalWrite(13, LOW);
      }
      readString="";
    }
  } 
}

//Move robot forward
void moveForward()
{
  rotateMotor(FORWARD_SPEED, FORWARD_SPEED);
}
void moveBackward()
{
  rotateMotor(-FORWARD_SPEED, -FORWARD_SPEED);
}
void moveLeft()
{
  rotateMotor(TURN_SPEED, -TURN_SPEED);
}
void moveRight()
{
  rotateMotor(-TURN_SPEED, TURN_SPEED);
}
void stopRobot()
{
  rotateMotor(0, 0);
}
//function to perform plant operations
void performAutomaticPlantingOperations()
{
  //perform drill operation code
  plotDrilling();
  delay(1000);
  seedSowing();
  delay(1000);
  waterPumping();
  delay(1000);
}
//function to drill the soil
void plotDrilling()
{
  for(int i=150; i>=70; i-=0.1)  //drop base to ground
  {
    drill_base_s1.write(i);
    delay(40);
  }
  delay(1000);
  for(int i=0; i<3; i++)  //drill the soil
  {
    drill_s2.write(180);
    delay(500);
    drill_s2.write(40);
    delay(500);
  }
  for(int i=70; i<=150; i++)  //raise the base from ground
  {
    drill_base_s1.write(i);
    delay(80);
  }
}
//function to sow seeds
void seedSowing()
{
  seed_servo.write(160);
  delay(400);
  seed_servo.write(80);
  delay(400);
}
//function to pump water
void waterPumping()
{
  digitalWrite(WATER_PUMP_PIN, LOW);
  delay(60);
  digitalWrite(WATER_PUMP_PIN, HIGH);
}
//function rotateMotor()
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {

  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
