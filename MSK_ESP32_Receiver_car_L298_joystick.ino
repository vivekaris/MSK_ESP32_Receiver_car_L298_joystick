//This code is developed by vivek kumar for joy stick for esp32 robot
#include <esp_now.h>
#include <WiFi.h>
//must use 38 pin
//Right motor
int enableRightMotor = 22;
int rightMotorPin1 = 16;
int rightMotorPin2 = 17;
//Left motor
int enableLeftMotor = 23;
int leftMotorPin1 = 18;
int leftMotorPin2 = 19;
#define BUILTIN_LED 2
#define IN_1 16 //rx2
#define IN_2 17 //tx2
// motor 2 settings
#define IN_3 18 //d18
#define IN_4 19 //d19


#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData receiverData;

bool throttleAndSteeringMode = false;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  if (len == 0)
  {
    digitalWrite(BUILTIN_LED, LOW);
    return;
  }
  digitalWrite(BUILTIN_LED, HIGH);
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.switchPressed;
  Serial.println(inputData);
  if (receiverData.switchPressed == true)
  {
    if (throttleAndSteeringMode == false)
    {
      throttleAndSteeringMode = true;
    }
    else
    {
      throttleAndSteeringMode = false;
    }
  }

  if (throttleAndSteeringMode)
  {
    throttleAndSteeringMovements();
  }
  else
  {
    simpleMovements();
  }

  lastRecvTime = millis();
}

void simpleMovements()
{
  if (receiverData.yAxisValue <= 130)       //Move car Forward
  {
    Serial.println("Move car Forward");
    //rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    Forward();
  }
  else if (receiverData.yAxisValue >= 175)   //Move car Backward
  {
    Serial.println("Move car Backward");
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValue >= 175)  //Move car Right
  {
    Serial.println("Move car Right");
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValue <= 130)   //Move car Left
  {
    Serial.println("Move car Left");
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                                      //Stop the car
  {
    Serial.println("Stop Car");
    //rotateMotor(0, 0);
    Stop();
  }
}

void throttleAndSteeringMovements()
{
  int throttle = map( receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map( receiverData.xAxisValue, 0, 254, -255, 255);
  int motorDirection = 1;

  if (throttle < 0)       //Move car backward
  {
    motorDirection = -1;
  }

  int rightMotorSpeed, leftMotorSpeed;
  rightMotorSpeed =  abs(throttle) - steering;
  leftMotorSpeed =  abs(throttle) + steering;
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

void setUpPinModes()
{
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);


  //Set up PWM for motor speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  rotateMotor(0, 0);
}


void Forward() {

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  // analogWrite(ENA, speedCar);
  Serial.println("forward");
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  //analogWrite(ENB, speedCar);
}

void Reverse() {

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  // analogWrite(ENA, speedCar);
  Serial.println("back");
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  // analogWrite(ENB, speedCar);
}

void Right() {

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  // analogWrite(ENA, speedCar);
  Serial.println("right");
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  // analogWrite(ENB, speedCar);
}

void Left() {

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  // analogWrite(ENA, speedCar);
  Serial.println("left");
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  //  analogWrite(ENB, speedCar);
}

void goAheadRight() {

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  //analogWrite(ENA, speedCar/speed_Coeff);
  Serial.println("ahead right");
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  // analogWrite(ENB, speedCar);
}



void goAheadLeft() {

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  // analogWrite(ENA, speedCar);
  Serial.println("ahead left");
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  // analogWrite(ENB, speedCar/speed_Coeff);
}


void Stop() {

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  //analogWrite(ENA, speedCar);
  Serial.println("stop");
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
  //  analogWrite(ENB, speedCar);
}

void setup()
{
  setUpPinModes();
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("ESP NOW Reciver.....");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    digitalWrite(BUILTIN_LED, LOW);
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT )
  {
    rotateMotor(0, 0);
    digitalWrite(BUILTIN_LED, LOW);
  }
}
