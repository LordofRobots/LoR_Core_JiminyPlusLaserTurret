/* LORD of ROBOTS - LoR_Core_LaserTurret - 202305222115
  Control inputs - LED Indication:
    PS4 control - Rainbow LED
    none/Stop/standby - Red LED

  Drive configurations:
    Mecanum 
    Standard tank style

*/

//////////////////////////////////////////////////////////////////////////
/////                         Libraries                              /////
//////////////////////////////////////////////////////////////////////////
#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>
#include <Adafruit_NeoPixel.h>


//////////////////////////////////////////////////////////////////////////
/////                       Definitions                              /////
//////////////////////////////////////////////////////////////////////////
// version control and major control function settings
String Version = "Base Version : LoR Core Laser turret : 1.2.1";

// IO Interface Definitions

#define ControllerSelectPin 34
#define MotorEnablePin 13

// Motor Pin Definitions
#define motorPin_M1_A 5
#define motorPin_M1_B 14
#define motorPin_M2_A 18
#define motorPin_M2_B 26
#define motorPin_M3_A 23
#define motorPin_M3_B 19
#define motorPin_M4_A 15
#define motorPin_M4_B 33
#define motorPin_M5_A 27
#define motorPin_M5_B 25
#define motorPin_M6_A 32
#define motorPin_M6_B 4
const int motorPins_A[] = { motorPin_M1_A, motorPin_M2_A, motorPin_M3_A, motorPin_M4_A, motorPin_M5_A, motorPin_M6_A };
const int motorPins_B[] = { motorPin_M1_B, motorPin_M2_B, motorPin_M3_B, motorPin_M4_B, motorPin_M5_B, motorPin_M6_B };

// PWM Configuration Definitions
const int Motor_M1_A = 0;
const int Motor_M1_B = 1;
const int Motor_M2_A = 2;
const int Motor_M2_B = 3;
const int Motor_M3_A = 4;
const int Motor_M3_B = 5;
const int Motor_M4_A = 6;
const int Motor_M4_B = 7;
const int Motor_M5_A = 8;
const int Motor_M5_B = 9;
const int Motor_M6_A = 10;
const int Motor_M6_B = 11;
const int MOTOR_PWM_Channel_A[] = { Motor_M1_A, Motor_M2_A, Motor_M3_A, Motor_M4_A, Motor_M5_A, Motor_M6_A };
const int MOTOR_PWM_Channel_B[] = { Motor_M1_B, Motor_M2_B, Motor_M3_B, Motor_M4_B, Motor_M5_B, Motor_M6_B };
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;

//////////////////////////////////////////////////////////////////////////
/////                    Motion Control                              /////
//////////////////////////////////////////////////////////////////////////
// Process joystick input and calculate motor speeds - Mecanum control
// Joystick control variables
const int DEAD_BAND = 20;
const float TURN_RATE = 1;
bool MecanumDrive_Enabled = true;
int Motor_FrontLeft_SetValue, Motor_FrontRight_SetValue, Motor_BackLeft_SetValue, Motor_BackRight_SetValue = 0;
void Motion_Control(int LY_Axis, int LX_Axis, int RX_Axis) {
  int FrontLeft_TargetValue, FrontRight_TargetValue, BackLeft_TargetValue, BackRight_TargetValue = 0;
  int ForwardBackward_Axis = LY_Axis;
  int StrafeLeftRight_Axis = LX_Axis;
  int TurnLeftRight_Axis = -RX_Axis;

  //Set deadband
  if (abs(ForwardBackward_Axis) < DEAD_BAND) ForwardBackward_Axis = 0;
  if (abs(StrafeLeftRight_Axis) < DEAD_BAND) StrafeLeftRight_Axis = 0;
  if (abs(TurnLeftRight_Axis) < DEAD_BAND) TurnLeftRight_Axis = 0;

  //Calculate strafe values
  FrontLeft_TargetValue = -ForwardBackward_Axis + (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  BackLeft_TargetValue = -ForwardBackward_Axis - (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  FrontRight_TargetValue = ForwardBackward_Axis + (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  BackRight_TargetValue = ForwardBackward_Axis - (StrafeLeftRight_Axis * MecanumDrive_Enabled);

  //calculate rotation values
  if (abs(TurnLeftRight_Axis) > DEAD_BAND) {
    FrontLeft_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    BackLeft_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    FrontRight_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    BackRight_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
  }

  //constrain to joystick range
  FrontLeft_TargetValue = constrain(FrontLeft_TargetValue, -127, 127);
  BackLeft_TargetValue = constrain(BackLeft_TargetValue, -127, 127);
  FrontRight_TargetValue = constrain(FrontRight_TargetValue, -127, 127);
  BackRight_TargetValue = constrain(BackRight_TargetValue, -127, 127);

  //set motor speed through slew rate function
  Motor_FrontLeft_SetValue = SlewRateFunction(FrontLeft_TargetValue, Motor_FrontLeft_SetValue);
  Motor_FrontRight_SetValue = SlewRateFunction(FrontRight_TargetValue, Motor_FrontRight_SetValue);
  Motor_BackLeft_SetValue = SlewRateFunction(BackLeft_TargetValue, Motor_BackLeft_SetValue);
  Motor_BackRight_SetValue = SlewRateFunction(BackRight_TargetValue, Motor_BackRight_SetValue);
}

//////////////////////////////////////////////////////////////////////////
/////                         Slew Rate                              /////
//////////////////////////////////////////////////////////////////////////
// Function to handle slew rate for motor speed ramping
// Slew rate for ramping motor speed
const int SLEW_RATE_MS = 20;
int SlewRateFunction(int Input_Target, int Input_Current) {
  int speedDiff = Input_Target - Input_Current;
  if (speedDiff > 0) Input_Current += min(speedDiff, SLEW_RATE_MS);
  else if (speedDiff < 0) Input_Current -= min(-speedDiff, SLEW_RATE_MS);
  constrain(Input_Current, -127, 127);
  return Input_Current;
}

//////////////////////////////////////////////////////////////////////////
/////                      Motor Output                              /////
//////////////////////////////////////////////////////////////////////////
// Function to control motor output based on input values
// Motor speed limits and starting speed
const int MAX_SPEED = 255;
const int MIN_SPEED = -255;
const int MIN_STARTING_SPEED = 140;
const int STOP = 0;
const int SerialControl_SPEED = 110;
bool INVERT = false;
void Set_Motor_Output(int Output, int Motor_ChA, int Motor_ChB) {
  if (INVERT) Output = -Output;

  Output = constrain(Output, -127, 127);

  int Mapped_Value = map(abs(Output), 0, 127, MIN_STARTING_SPEED, MAX_SPEED);
  int A, B = 0;
  if (Output < -DEAD_BAND) {  // Rotate Clockwise
    A = 0;
    B = Mapped_Value;
  } else if (Output > DEAD_BAND) {  // Rotate Counter-Clockwise
    A = Mapped_Value;
    B = 0;
  } else {  // Rotation Stop
    A = STOP;
    B = STOP;
  }
  ledcWrite(Motor_ChA, A);  //send to motor control pins
  ledcWrite(Motor_ChB, B);
}

//////////////////////////////////////////////////////////////////////////
/////            Motor Output to Phycial motor                       /////
//////////////////////////////////////////////////////////////////////////
void Motor_Control() {
  Set_Motor_Output(Motor_FrontLeft_SetValue, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(Motor_BackLeft_SetValue, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(Motor_FrontRight_SetValue, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(Motor_BackRight_SetValue, Motor_M6_A, Motor_M6_B);
}

void Motor_STOP() {
  Set_Motor_Output(STOP, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(STOP, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(STOP, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(STOP, Motor_M6_A, Motor_M6_B);
}

//////////////////////////////////////////////////////////////////////////
/////                     Servo Control                              /////
//////////////////////////////////////////////////////////////////////////
// Function to control the servo position using LEDC library
// Servo Configurations
const int servoPin = 22;
const int ServoPosition_Center = 45;
const int ServoPWM_FREQUENCY = 50;
const int SERVO_PWM_Channel = 12;
const int ServoPWM_RESOLUTION = 12;
const int minPulseWidth = 1000;  // Pulse width range in microseconds
const int maxPulseWidth = 2000;
int SERVO_PWM_MIN = minPulseWidth / ((1000000 / ServoPWM_FREQUENCY) / 4095);  // Duty cycle range for SERVO_PWM_Channel - Calculate SERVO_PWM_MIN and SERVO_PWM_MAX
int SERVO_PWM_MAX = maxPulseWidth / ((1000000 / ServoPWM_FREQUENCY) / 4095);
void Servo_Control(int joystickValue) {
  //Serial.print("joystickValue: " + String(joystickValue));                    // Convert the servo position to a corresponding PWM duty cycle
  int pwmDuty = map(joystickValue, -127, 127, SERVO_PWM_MIN, SERVO_PWM_MAX);  // Map joystick value to servo position (0-180 degrees)
  ledcWrite(SERVO_PWM_Channel, pwmDuty);                                      // Set the PWM duty cycle for the servo
}

//////////////////////////////////////////////////////////////////////////
/////                     Laser Control                              /////
//////////////////////////////////////////////////////////////////////////
//laser configurations
const int laserPin = 21;
void Laser(int LaserState) {
  if (LaserState) digitalWrite(laserPin, true);
  else digitalWrite(laserPin, false);
}

//////////////////////////////////////////////////////////////////////////
/////                         NeoPixel                               /////
//////////////////////////////////////////////////////////////////////////
// NeoPixel Configurations
#define StripLED_DataPin 12
#define HeadLightLED_DataPin 17
#define StripLED_COUNT 20
#define HeadLightLED_COUNT 2
Adafruit_NeoPixel strip(StripLED_COUNT, StripLED_DataPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel HeadLight(HeadLightLED_COUNT, HeadLightLED_DataPin, NEO_GRBW + NEO_KHZ800);

// Colour presets
const uint32_t RED = strip.Color(255, 0, 0, 0);
const uint32_t GREEN = strip.Color(0, 255, 0, 0);
const uint32_t BLUE = strip.Color(0, 0, 255, 0);
const uint32_t WHITE = strip.Color(255, 255, 255, 255);
const uint32_t PURPLE = strip.Color(255, 0, 255, 0);
const uint32_t CYAN = strip.Color(0, 255, 255, 0);
const uint32_t YELLOW = strip.Color(255, 255, 0, 0);

const uint32_t RED_H = HeadLight.Color(255, 0, 0, 0);
const uint32_t GREEN_H = HeadLight.Color(0, 255, 0, 0);
const uint32_t BLUE_H = HeadLight.Color(0, 0, 255, 0);
const uint32_t WHITE_H = HeadLight.Color(255, 255, 255, 255);
const uint32_t PURPLE_H = HeadLight.Color(255, 0, 255, 0);
const uint32_t CYAN_H = HeadLight.Color(0, 255, 255, 0);
const uint32_t YELLOW_H = HeadLight.Color(255, 255, 0, 0);

// Set a specific color for the entire NeoPixel strip
void NeoPixel_SetColour(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {     // For each pixel in strip...
    strip.setPixelColor(i, color);  //  Set pixel's color (in RAM)
    strip.show();                   // Update strip with new contents
  }
}
void NeoPixel_SetPixel(int PixelNum, uint32_t color) {
  strip.setPixelColor(PixelNum, color);  //  Set pixel's color (in RAM)
  strip.show();                          // Update strip with new contents
}
void NeoPixel_SetHeadLight(int PixelNum, uint32_t color) {
  HeadLight.setPixelColor(PixelNum, color);  //  Set pixel's color (in RAM)
  HeadLight.show();                          // Update strip with new contents
}
// Rainbow pattern for NeoPixel strip
long firstPixelHue = 0;
void NeoPixel_Rainbow() {
  strip.rainbow(firstPixelHue);
  firstPixelHue += 256;
  if (firstPixelHue >= 5 * 65536) firstPixelHue = 0;
    strip.show();                          // Update strip with new contents

}

//////////////////////////////////////////////////////////////////////////
/////                       Head Lights                              /////
//////////////////////////////////////////////////////////////////////////
//Contol last pixel in string as Headlights
uint32_t Off = 0;
uint32_t Colour = Off;
bool ToggleModeState = 0;
long HoldTime = 4000;
void LED_Functions(bool Green, bool Red, bool Blue, bool White) {
  HeadLight.clear();
  // NeoPixel_SetHeadLight(0,GREEN_H);


  if (Red || Green || Blue || White) {
    if (millis() > HoldTime) {
      ToggleModeState = !ToggleModeState;
      HeadLight.clear();
        HeadLight.show();                          // Update strip with new contents
      delay(100);
      HoldTime = millis() + 1000;
    }
  } else HoldTime = millis() + 1000;

  if (Green) Colour = GREEN_H;
  else if (Red) Colour = RED_H;
  else if (Blue) Colour = BLUE_H;
  else if (White) Colour = WHITE_H;
  else if (!ToggleModeState) Colour = Off;

  // NeoPixel_SetPixel(3, Colour);
    NeoPixel_SetHeadLight(0, Colour);
    NeoPixel_SetHeadLight(1, Colour);

}

//////////////////////////////////////////////////////////////////////////
/////                     PS4 Battery Check                          /////
//////////////////////////////////////////////////////////////////////////
// check battery status of the ps4 controller
long DelaySerialPrint = 0;
void PS4controller_BatteryCheck() {
  if (millis() > DelaySerialPrint) {
    Serial.printf("Controller Battery Level : %d\n", PS4.Battery());
    DelaySerialPrint = millis() + 1000;
  }
  if (PS4.Charging()) PS4.setFlashRate(1000, 1000);
  else PS4.setFlashRate(0, 0);
  if (PS4.Battery() > 5) PS4.setLed(0, 255, 0);
  else if (PS4.Battery() > 2) PS4.setLed(255, 255, 0);
  else {
    PS4.setFlashRate(500, 500);
    PS4.setLed(255, 0, 0);
    PS4.setRumble(50, 50);
  }
  PS4.sendToController();
}

//////////////////////////////////////////////////////////////////////////
/////                   PS4 Connection Rumble                        /////
//////////////////////////////////////////////////////////////////////////
// controller rumble control for connecting
bool Connected_Rumble = false;
void Rumble_Once() {
  if (Connected_Rumble) return;
  Connected_Rumble = true;
  PS4.setRumble(255, 255);
  PS4.sendToController();
  delay(500);
  PS4.setRumble(0, 0);
  PS4.sendToController();
}

//////////////////////////////////////////////////////////////////////////
/////                      Startup Tones                             /////
//////////////////////////////////////////////////////////////////////////
// Tones created in the motors. Cycle through each motor.
void Start_Tone() {
  for (int i = 0; i < 6; i++) {
    long ToneTime = millis() + 200;
    bool state = 0;
    while (millis() < ToneTime) {
      digitalWrite(motorPins_A[i], state);
      digitalWrite(motorPins_B[i], !state);
      state = !state;
      long WaitTime = micros() + (100 * (i + 1));
      while (micros() < WaitTime) {}
    }
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
    delay(50);
  }
}

//////////////////////////////////////////////////////////////////////////
/////                         SetUp                                  /////
//////////////////////////////////////////////////////////////////////////
// Set up pins, LED PWM functionalities and begin PS4 controller, Serial and Serial2 communication
void setup() {
  // Set up the pins
  pinMode(StripLED_DataPin, OUTPUT);
  pinMode(HeadLightLED_DataPin, OUTPUT);
  pinMode(ControllerSelectPin, INPUT_PULLUP);
  pinMode(MotorEnablePin, OUTPUT);

  for (int i = 0; i < 6; i++) {
    pinMode(motorPins_A[i], OUTPUT);
    pinMode(motorPins_B[i], OUTPUT);
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
  }

  // output preset bias
  digitalWrite(StripLED_DataPin, 0);
  digitalWrite(HeadLightLED_DataPin, 0);
  digitalWrite(MotorEnablePin, 1);

  // Neopixels Configuration
  strip.begin();            // INITIALIZE NeoPixel strip object
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(100);  // Set BRIGHTNESS to about 1/2 (max = 255)
   // Neopixels Configuration
  HeadLight.begin();            // INITIALIZE NeoPixel strip object
  HeadLight.show();             // Turn OFF all pixels ASAP
  HeadLight.setBrightness(100);  // Set BRIGHTNESS to about 1/2 (max = 255)

  // Motor test tones
  NeoPixel_SetColour(BLUE);
  Start_Tone();

  // configure LED PWM functionalitites
  for (int i = 0; i < 6; i++) {
    ledcSetup(MOTOR_PWM_Channel_A[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_PWM_Channel_B[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motorPins_A[i], MOTOR_PWM_Channel_A[i]);
    ledcAttachPin(motorPins_B[i], MOTOR_PWM_Channel_B[i]);
  }

  // Configure LEDC for servo PWM
  const int ServoPWM_FREQUENCY = 50;
  const int SERVO_PWM_Channel = 12;
  ledcSetup(SERVO_PWM_Channel, ServoPWM_FREQUENCY, ServoPWM_RESOLUTION);
  ledcAttachPin(servoPin, SERVO_PWM_Channel);

  // laser setup
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, false);
  NeoPixel_SetColour(PURPLE);

  // PS4 controller configuration (Target mac address saved on the controller)
  // PS4.begin("e8:9e:b4:23:db:01");  // REPLACE WITH THE MAC ADDRESS FROM YOUR PS4 CONTROLLER
  // PS4.begin("e8:9e:23:10:03:01");  // REPLACE WITH THE MAC ADDRESS FROM YOUR PS4 CONTROLLER
  PS4.begin("a8:47:4a:67:89:01");  // REPLACE WITH THE MAC ADDRESS FROM YOUR PS4 CONTROLLER

  NeoPixel_SetColour(YELLOW);

  // Serial comms configurations (USB for debug messages)
  Serial.begin(115200);  // USB Serial
  delay(2000);

  Serial.print("Drive Style: ");
  if (MecanumDrive_Enabled) Serial.println("MECANUM");
  else Serial.println("STANDARD");
  NeoPixel_SetColour(CYAN);

  Serial.println("CORE System Ready! " + Version);
  NeoPixel_SetColour(WHITE);
}

//////////////////////////////////////////////////////////////////////////
/////                         Main Loop                              /////
//////////////////////////////////////////////////////////////////////////
void loop() {
  // Main loop to handle PS4 controller and serial input
  //PS4 Control
  if (PS4.isConnected()) {
    Rumble_Once();
    NeoPixel_Rainbow();
    PS4controller_BatteryCheck();
    LED_Functions(PS4.Triangle(), PS4.Circle(), PS4.Cross(), PS4.Square());
    Motion_Control(-PS4.LStickY(), -PS4.LStickX(), -PS4.RStickX());  // Joystick control
    delay(5);
    Servo_Control(PS4.RStickY());
    delay(5);
    Laser(PS4.R2());
    Motor_Control();

    //Stop/Standby
  } else {
    NeoPixel_SetColour(RED);
    Motor_STOP();
    Connected_Rumble = false;
  }
}
