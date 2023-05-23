# LoR_Core_JiminyPlusLaserTurret
This project holds all configurations for the MiniBot named jiminy plus a laser turret attachment.

Board : ESP32 dev module

This Arduino based code is for an ESP32-controlled robot equipped with a mecanum drive and a laser turret, both of which are controlled by a PS4 controller.

Key elements of this code are:

- Includes libraries for PS4 Controller and Adafruit_NeoPixel. This enables the PS4 controller to communicate with the robot and allows control of LED lights on the robot.

- Has defined pin configurations for the ESP32 to control the motors of the mecanum drive and the laser turret.

- Configures PWM channels and frequencies for the motors, allowing precise control over the robot's movement.

- Implements Neopixel strip configurations to display different colors on the robot.

- Establishes joystick control variables and dead zones to manage the robot's movement and rotation.

- Uses a slew rate to ramp up and down motor speeds smoothly, preventing abrupt changes that could harm the motors.

- Contains servo configurations for controlling the laser turret.

- In the main `setup()` function, the code initializes the Neopixels, sets up PWM functionalities for the LED lights and motors, configures the laser pin, and begins PS4 controller communication.

- Within the main `loop()` function, it handles PS4 controller inputs and serial input. If the PS4 controller is connected, the robot executes various operations like LED display, joystick control, servo control, and laser operation.

- It also includes functions to handle the mecanum drive control, which manages strafing and rotation values, as well as setting the motor output based on the input values.

- The `Servo_Control()` function converts joystick values to a corresponding PWM duty cycle, allowing control over the laser turret's position.

- Lastly, there's a `Laser()` function that turns on the laser when the R2 button on the PS4 controller is pressed.

Overall, this is a comprehensive codebase that allows full control over a complex robot using a PS4 controller.
