/*
      NITRO Clubs EU - Network of IcT Robo Clubs
 
 WEB site: https://www.nitroclubs.eu 
 GitHub repositories: https://github.com/nitroclubs?tab=repositories 
 
      NITRObot Maze solver

 NITRObot is equipped with different sensors and designed to let you simulate tasks from the industrial robotics world. 
 
 In this program, we will use NITRObot's ultrasonic sensor and the hobby servo to perform the  
 "right hand rule" maze solving algorithm 
 
 Find the detailed instructions in NITRObot_maze_solver_EN.docx
 There are translations available in Bulgarian, Romanian and Slovak languages.
*/

// Robot parameters:
// NITRObot length is 25.0 cm.
// NITRObot width is  16.7 cm.

// Maze parameters:
// In order for the robot to be able to safely make an U turn,
// we will choose the maze width to be 3 times the robot width,
// which is equal to 50.1, we will approximate this value to 50 cm.


//====== INCLUDE ======
#include <Arduino.h>

#include <Servo.h> // Servo library

//====== DEFINE ======
#define MOTOR_LEFT_FWD_PIN 9    // PWMB
#define MOTOR_LEFT_BKWD_PIN 5   // DIRB  ---  Left
#define MOTOR_RIGHT_FWD_PIN 6   // PWMA
#define MOTOR_RIGHT_BKWD_PIN 10 // DIRA  ---  Right

//====== CONSTANTS ======
const int UltrasonicPin = 3;
const int ServoPin = 13;

const int WallFollowingSide = -90;      //Set: -90 for right wall following or +90 for left wall following
                                        //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                        // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)

const int LeftSpeed = 100;  // !Replace this value with the one obtained from calibration! (using NITRObot_motor_calibration.ino)
const int RightSpeed = 100; // !Replace this value with the one obtained from calibration! (using NITRObot_motor_calibration.ino)

//====== VARIABLES ======
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;

//====== Instantiate (create) an instance of the Servo object (class), named servo
Servo servo;

//====== FUNCTION FORWARD DECLARATIONS ======
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();
float getDistance(int servoAngle); // Read the Ultasonic Sensor pointing at the given servo angle

//-----------------------------------------------

void setup()
{
  pinMode(ServoPin, OUTPUT);
  pinMode(MOTOR_LEFT_FWD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_BKWD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_BKWD_PIN, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  servo.attach(ServoPin);
  servo.write(90); //Move the servo to center position

  moveForward();
  delay(1000);
}

//---------------------------------------------------------

void loop()
{

  float frontDistance, sideDistance;

  int robotPosition = 0;

  frontDistance = getDistance(FrontServoAngle);
  sideDistance = getDistance(SideServoAngle);

  if (frontDistance <= 20.0) //Close to the wall in front of the robot
  {
    robotPosition = 1;
  } 
  else if (sideDistance >= 50.0) //The wall on the right is far away - turn to the wall
    {
      robotPosition = 2;
    }
   else if (sideDistance < 35.0 && sideDistance >= 29.0)      //    |_________|__ROBOT__|_________|_________|_________|     The robot is to the left from the centerline treshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      robotPosition = 3;
    }
  else if (sideDistance > 15.0 && sideDistance <= 21.0)       //    |_________|_________|_________|__ROBOT__|_________|     The robot is to the right from the centerline treshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      robotPosition = 4;
    }
  else if (sideDistance <= 15.0)                              //    |_________|_________|_________|_________|__ROBOT__|     The robot is on the far right of the corridor
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      robotPosition = 5;
    }
  else if (sideDistance >= 35.0 && sideDistance < 50.0)       //    |__ROBOT__|_________|_________|_________|_________|     The robot is on the far left of the corridor
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      robotPosition = 6;
    }
  else if (sideDistance >= 21.0 && sideDistance < 29.0)       //    |_________|_________|__ROBOT__|_________|_________|     The robot is inside the center line threshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      robotPosition = 7;
    } 
     
  switch (robotPosition)
  {
  case 1: // Turn 90 degrees left
    moveBackward();
    delay(100);
    speedLeft = LeftSpeed * 1.35;
    turnLeft();
    delay(700);  // 90 degree turn timing
     speedLeft = LeftSpeed;
     speedRight = RightSpeed;
    moveBackward();
    delay(100);
    break;
  case 2: // Turn 90 degrees right
      speedLeft = LeftSpeed;
     speedRight = RightSpeed;
     moveForward();
     delay(600);    // Move 20 cm forward timing
     speedRight = RightSpeed * 1.35;
     turnRight();
     delay(700);  // 90 degree turn timing
     speedLeft = LeftSpeed;
     speedRight = RightSpeed;
     moveForward();
     delay(750);   // Move 25 cm forward timing
    break;
     case 3: // Turn slight right
    speedRight = RightSpeed * 2.55;
    speedLeft = LeftSpeed * 2.55;
    turnRight();
    delay(50);
    break;
  case 4: // Turn slight left
    speedLeft = LeftSpeed * 2.55;
    speedRight = RightSpeed * 2.55;
    turnLeft();
    delay(50);
    break;
  case 5: // Turn  more agressive to the left
    speedLeft = LeftSpeed * 2.55;
    speedRight = RightSpeed * 2.55;
    turnLeft();
    delay(100);
    break;
  case 6: // Turn  more agressive to the right
    speedRight = RightSpeed * 2.55;
    speedLeft = LeftSpeed * 2.55;
    turnRight();
    delay(100);
    break;
  case 7: // Go forward
  // Nothing to do here, continue with the execution of the loop
    break;
  default:
    break;
  }
  speedRight = RightSpeed ;
  speedLeft = LeftSpeed;
  moveForward();
}

//============== FUNCTION DEFINITIONS ==============

void moveForward() // Move forward
{
  analogWrite(MOTOR_LEFT_FWD_PIN, abs(speedLeft));
  analogWrite(MOTOR_LEFT_BKWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_FWD_PIN, abs(speedRight));
  analogWrite(MOTOR_RIGHT_BKWD_PIN, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(MOTOR_LEFT_FWD_PIN, LOW);
  analogWrite(MOTOR_LEFT_BKWD_PIN, abs(speedLeft));
  analogWrite(MOTOR_RIGHT_FWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(MOTOR_LEFT_FWD_PIN, LOW);
  analogWrite(MOTOR_LEFT_BKWD_PIN, speedLeft);
  analogWrite(MOTOR_RIGHT_FWD_PIN, speedLeft);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(MOTOR_LEFT_FWD_PIN, speedRight);
  analogWrite(MOTOR_LEFT_BKWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_FWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, speedRight);
}

void stopMoving() // Stop movement
{
  analogWrite(MOTOR_LEFT_FWD_PIN, HIGH);
  analogWrite(MOTOR_LEFT_BKWD_PIN, HIGH);
  analogWrite(MOTOR_RIGHT_FWD_PIN, HIGH);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, HIGH);
}

float getDistance(int servoAngle)  // Read the Ultasonic Sensor pointing at the given servo angle
{
  float distance;
  servo.write(servoAngle);
  delay(150);
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}


// N O T E :
// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.