#include <Arduino.h>

// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

#include <Servo.h>

#define LEFT_FOR 9    // PWMB
#define LEFT_BACK 5   // DIRB  ---  Left
#define RIGHT_FOR 6   // PWMA
#define RIGHT_BACK 10 // DIRA  ---  Right

const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;

// Robot parameters:
// Robot length measured on the robot is 25.0 cm.
// Robot width measured on the robot is  16.7 cm.

// Maze parameters:
// In order for the robot to be able to safely make an U turn,
// we will choose the maze width to be 3 times the robot width,
// which is equal to 50.1, we will approximate this value to 50 cm.
const int MazeCorridorWidth = 50;

// Tresholds:
const float FrontDistanceTreshold = MazeCorridorWidth / 2;
const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

const float CenterLineTolerance = 2.5;  // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                        // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 15.0;   // Measured by experiments with the robot
const int WallFollowingSide = -90;      //Set: -90 for right wall following or +90 for left wall following
                                        //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                        // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int FrontServoDelay = 150;
const int SideServoDelay = 150;

const int LeftSpeed = 100; //да се подбере оптималната скорост на левия двигател
const int RightSpeed = 100; //да се подбере оптималната скорост на десния двигател

float maxDistance = 130.0;
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
bool directionCompensation = false;

Servo myservo;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();
float getDistance(int servoAngle, int delayAfterServoMovement); //read the Ultasonic Sensor pointing at the given servo angle

//-----------------------------------------------

void setup()
{
  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(9600);
  myservo.attach(ServoPin);
  myservo.write(90); //Move the servo to center position

  moveForward();
  delay(500);
}

//---------------------------------------------------------

void loop()
{

  float frontDistance, sideDistance;

  int currentState = 0;
  sideDistance = getDistance(SideServoAngle, SideServoDelay);
  frontDistance = getDistance(FrontServoAngle, FrontServoDelay);

  if (frontDistance <= 20.0) //Стената отпред е близко
  {
    digitalWrite(LedPin, HIGH);
    currentState = 1;
  }
  if (frontDistance >= 20.0) //Стената отпред е далече
  {
    if (sideDistance >= 50.0) //Стената отдясно е далече
    {
      currentState = 2;
    }
    else if (sideDistance < 35.0 && sideDistance >= 29.0)     //    |_________|__ROBOT__|_________|_________|_________|     The robot is to the left from the centerline treshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 3;
    }
    else if (sideDistance > 15.0 && sideDistance <= 21.0)     //    |_________|_________|_________|__ROBOT__|_________|          The robot is to the right from the centerline treshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 4;
    }
    else if (sideDistance <= 15.0)                            //    |_________|_________|_________|_________|__ROBOT__|           The robot is on the far right os the corridor
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 5;
    }
    else if (sideDistance >= 35.0 && sideDistance < 50.0)     //    |__ROBOT__|_________|_________|_________|_________|     The robot is on the far left os the corridor
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 6;
    }
    else if (sideDistance >= 21.0 && sideDistance < 29.0)     //    |_________|_________|__ROBOT__|_________|_________|   The robot is close to the center line
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 7;
    } 
  }   
  switch (currentState)
  {
  case 1: // Turn 90 degrees left
    moveBackward();
    delay(100);
    speedLeft = LeftSpeed * 1.35;
    turnLeft();
    delay(700);  //завой на ляво на 90 градуза (задава се продължителността на завоя)
     speedLeft = LeftSpeed;
     speedRight = RightSpeed;
    moveBackward();
    delay(100);
    break;
  case 2: // Turn 90 degrees right
      speedLeft = LeftSpeed;
     speedRight = RightSpeed;
     moveForward();
     delay(600);    // придвижване 20 см напред (задава се продължителността на придвижването)
     speedRight = RightSpeed * 1.35;
     turnRight();
     delay(700);  //завой на дясно на 90 градуза (задава се продължителността на завоя)
     speedLeft = LeftSpeed;
     speedRight = RightSpeed;
     moveForward();
     delay(750);   // придвижване 25 см напред (задава се продължителността на придвижването)
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

//==================================== FUNCTIONS =====================================================

void moveForward() // Move forward
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedLeft);
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void stopMoving() // Stop movement
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float getDistance(int servoAngle, int delayAfterServoMovement)
{
  float distance;
  myservo.write(servoAngle);
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
