/* 
 *  0: right foot
 *  1: right ankle
 *  2: right knee
 *  3: right thigh
 *  4: right hip
 *  5: right rotate
 *  
 *  16: left foot
 *  17: left ankle
 *  18: left knee
 *  19: left thigh
 *  20: left hip
 *  21: left rotate
 */
void setup() {
  Serial.begin(9600);
  delay(5000);
}

struct servoData{
  int servo;
  int angle;
  int time;
};
static servoData data0 = {0, 1200, 100};
static servoData data1 = {1, 1500, 100};
static servoData data2 = {2, 2000, 100};
static servoData data3 = {3, 1830, 100};
static servoData data4 = {4, 1590, 100};
static servoData data5 = {5, 1500, 100};
static servoData data6 = {16, 1450, 100};
static servoData data7 = {17, 1500, 100};
static servoData data8 = {18, 1000, 100};
static servoData data9 = {19, 1500, 100};
static servoData data10 = {20, 1550, 100};
static servoData data11 = {21, 1500, 100};
static servoData neutralData[] = {data0, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11};
bool hasIntialized = false;

//the most basic move function for the robot. One takes each individual argument and one takes the servoData struct 
void moveServo(int servo, int angle, int time) {
  Serial.print("#");
  Serial.print(servo);
  Serial.print("P");
  Serial.print(angle);
  Serial.print("T");
  Serial.println(time);
}
void moveServo(struct servoData data){
  Serial.print("#");
  Serial.print(data.servo);
  Serial.print("P");
  Serial.print(data.angle);
  Serial.print("T");
  Serial.println(data.time);
}

//The next highest level of movement, this lets you move multiple servos at once
void groupServoMove(int num, struct servoData groupData[]){
  for(int i = 0; i < num; i++){
    moveServo(groupData[i]);
  }
}

//takes a 1-12 servo index and converts to the servo controllers 0-5 and 16-21
int physicalIndex(int servo){
  switch(servo){
    case 1:
      return 0;
    case 2:
      return 1;
    case 3:
      return 2;
    case 4:
      return 3;
    case 5:
      return 4;
    case 6:
      return 5;
    case 7:
      return 16;
    case 8:
      return 17;
    case 9:
      return 18;
    case 10:
      return 19;
    case 11:
      return 20;
    case 12:
      return 21;
    default:
      return 69; //this is the error value (normally would mean it does nothing)
  }
}
//gets the 'logical index' or the 1-12 indexing of the servo from the physical index
int logicalIndex(int servo){
  switch(servo){
    case 0:
      return 1;
    case 1:
      return 2;
    case 2:
      return 3;
    case 3:
      return 4;
    case 4:
      return 5;
    case 5:
      return 6;
    case 16:
      return 7;
    case 17:
      return 8;
    case 18:
      return 9;
    case 19:
      return 10;
    case 20:
      return 11;
    case 21:
      return 12;
    default:
      return 69; //this is the error value (normally would mean it does nothing)
  }
}

//returns a servo to neutral position
//uses both index types but with a boolean to tell if its logical or physical
void returnToNeutral(int servo, bool isLogicalServo){
  if(isLogicalServo){
    moveServo(physicalIndex(servo), neutralData[servo].angle, 500);
  } else {
    moveServo(servo, neutralData[logicalIndex(servo)].angle, 500);
  }
}

//relativeMove is a higher level servoMove that moves the servo relative to its neutral position
//negative for left relative movement and positive for right relative movement
//uses a 1-12 servo indexing with 1-6 being the left leg, and 7-12 being right leg
void relativeServoMove(int servo, int relativeAngle, int time){
  moveServo(physicalIndex(servo), neutralData[servo-1].angle + relativeAngle, time);
}
//dafult time value of 500 milliseconds
void relativeServoMove(int servo, int relativeAngle){
  moveServo(physicalIndex(servo), neutralData[servo-1].angle + relativeAngle, 500);
}
//takes a servoData where the angle in the servoData is the relativeAngle to be moved
void relativeServoMove(struct servoData data){
 moveServo(physicalIndex(data.servo), neutralData[data.servo-1].angle + data.angle, data.time);
}

//TESTING AND HARDCODED MOVEMENTS
//has built in delays
void sideSway(int magnitudeOfSway){
  relativeServoMove(5, magnitudeOfSway);
  relativeServoMove(11, magnitudeOfSway);
  relativeServoMove(1, magnitudeOfSway);
  relativeServoMove(7, magnitudeOfSway);
  delay(1000);
  relativeServoMove(5, -magnitudeOfSway);
  relativeServoMove(11, -magnitudeOfSway);
  relativeServoMove(1, -magnitudeOfSway);
  relativeServoMove(7, -magnitudeOfSway);
  delay(1000);
}

void moveCenterOfGravity(bool isLeftSide){
  if(!isLeftSide){
    relativeServoMove(5, 190);
    relativeServoMove(11, 190);
    relativeServoMove(1, 190);
    relativeServoMove(7, 190);
  } else {
    relativeServoMove(5, -190);
    relativeServoMove(11, -190);
    relativeServoMove(1, -190);
    relativeServoMove(7, -190);
  }
  delay(1000);
}

void raiseFoot(bool isLeftSide, int magnitude){
  if(isLeftSide){
    relativeServoMove(8, -magnitude);
    relativeServoMove(9, -magnitude);
  } else {
    relativeServoMove(2, magnitude);
    relativeServoMove(3, magnitude);
  }
  delay(1000);
}

void returnFoot(bool isLeftSide){
  if(isLeftSide){
    relativeServoMove(8, 0);
    relativeServoMove(9, 0);
  } else {
    relativeServoMove(2, 0);
    relativeServoMove(3, 0);
  }
  delay(1000);
}

void moveLegForward(bool isLeftSide, int magnitude){
  if(!isLeftSide){
    relativeServoMove(4, -magnitude);
    relativeServoMove(2, 1.2*magnitude);
  } else {
    relativeServoMove(8, -magnitude);
    relativeServoMove(10, 1.2*magnitude);
  }
  delay(1000);
}

void returnLeg(bool isLeftSide){
  if(!isLeftSide){
    relativeServoMove(4, 0);
    relativeServoMove(2, 0);
  } else {
    relativeServoMove(8, 0);
    relativeServoMove(10, 0);
  }
  delay(1000);
}

void loop(){
  if(!hasIntialized){
    groupServoMove(12, neutralData);
    hasIntialized = true;
    delay(2000);
  }


  
  moveCenterOfGravity(true);
  raiseFoot(false, 300);
 moveLegForward(false, 300);
  moveLegForward(true, -200);
  returnFoot(false);
 returnLeg(true);
  returnLeg(false);
//
  moveCenterOfGravity(false);
  raiseFoot(true, 300);
 moveLegForward(true, 300);
  moveLegForward(false, -200);
  returnFoot(true);
  returnLeg(false);
  returnLeg(true);
}
