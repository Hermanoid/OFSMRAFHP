#include <Pixy2.h> //Import libraries to help interface with the Pixy and Servos
#include <VarSpeedServo.h>

//Initially define the data pins for the servs and motor controller (which didn't actually end up working)
#define BaseSwivelPin 2
#define ShoulderPin 3
#define ElbowPin 4
#define ForearmRotatePin 5
#define WristSwivelPin 6
#define WristRotatePin 7
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

//Specify the operating range of the servos, so that the program can complain and 
// kick out of the code wants to go where the servos cannot.
#define BaseUpperLimit 180
#define BaseLowerLimit 0
#define ShoulderUpperLimit 130
#define ShoulderLowerLimit 0
#define ElbowUpperLimit 180
#define ElbowLowerLimit 15

//The first two are used by the Law of Cosines and trigonometry included in this code to calculate positions.  
//The last is a default speed.
#define UpperArmLen  120
#define ForeArmLen 145
#define ServoSpeed 80

//Make a place in the program memory for all the servos (3 of these are unused)
VarSpeedServo 
BaseSwivelServo, ShoulderServo, ElbowServo, ForearmRotateServo, WristSwivelServo, WristRotateServo; 

//Initial servo positions, 
//and a place in memory to store the exact angle (0-180 degrees) the servos should target.
byte BaseSwivelPos = 90;
byte ShoulderPos = 90;
byte ElbowPos = 20;
byte ForearmRotatePos = 90;
byte WristSwivelPos = 90;
byte WristRotatePos = 90;

//Some constant  numbers used by the object tracking math, 
//including the range of positions the arm should target in adaptation to the PID loop's demands
#define BASE_MAX_POS 180
#define BASE_MIN_POS 0
#define ARM_Y_MAX_POS 115
#define ARM_Y_MIN_POS 0
#define BASE_CENTER_POS ((BASE_MAX_POS+BASE_MIN_POS)/2)
#define ARM_Y_CENTER_POS ((ARM_Y_MAX_POS+ARM_Y_MIN_POS)/2)

//Confusing stuff.
#define PID_MAX_INTEGRAL         2000
#define DRIVE_BASE_DEADBAND       20

//Ahh... sweet sweet PIDLoop.  I did not create this code, but rather imported it from somebody else's
//This is an "integrated control loop", which means it reads a certain variable 
//(Say, the position of the object, as seen by the camera), and adminisiters a corrective response,
//as determined by a bunch of unpleasant math.
// PID stands for the 3 big constants in a PID loop:  The Proportional, Integral, and Derivitive reponses
// Yes, thanks for noticing that those are 2/3 relating to calculus.
// The magic in "tuning" a PID  loop is getting those three numbers perfectly matched to the application,
// (say, a robot arm) to minimize overshoot and maximize efficiency.
// It also happens to be a sizable pain in the rear.
class PIDLoop
{
public:
  int32_t m_pgain;
  int32_t m_igain;
  int32_t m_dgain;
  PIDLoop(int32_t pgain, int32_t igain, int32_t dgain, bool servo,int32_t min_pos, int32_t max_pos)
  {
    m_pgain = pgain;
    m_igain = igain;
    m_dgain = dgain;
    m_servo = servo;
    m_min_pos = min_pos;
    m_max_pos = max_pos;

    reset();
  }

  void reset()
  {
    if (m_servo)
      m_command = (m_min_pos+m_max_pos)/2;
    else
      m_command = 0;
      
    m_integral = 0;
    m_prevError = 0x80000000L;
  }  
  
  void update(int32_t error)
  {
    int32_t pid;
  
    if (m_prevError!=0x80000000L)
    { 
      // integrate integral
      m_integral += error;
      // bound the integral
      if (m_integral>PID_MAX_INTEGRAL)
        m_integral = PID_MAX_INTEGRAL;
      else if (m_integral<-PID_MAX_INTEGRAL)
        m_integral = -PID_MAX_INTEGRAL;

      // calculate PID term
      pid = (error*m_pgain + ((m_integral*m_igain)>>4) + (error - m_prevError)*m_dgain)>>10;
    
      if (m_servo)
      {
        m_command += pid; // since servo is a position device, we integrate the pid term
        if (m_command>m_max_pos) 
          m_command = m_max_pos; 
        else if (m_command<m_min_pos) 
          m_command = m_min_pos;
      }
      else
      {
        // Deal with Zumo base deadband
        if (pid>0)
          pid += DRIVE_BASE_DEADBAND;
        else if (pid<0)
          pid -= DRIVE_BASE_DEADBAND;
         m_command = pid; // Zumo base is velocity device, use the pid term directly  
      }
    }

    // retain the previous error val so we can calc the derivative
    m_prevError = error; 
  }

  int32_t m_command; 

private: 

  
  int32_t m_prevError;
  int32_t m_integral;
  bool m_servo;
  int32_t m_min_pos;
  int32_t m_max_pos;
};

//End of reused code.

// This is the main Pixy object 
Pixy2 pixy;
//This is where I set those three PID variables to the proper values.
//Please note, these values are not even close to proper.
//They are, however, the result of many an antagonizing hour, and eventually
//  I just had to call it "good enough".
//I probably changed those first three numbers at least hundreds of times.
PIDLoop panLoop(18, 0, 24, true,BASE_MIN_POS,BASE_MAX_POS);
PIDLoop tiltLoop(52, 0, 8, true,ARM_Y_MIN_POS,ARM_Y_MAX_POS);
//These are the two loops I was suppose to add in and tune with the addition of the base.
//PIDLoop rotateLoop(300, 600, 300, false);
//PIDLoop translateLoop(400, 800, 300, false);


//Enter trigonometry!! Everyone loves trig.
//This is the "law of cosines", and while I have no idea why it works,
//I know the internet told me about it, and it gives me what I want, which is good.
//This effectively reverse engineers a triangle, taking in three sides 
// and calculating an angle from it.
float lawOfCosines(float a, float b, float c){
  return acos((a*a+b*b-c*c)/(2*a*b));
}

//Distance formula... you should know this from 7th grade...
float distance(float x, float y){
  return sqrt(x*x+y*y);
}

//This is what actually does the math to move the arm to a position, and is the product of an extra 
//8 or so hours before I even started this 4H project.
//
//facing outward from behind the bot (when the base is angled to its middle, aka 90 degrees):
//baseAngle = Exactly what it sounds like
//y = up/down
//z = forward/back (from base.  That is, 50 millimeters z will result in 50 millimeters 
// in the direction of baseAngle)
bool set_arm_angle(float baseAngle, float y, float z){
  float basePos = baseAngle; 
  //In baseAngle mode, we want arm_x 
  //(that is, x along a plane running vertically through the base and arm, disregarding the 
  // angle between the base and earth) 
  //to be the "z" value, or distance from base.
  float arm_x = z; 
  float arm_y = y; // vertically, the coordinates stay the same

  float arm_dist = distance(arm_x, arm_y);  //Distance to desired point, disregarding base angle 
  //(we're talking vertical planes here)
  if(arm_dist+ForeArmLen<=UpperArmLen||arm_dist+UpperArmLen<=ForeArmLen||ForeArmLen+UpperArmLen<=arm_dist){
    Serial.println("Coordinates out of range (inaccessable due to arm segment lengths)");
    return false;
  }
  float D1 = atan2(arm_y,arm_x); //angle from 0 (parallel to base surface) an imaginary segment 
  // ...running from (0,0,0) to (x,y,z)
  float D2 = lawOfCosines(arm_dist, UpperArmLen, ForeArmLen); //this guy does the math to find the 
  // ...angle from our imaginary segment to the first joint's final angle
  float armAngleOne = (D1+D2)*RAD_TO_DEG; // the arm's final resting place is the combination of the 
  // ...angles between the ground and the imaginary line, and from the imaginary line to the final resting 
  // ...place
  

  float A2 = lawOfCosines(UpperArmLen, ForeArmLen, arm_dist); //more math to figure out the elbow angle;
  //(NOTE:  This particular bot has a different self-leveling elbow setup that needs additional finagling
  float armAngleTwo = 195 - armAngleOne - A2*RAD_TO_DEG; //This comphensates for the self-leveling business.

  if(armAngleOne>ShoulderUpperLimit||armAngleOne<ShoulderLowerLimit
    ||armAngleTwo>ElbowUpperLimit||armAngleTwo<ElbowLowerLimit
    ||basePos>BaseUpperLimit||basePos<BaseLowerLimit){
    Serial.println("Coordinates are out of range (inaccessable due to the servo's limits on range of motion)");
    return false;
  }

  BaseSwivelPos = basePos;
  ShoulderPos = armAngleOne;
  ElbowPos = armAngleTwo;
  //We don't worry about the forearm rotation or wrist servos here.
  return true;
}

//The motor controller code, commented out because 
//  it never did quite work and I don't want it slowing down the Arduino.
//void set_drive(float left, float right){
//  float left_PWM = map(left, -100,100,-255,255);
//  if(left_PWM>0){
//    analogWrite(IN1,left_PWM);
//    digitalWrite(IN2, LOW);
//  }else if(left_PWM<0){
//    digitalWrite(IN1, LOW);
//    analogWrite(IN2, -left_PWM);
//  }else{
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//  }
//  float right_PWM = map(right, -100, 100, -255, 255);
//    if(right_PWM>0){
//    analogWrite(IN3,left_PWM);
//    digitalWrite(IN4, LOW);
//  }else if(left_PWM<0){
//    digitalWrite(IN3, LOW);
//    analogWrite(IN4, -left_PWM);
//  }else{
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//  }
//}

float startPanGain; //see inside setup()
void setup()  //The "setup()" function is called once on startup.
{
  Serial.begin(115200); //Start communicating with the computer, if attached
  Serial.print("Starting...\n");  //Send a greeting.
  BaseSwivelServo.attach(BaseSwivelPin);  //Set up all the servos to be attached to thier respective Pins.
  ShoulderServo.attach(ShoulderPin);
  ElbowServo.attach(ElbowPin);
  ForearmRotateServo.attach(ForearmRotatePin);
  WristSwivelServo.attach(WristSwivelPin);
  WristRotateServo.attach(WristRotatePin);
  pixy.init(); //Start up the Pixy camera.
  set_arm_angle(90,100,150);  //Set the arm to an initial position of front and center.
  updateServos();  //Propogate set_arm_angle()'s changes immediately.
  //This was an attempt to dynamically change the gain variable to make the PID loop
  //less responsive when the ball got closer.  It only ever half-worked.
  startPanGain = panLoop.m_pgain; 
  //Again, the motor controllers are commented out.
//  pinMode(IN1, OUTPUT);
//  pinMode(IN2, OUTPUT);
//  pinMode(IN3, OUTPUT);
//  pinMode(IN4, OUTPUT);
//  set_drive(0,0);
  Serial.println("READY");  //If all the initialization stuff completed successfully, tell the computer about it.
}

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>10)
    return pixy.ccc.blocks[0].m_index;

  return -1;
}

// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block *trackBlock(uint8_t index)
{
  uint8_t i;

  for (i=0; i<pixy.ccc.numBlocks; i++)
  {
    if (index==pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}

void loop()  //The "loop" function is called repeatedly until the Arduino loses power.
{ 
  //some variables to hold math and such.
  static int16_t index = -1; //"static" means this variable does not get deleted every loop.
  int32_t panOffset, tiltOffset,left, right;
  Block *block=NULL;
  
  pixy.ccc.getBlocks(); //tell the Pixy to get all the different "blocks", or objects, it can see.

  if (index==-1) // search....
  {
    Serial.println("Searching for block...");
    index = acquireBlock();
    if (index>=0)
      Serial.println("Found block!");
 }
  // If we've found a block, find it, track it
  if (index>=0)
     block = trackBlock(index);

  if(block){ //if the block exists and can be found
    //Enter more PID.  This below has to due with the corrective internals of the PID loop.
    //
    // calculate pan and tilt errors
    panOffset = (int32_t)pixy.frameWidth/2 - (int32_t)block->m_x;
    tiltOffset =  (int32_t)pixy.frameHeight/2 - (int32_t)block->m_y; 

    //my half-successful attempt o make the "pgain" portion of the PID loop
    //Change dynamically.
    if(block->m_width>pixy.frameWidth/3){
      panLoop.m_pgain = startPanGain/5;
    }else{
      panLoop.m_pgain = startPanGain;
    }
    Serial.println(panLoop.m_pgain);
    
    // calculate how to move pan and tilt servos
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    //Set the arm to the positions provided now by the now-fully-mathed PID loop
    set_arm_angle(panLoop.m_command, tiltLoop.m_command,150);
    updateServos();  //make the servos go where desired immediately.
    //Print off some handy information about what's going on in the code to the computer.
    Serial.println("tiltOffset:  "+String(tiltOffset)+"  tiltLoop:  "+String(tiltLoop.m_command));

    block->print(); //Relay some information about the object, like location and size.
  }else{
    index = -1; //If the block cannot be found, set the blocks index to the special
    //... value of -1, which tells the code it cannot be found in the next loop.
  }
  
}

void updateServos(){
  //Set the servos to the positions in the position variables
  // (as are changed by set_arm_angle)
  // at the speed specified earlier.
  BaseSwivelServo.write(BaseSwivelPos,ServoSpeed);
  ShoulderServo.write(ShoulderPos,ServoSpeed);
  ElbowServo.write(ElbowPos,ServoSpeed);
  ForearmRotateServo.write(ForearmRotatePos,ServoSpeed);
  WristSwivelServo.write(WristSwivelPos,ServoSpeed);
  WristRotateServo.write(WristRotatePos,ServoSpeed);
}
