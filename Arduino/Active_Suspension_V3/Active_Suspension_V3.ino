/*
  Summary:
  
  Written by: Rachel Du
  Aug 2019
  ------------------------
  Edited by: Lucas Dondertman
  Fall 2019 (Sept - Dec)
*/

///////////////////////////////////////

//SETUP!!!!!!!!!

// Accelerometer library
#include <MPU9250.h>   

// Encoder library
#include <Encoder.h> 

// Initialize all pins
// Color-coded list of variables: 
// https://docs.google.com/document/d/1rfLVPlw9mu2PoYqc_Dgf6W_8biOQRDDQttkra0Mk32U/edit?usp=sharing
int roadEncA = 17;
int roadEncB = 19;  
int wheelEncA = 3;
int wheelEncB = 5;
int carEncA = 2;
int carEncB = 4;
int roadDir = 7;
int susDir = 8;
int roadPWM = 9;
int susPWM = 10;
int accelTopAD0 = 13;

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 accelTop(Wire, 0x68);
//MPU9250 accel(Wire, 0x68);
int status;

// Initialize encoder objects with numbers of pins connected
Encoder carEnc(carEncA, carEncB);               //top plate encoder
Encoder wheelEnc(wheelEncA, wheelEncB);         //road plate encoder
Encoder roadEnc(roadEncA, roadEncB);            //road plate encoder

// Counts number of times looped, used in average() function
int loopCounter = 0;

// Bool to control speed of road motor, initialized to false
bool speed1Active = false;

// Initialize variables used in findPlateVelocities();
double lastCarPos = 0.0, lastWheelPos = 0.0, lastRoadPos = 0.0, lastTime;

// PID initializations
double prevError = 0.0, integral = 0.0, setpoint = 0.0;
double clamp = 1;
bool saturated, integWind;
  
////////////////////////////////////////////

// FUNCTION DECLARATIONS!!!!!!
// All functions are below loop(), with explanations of their purpose and how they work

void initializeAccelerometers();

double encCountsToPosition(long counts, double cpr, double diameter);

double findPlateVelocity(double lastPos, double curPos, double duration);

void contractSus(int PWM);

void expandSus(int PWM);

void resetSusMotor();

void resetRoadMotor(double& setpoint);

void serialHandshake();

void serialHandshakeMotor();

void pidControl(double measuredPos, double duration);

/////////////////////////////////////////

void setup() {
  
  // If using serial plotter/monitor ensure that baud rate is the same as this one
  Serial.begin(115200);
  
  // Initial connection with matlab
  serialHandshake();
  
  // Initialize and calibrate both accelerometers
  initializeAccelerometers();
  
  // Reset susoension motor
  resetSusMotor();

  // Reset road motor (and whole system) to 'home' position
  resetRoadMotor(setpoint);

  // MATLAB wait for signal motor is reset before starting data aquisition
  serialHandshakeMotor();

  // Starting time of loop
  lastTime = millis();
}

////////////////////////////////////////

void loop() {
  
  // Increment loop counter
  loopCounter++;

  // Save start time
  double startTime = millis();
  
  // Find amount of time that has passed since
  double loopDuration = startTime - lastTime;

  // Read accelerometer
  accelTop.readSensor();                           

  // Read all encoders for counts
  long carPosInCounts = carEnc.read();
  long wheelPosInCounts = wheelEnc.read();
  long roadPosInCounts = roadEnc.read();

  // Convert counts measured into actual positions
  double carPosActual = encCountsToPosition(carPosInCounts, 4048.0, 6.35);  
  double wheelPosActual = encCountsToPosition(wheelPosInCounts, 4048.0, 6.35);
  double roadPosActual = encCountsToPosition(roadPosInCounts, 4048.0, 6.35);

  // Find velocities of plates with their position readings over a time interval
  double carPlateVelocity = findPlateVelocity(lastCarPos, carPosActual, loopDuration);
  double wheelPlateVelocity = findPlateVelocity(lastWheelPos, wheelPosActual, loopDuration);
  double roadPlateVelocity = findPlateVelocity(lastRoadPos, roadPosActual, loopDuration);

  // For controlling suspension motor using PID
  pidControl(carPosActual, loopDuration);
  
  // Set current values to past values, to be used in proceding loop
  lastCarPos = carPosActual;
  lastWheelPos = wheelPosActual;
  lastRoadPos = roadPosActual;
  lastTime = startTime;


  // Send serial values to MATLab to be plotted
  Serial.println(millis());                     //print time
  Serial.println(carPosActual);                 //print car position and offset on  graph
  Serial.println(wheelPosActual);               //print wheel position and offset on  graph
  Serial.println(roadPosActual);                //print road position
  Serial.println(accelTop.getAccelX_mss(), 6);  //print x-accel
  Serial.println(accelTop.getAccelY_mss(), 6);  //print y-accel
  Serial.println(accelTop.getAccelZ_mss(), 6);  //print z-accel
  Serial.println(carPlateVelocity);             //print car plate velocity
  Serial.println(wheelPlateVelocity);           //print wheel plate velocity
  Serial.println(roadPlateVelocity);            //print road plate velocity

}

///////////////////////////////////////////////////

// FUNCTIONS!!!!!!!!!

// Initialize and calibrate both accelerometers
void initializeAccelerometers()
{
  // Send HIGH to ADO port on top accelerometer to change MPU9250 adress to 0x69
  pinMode(accelTopAD0, OUTPUT);
  digitalWrite(accelTopAD0, LOW);
  
  // Start commsunication with accelerometer
  status = accelTop.begin();
  //status = accel.begin();

  // Set up accelerometers scale factors and bias'
  float axbTop = accelTop.getAccelBiasX_mss();
  float axsTop = accelTop.getAccelScaleFactorX();
  float aybTop = accelTop.getAccelBiasY_mss();
  float aysTop = accelTop.getAccelScaleFactorY();
  float azbTop = accelTop.getAccelBiasZ_mss();
  float azsTop = accelTop.getAccelScaleFactorZ();
  /*
  float axb = accel.getAccelBiasX_mss();
  float axs = accel.getAccelScaleFactorX();
  float ayb = accel.getAccelBiasY_mss();
  float ays = accel.getAccelScaleFactorY();
  float azb = accel.getAccelBiasZ_mss();
  float azs = accel.getAccelScaleFactorZ();
  */

  // Calibrate accelerometers
  accelTop.setAccelCalX(axbTop, axsTop);
  accelTop.setAccelCalY(aybTop, aysTop);
  accelTop.setAccelCalZ(azbTop, azsTop);
 /*
  accel.setAccelCalX(axb, axs);
  accel.setAccelCalY(ayb, ays);
  accel.setAccelCalZ(azb, azs);
  */
  
  // Set accelerometer ranges
  accelTop.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  accelTop.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  accelTop.setSrd(19); 
  /*
  accel.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  accel.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  accel.setSrd(19); 
  */
}

// Function that converts position from number of counts to height (mm); using counts per 
// revolution, diameter of encoder shaft (in mm), cpr=counts per rev
double encCountsToPosition(long counts, double cpr, double diameter)           
{
  double pos = counts;
  pos /= cpr;                 // = number of revolutions
  pos *= PI * diameter;       // each revolution = height change of 1 circumference
  return pos;
}

// Function to find velocity of car and wheel plates (m/s)
// Velocity in m/s with direction (+ up/- down)
double findPlateVelocity(double lastPos, double curPos, double duration)
{
  double velocity = curPos - lastPos;   //find dist travelled in mm
  velocity /= duration;                 //get velocity in mm/ms (which = m/s)
  return velocity;
}

void contractSus(int PWM) 
{
  digitalWrite(susDir, HIGH);
  analogWrite(susPWM, PWM);
}

void expandSus(int PWM)
{
  digitalWrite(susDir, LOW);
  analogWrite(susPWM, PWM);
}

void resetSusMotor() 
{
  int smallDelay = 10;
  double firstPos = 0, secondPos = 0;
  double tol = 0.03;

  expandSus(50);
  delay(smallDelay);

  // Wait while motor not running/ power off
  while (tol >= abs(firstPos - secondPos)) {
    firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    delay(smallDelay);
    secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  }

  // While plate going up (because expanding)
  while (secondPos > firstPos)
  {
    firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    delay(smallDelay);
    secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  }
  // While in equilibrium
  while (tol >= abs(firstPos - secondPos)) 
  {
    firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    delay(smallDelay);
    secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  }
  double posInit = secondPos;
  
  // As soon as expandSus starts to contract, switch motor direction to contract
  contractSus(80);
  while (12 > posInit - encCountsToPosition(carEnc.read(), 4048.0, 6.35)) {}
  contractSus(0);
  // Upon exiting loop, plate should be in starting condition
}

// Function to reset road motor to starting position and find setpoint
void resetRoadMotor(double& setpoint)
{
  int smallDelay = 10;
  double firstPos = 0, secondPos = 0;
  double tol = 0.03;
  double crest = 0, trough = 0;
    
  // Set direction for road motor and initial direction for suspension motor
  digitalWrite(roadDir, HIGH);
  analogWrite(roadPWM, 95);

  // Wait while road motor is not running/ power is off
  while (tol >= abs(firstPos - secondPos)) {
    firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    delay(smallDelay);
    secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  }
  crest = secondPos, trough = secondPos;


  // If top plate going up
  if (secondPos > firstPos)  
  {
    // Then wait for crest by comparing both positions
    // Set crest to highest position
    while (secondPos > firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      if (secondPos > crest)
      {
        crest = secondPos;
      }
      else if (secondPos < trough)
      {
        trough = secondPos;
      }
    }
    
    // Then wait for trough by comparing both positions
    // Set trough to lowest position
    while (firstPos >= secondPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      if (secondPos > crest)
      {
        crest = secondPos;
      }
      else if (secondPos < trough)
      {
        trough = secondPos;
      }
    }
    // Road motor back at bottom so were good
  }
  // Else going down already
  else 
  {
    // Then wait for trough by comparing both positions
    // Set trough to lowest position
    while (secondPos < firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      if (secondPos > crest)
      {
        crest = secondPos;
      }
      else if (secondPos < trough)
      {
        trough = secondPos;
      }
    }
    // Motor back at bottom so were good
  }
  
  for (int startingRevs = 0; startingRevs < 4; startingRevs++)
  {
    // Then wait for crest by comparing both positions
    // Set crest to highest position
    while (secondPos >= firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      if (secondPos > crest)
      {
        crest = secondPos;
      }
      else if (secondPos < trough)
      {
        trough = secondPos;
      }
    }
    
    // Then wait for trough by comparing both positions
    // Set trough to lowest position
    while (secondPos <= firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      if (secondPos > crest)
      {
        crest = secondPos;
      }
      else if (secondPos < trough)
      {
        trough = secondPos;
      }
    }
  }
  // Set setpoint to middle of graph
  setpoint = (crest+trough)/2;
}

// Function that interfaces with matlab to set up serial connection
void serialHandshake()
{
  // Handshake with matlab to start serial connection
  Serial.println('a');
  char a = 'b';
  while (a != 'a')
  {
    a = Serial.read();
  }
}

void serialHandshakeMotor()
{
    // Let MATLAB know motor is reset
  Serial.println('c');
  char c = 'd';
  while (c != 'c')
  {
    c = Serial.read();
  }
}

// Function to get PID adjustment output for sus motor and to control motor based on this
// measuredPos (mm), duration (ms), 
void pidControl(double measuredPos, double duration)
{
  int controlPWM = 0;
  double Kp = 18.0;
  double Ki = 0.0;
  double Kd = -1.7;
  
  // First few lines to get PID output, control kp, ki, kd for response tuning
  double error = setpoint - measuredPos; // mm
  integral += error * duration;   // mm*ms = m*s (absement)
  double derivative = (error - prevError) / duration; // mm/ms = m/s (velocity)
  double output = Kp * error + Ki * clamp * integral + Kd * derivative; // what to do with the output?!
  prevError = error;

  // Convert output to valid PWM integer + check for saturation
  if (output > 255 || output < -255)
  {
    controlPWM = 255;
    saturated = true;
  }
  else
  {
    controlPWM = abs(output);
  }

  // Figure out which direction motor should spin
  // Depending on if error is + or -, set direction of motor!
  if (error > 0)
  
  {
    expandSus(controlPWM);
  }
  else {
    
    contractSus(controlPWM);
  }
}
