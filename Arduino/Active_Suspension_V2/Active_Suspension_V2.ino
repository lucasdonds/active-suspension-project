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
MPU9250 accel(Wire, 0x68);
MPU9250 accelTop(Wire, 0x69);
int status;

// Initialize encoder objects with numbers of pins connected
Encoder carEnc(carEncA, carEncB);               //top plate encoder
Encoder wheelEnc(wheelEncA, wheelEncB);         //road plate encoder
Encoder roadEnc(roadEncA, roadEncB);            //road plate encoder

// Counts number of times looped, used in average() function
int loopCounter = 0;

// Bool to control speed of road motor, initialized to false
bool speed1Active = false;

// Variable to store loopDuration, used in many fucntions
double loopDuration;

/*
// Initialize array that stores previous speed values so average can be calculated 
// Used in fuction: averageSpeed()
double speedStorage[20] = { 0 };
*/

// Initialize variables used in findPlateVelocities();
double lastCarPos = 0.0, lastWheelPos = 0.0, lastRoadPos = 0.0, lastTime;

// PID initializations
double prevError = 0.0, integral = 0.0, setpoint = 0.0;
int controlPWM = 0;
double Kp = 9.0;
double Ki = 0.0;
double Kd = 2;

////////////////////////////////////////////

// FUNCTION DECLARATIONS!!!!!!
// All functions are below loop(), with explanations of their purpose and how they work

void initializeAccelerometers();

double encCountsToPosition(long counts, double cpr, double diameter);

//double findMotorRPM(long counts, double cpr, double duration);

//double average(double stored[], int arraySize, int numLoops);

double findPlateVelocity(double lastPos, double curPos, double duration);

//int toggleRoadMotorSpeed(int speed1, int speed2);

void contractSus(int PWM);

void expandSus(int PWM);

void resetSusMotor();

void resetRoadMotor();

void serialHandshake();

//void setupTimer1(double desiredHz1);

//void setupTimer3(double desiredHz3);

void pidControl(double measuredPos, double duration);

////////////////////////////////////////

// INTERRUPTS!!!!!!!!!!!!!!
// Interrupts get set up in setup()
// Disable interrupts by commenting out whats inside the interrupt, but keep the interrupt

/*
// Interrupt for toggling speed of road motor
ISR(TIMER3_COMPA_vect) {  //change the 1 to 0 for timer0 or 2 for timer2
   toggleRoadMotorSpeed(200, 200);
}

// Interrupt calling function to toggle motor direction of sus motor
ISR(TIMER1_COMPA_vect) {  //change the 1 to 0 for timer0 or 2 for timer2

}
*/
/////////////////////////////////////////

void setup() {
  
  // If using serial plotter/monitor ensure that baud rate is the same as this one
  Serial.begin(115200);

  // Initial connection with matlab
  serialHandshake();

  // Initialize and calibrate both accelerometers
  initializeAccelerometers();

  // Reset susoension motor
  //resetSusMotor();

  // Set direction for road motor and initial direction for suspension motor
  //pinMode(roadDirA, OUTPUT);
  digitalWrite(roadDir, HIGH);
  analogWrite(roadPWM, 90);

  // Reset road motor (and whole system) to 'home' position
  resetRoadMotor();

  // Starting time of loop
  lastTime = millis();

  // Pin mode for suspension motor
  //pinMode(susDirA, OUTPUT);

  // Initialize all timer interrupts
  //cli();          //stops inturrupts
  //setupTimer1(2);
  //setupTimer3(0.25);
  //sei();          //allow interrupts

}

////////////////////////////////////////

void loop() {

  //digitalWrite(susDirA, LOW);
  //analogWrite(susPWM, 200);
  // Increment loop counter
  loopCounter++;

  // Save start time
  double startTime = millis();
  
  // Find amount of time that has passed since
  double loopDuration = startTime - lastTime;

  // Read accelerometer
  accel.readSensor();
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
  Serial.println(accel.getAccelX_mss(), 4);     //print x-accel
  Serial.println(accel.getAccelY_mss(), 4);     //print y-accel
  Serial.println(accel.getAccelZ_mss(), 4);     //print z-accel
  Serial.println(accelTop.getAccelX_mss(), 4);  //print x-accel
  Serial.println(accelTop.getAccelY_mss(), 4);  //print y-accel
  Serial.println(accelTop.getAccelZ_mss(), 4);  //print z-accel
  Serial.println(carPlateVelocity);             //print car plate velocity
  Serial.println(wheelPlateVelocity);           //print wheel plate velocity
  Serial.println(roadPlateVelocity);            //print road plate velocity


/*
  // Speed returned using motor enc counts for the past loop
  double motorSpeed = findMotorRPM(motorPosInCounts, 3415.92, loopDuration);
 
  // Shift everything in speed storage array one index over and store new variable
  for (int index = 0; index < 19; index++)                                
  {
    speedStorage[index] = speedStorage[index + 1];
  }
  speedStorage[0] = motorSpeed;

  // Average of the last 20 motor speeds
  double averageSpeed = average(speedStorage, 20, loopCounter);
*/

  //Serial.print("controlPWM: "); Serial.println(controlPWM);
  //Serial.print("plateDirUp?: "); Serial.println(plateDirUp);
  
 /*
  // For plotting counts from encoders on serial plotter
  Serial.print("car: "); Serial.print(-carEnc.read()); Serial.print("  ");           
  Serial.print("wheel: "); Serial.print(wheelEnc.read()); Serial.print("  ");
  Serial.print("road: "); Serial.print(roadEnc.read()); Serial.print("  ");
  Serial.println("uT");
  
  // For plotting acceleration in each axis on serial plotter
  Serial.print("x: "); Serial.print(accel.getAccelX_mss(),6); Serial.print("  ");   
  Serial.print("y: "); Serial.print(accel.getAccelY_mss(),6); Serial.print("  ");
  Serial.print("z: "); Serial.print(accel.getAccelZ_mss(),6); Serial.print("  ");
  Serial.println("uT");
  
  // For plotting motor speed and average speed on serial plotter
  Serial.print("speed: "); Serial.print(motorSpeed); Serial.print("  ");              
  Serial.print("average: "); Serial.print(averageSpeed); Serial.print("  ");
  Serial.println("uT");
 
  // For plotting positions from encoders on serial plotter
  Serial.print("car: "); Serial.print(carPosActual); Serial.print("  "); 
  Serial.print("wheel: "); Serial.print(wheelPosActual); Serial.print("  ");     
  Serial.print("road: "); Serial.print(roadPosActual); Serial.print("  ");
  Serial.print("Setpoint: "); Serial.print(setpoint); Serial.print("  ");
  Serial.println("uT"); 
/*
  // For plotting car and wheel plate velocities
  Serial.print("car plate velocity: "); Serial.println(carPlateVelocity);
  //Serial.print("wheel plate velocity: "); Serial.println(wheelPlateVelocity);
  Serial.println("uT");
*/

}

///////////////////////////////////////////////////

// FUNCTIONS!!!!!!!!!

// Initialize and calibrate both accelerometers
void initializeAccelerometers()
{
  // Send HIGH to ADO port on top accelerometer to change MPU9250 adress to 0x69
  digitalWrite(accelTopAD0, HIGH);
  
  // Start commsunication with accelerometer
  status = accel.begin();
  status = accelTop.begin();

  // Set up accelerometers scale factors and bias'
  float axb = accel.getAccelBiasX_mss();
  float axs = accel.getAccelScaleFactorX();
  float ayb = accel.getAccelBiasY_mss();
  float ays = accel.getAccelScaleFactorY();
  float azb = accel.getAccelBiasZ_mss();
  float azs = accel.getAccelScaleFactorZ();
  float axbTop = accelTop.getAccelBiasX_mss();
  float axsTop = accelTop.getAccelScaleFactorX();
  float aybTop = accelTop.getAccelBiasY_mss();
  float aysTop = accelTop.getAccelScaleFactorY();
  float azbTop = accelTop.getAccelBiasZ_mss();
  float azsTop = accelTop.getAccelScaleFactorZ();

  // Calibrate accelerometers
  accel.setAccelCalX(axb, axs);
  accel.setAccelCalY(ayb, ays);
  accel.setAccelCalZ(azb, azs);
  accelTop.setAccelCalX(axbTop, axsTop);
  accelTop.setAccelCalY(aybTop, aysTop);
  accelTop.setAccelCalZ(azbTop, azsTop);

  // Set accelerometer ranges
  accel.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  accel.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  accel.setSrd(19);
  accelTop.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  accelTop.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  accelTop.setSrd(19);
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
  double tol = 0.01;

  expandSus(50);
  delay(smallDelay);

  // Wait while motor not running/ power off
  while (tol >= abs(firstPos - secondPos)) {
    firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    delay(smallDelay);
    secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  }

  // If top plate going up
  if (secondPos > firstPos)
  {
    while (secondPos > firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    }
    // Upon exiting loop, plate should be in starting condition
  }
  // Else top plate going down
  // Note if this happened, then the contract and expand got mixed up somehow and
  // we must wait until the contract actually contracts the system
  else 
  {
    // Switch direction so its going up
    contractSus(100);
    delay(smallDelay);

    // Wait until stops going up
    while (secondPos > firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    }
    // Wait while plate is steady
    while (tol >= abs(firstPos - secondPos)) {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    }
    // Upon exiting this loop, plate should be in starting condition
   
  // Turn motor off, top plate should be in starting condition
  }
  analogWrite(susPWM, 0);
}

// Function to reset road motor to starting position and find setpoint
void resetRoadMotor()
{
  int smallDelay = 20;
  double firstPos = 0, secondPos = 0;
  double tol = 0.03;
  //double crest = 0, trough = 0;

  // Wait while road motor is not running/ power is off
  while (tol >= abs(firstPos - secondPos)) {
    firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
    delay(smallDelay);
    secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  }
  double crest = secondPos, trough = secondPos;
  
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
    // Motor back at bottom so were good
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

// Function to get PID adjustment output for sus motor and to control motor based on this
// measuredPos (mm), duration (ms), 
void pidControl(double measuredPos, double duration)
{
  // First few lines to get PID output, kp, ki, kd controlled globally
  double error = setpoint - measuredPos; // mm
  integral += error * duration;   // mm*ms = m*s (absement)
  double derivative = (error - prevError) / duration; // mm/ms = m/s (velocity)
  double output = Kp * error + Ki * integral + Kd * derivative; // what to do with the output?!
  prevError = error;

  // Convert output to valid PWM integer
  output = abs(floor(output));
  if (output > 255)
  {
    controlPWM = 255;
  }
  else if (output < 0)
  {
    controlPWM = 0;
  }
  else {
    controlPWM = output;
  }

  // For testing!
  //Serial.println(controlPWM);

  // Figure out which direction motor should spin
  // Depending on if error is + or -, set direction of motor!
  if (error > 0)
  {
    contractSus(controlPWM);
  }
  else {
    
    expandSus(controlPWM);
  }
}

/*
// Function that finds the speed (RPM) of the motor using the change in position from 
// the motor encoder duration in milliseconds
double findMotorRPM(long counts, double cpr, double duration)           
{
  double rpm = fabs(counts);
  rpm = rpm/duration;
  rpm = rpm/cpr;                 //counts per revolution of the motor
  rpm = rpm*60000;               //number of milliseconds in one minute
  return rpm;
}

// Function that finds the average of all array values parameters: array, size of array
double average(double stored[], int arraySize, int numLoops)        
{
  double ave = 0;
  if (numLoops < arraySize)
  {
    for (int index = 0; index < numLoops; index++)
      ave += stored[index];
    ave /= numLoops;
  }
  else {
    for (int index = 0; index < arraySize; index++)
      ave += stored[index];
    ave /= arraySize;
  }
  return ave;
}

// Function use din interrupt to toggle road Motor speed
int toggleRoadMotorSpeed(int speed1, int speed2) 
{
  if (!speed1Active) {
    analogWrite(roadPWM, speed1);
    speed1Active = true;
  }
  else {
    analogWrite(roadPWM, speed2);
    speed1Active = false;
  }
}

// Fuinction to setup interrupt, give it desired Hz at which interrupt is to be called
void setupTimer1(double desiredHz1) 
{
  // Set up interrupt for changing the sus motor direction
  // User should change desiredHz to what they desire
  double CMR1 = (16*pow(10,6)) / (desiredHz1*1024) - 1;
  TCCR1A = 0;     // Set entire TCCR1A register to 0
  TCCR1B = 0;     // Same for TCCR1B
  TCNT1  = 0;     // Initialize counter value to 0
  // Set compare match register for desiredHz increments
  OCR1A = CMR1;    // Must be <65536
  // Turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

// Fuinction to setup interrupt, give it desired Hz at which interrupt is to be called
void setupTimer3(double desiredHz3)
{
  // Set up interrupt for changing the road motor speed
  // User should change desiredHz to what they desire
  double CMR3 = (16*pow(10,6)) / (desiredHz3*1024) - 1;
  TCCR3A = 0;     // Set entire TCCR0A register to 0
  TCCR3B = 0;     // Same for TCCR0B
  TCNT3  = 0;     // Initialize counter value to 0
  // Set compare match register for desiredHz increments
  OCR3A = CMR3;
  // Turn on CTC mode
  TCCR3B |= (1 << WGM32);
  // Set CS bits for 2014 prescaler
  TCCR3B |= (1 << CS32) | (1 << CS30);   
  // Enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);
}
*/
