/*
  Final code to run road actuation motor, plot encoder and accelerometer data to serial.
  Top, middle, and bottom plates are referred to as car, wheel, and road respectively.
  Code for MPU9250 accelerometer is commented out.
  Rachel Du
  Aug 2019
  ------------------------
  Edited by: Lucas Dondertman
  Sept 2019, Oct 2019, Nov 2019
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
int carEncA = 2;
int wheelEncA = 3;
int carEncB = 4;
int wheelEncB = 5;
int roadDirA = 6;
int susPWM = 8;
int roadPWM = 9;
int motorEncA = 18;
int roadEncB = 17;
int motorEncB = 16;
int roadEncA = 19;  
int susDirA = 10;
int accelTopAD0 = 13;

// Initialize array that stores previous speed values so average can be calculated 
// Using fuction: average()
double speedStorage[20] = { 0 };
double averageSpeed = 0;
double lastCarPos = 0;
double lastWheelPos = 0;
double lastRoadPos = 0;
double lastTime = 0;

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 accel(Wire, 0x68);
MPU9250 accelTop(Wire, 0x69);
int status;

// Initialize encoder objects with numbers of pins connected
Encoder carEnc(carEncA, carEncB);               //top plate encoder
Encoder wheelEnc(wheelEncA, wheelEncB);         //road plate encoder
Encoder roadEnc(roadEncA, roadEncB);            //road plate encoder
Encoder motorEnc(motorEncA, motorEncB);         //encoder on road actuation motor

// Counts number of times looped, used in average() function
int loopCounter = 0;

// Bool to control speed of road motor, initialized to false
bool speed1Active = false;

// Vertical shift to zero position graphs like a y=-Cos(x) wave
double vShiftCar = 0;
double vShiftWheel = 0;
double vShiftRoad = 0;

// PID initializations
double prevError = 0;
double integral = 0;
double setpoint = 0;
double Kp = 0.1;
double Ki = 0;
double Kd = 0;

////////////////////////////////////////////

// FUNCTION DECLARATIONS!!!!!!
// All functions are below loop(), with explanations of their purpose and how they work

double encCountsToPosition(long counts, double cpr, double diameter);

double findSpeed(long counts, double cpr, double duration);

double average(double stored[], int arraySize, int numLoops);

double findPlateVelocity(double lastPos, double curPos, double duration);

double toggleSusMotorDir(int PWM);

int toggleRoadMotorSpeed(int speed1, int speed2);

void resetRoadMotor();

void serialHandshake();

void setupTimer1(double desiredHz1);

void setupTimer3(double desiredHz3);

double getPidError();

////////////////////////////////////////

// INTERRUPTS!!!!!!!!!!!!!!
// Interrupts get set up in setup()
// Disable interrupts by commenting out whats inside the interrupt, but keep the interrupt

// Interrupt for toggling speed of road motor
ISR(TIMER3_COMPA_vect) {  //change the 1 to 0 for timer0 or 2 for timer2
   toggleRoadMotorSpeed(200, 200);
}

// Interrupt calling function to toggle motor direction of sus motor
ISR(TIMER1_COMPA_vect) {  //change the 1 to 0 for timer0 or 2 for timer2
   toggleSusMotorDir(50);
}

/////////////////////////////////////////

void setup() {
  
  // If using serial plotter/monitor ensure that baud rate is the same as this one
  Serial.begin(115200);

  // Initial connection with matlab
  //serialHandshake();

  // Send HIGH to ADO port on top accelerometer to change MPU9250 adress to 0x69
  digitalWrite(accelTopAD0, HIGH);

  // Start communication with accelerometer
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

  // Set pinModes (Encoder initializer sets pinMode for encoder objects)
  pinMode(roadDirA, OUTPUT);
  pinMode(susDirA, OUTPUT);

  // Set direction for road motor and initial direction for suspension motor
  digitalWrite(roadDirA, HIGH);
  digitalWrite(susDirA, LOW);

  // Initialize all timer interrupts
  cli();          //stops inturrupts
  setupTimer1(1);
  setupTimer3(0.25);
  sei();          //allow interrupts
  
  resetRoadMotor();
  
}

////////////////////////////////////////

void loop() {
  
  // Increment loop counter
  loopCounter++;
  
  // Save start time
  double startTime = millis();
  
  // Find amount of time that has passed since
  double loopDuration = startTime - lastTime;

  // Save start position of motor, in counts
  long motorStart = motorEnc.read();

  // Read accelerometer
  accel.readSensor();
  accelTop.readSensor();                           

  // Read all encoders for counts
  long carPosInCounts = carEnc.read();
  long wheelPosInCounts = wheelEnc.read();
  long roadPosInCounts = roadEnc.read();
  long motorPosInCounts = motorEnc.read() - motorStart;

  // Convert counts measured into actual positions
  // Measure actual encoder diameter and update later
  double carPosActual = encCountsToPosition(carPosInCounts, 4048.0, 6.35)+vShiftCar;  
  double wheelPosActual = encCountsToPosition(wheelPosInCounts, 4048.0, 6.35)+vShiftWheel;
  double roadPosActual = encCountsToPosition(roadPosInCounts, 4048.0, 6.35)+vShiftRoad;

  double carPlateVelocity = findPlateVelocity(lastCarPos, carPosActual, loopDuration);
  double wheelPlateVelocity = findPlateVelocity(lastWheelPos, wheelPosActual, loopDuration);
  double roadPlateVelocity = findPlateVelocity(lastRoadPos, roadPosActual, loopDuration);

  // Speed returned using motor enc counts for the past loop
  double motorSpeed = findSpeed(motorPosInCounts, 3415.92, loopDuration);
 
  // Shift everything in speed storage array one index over and store new variable
  for (int index = 0; index < 19; index++)                                
  {
    speedStorage[index] = speedStorage[index + 1];
  }
  speedStorage[0] = motorSpeed;

  // Average of the last 20 motor speeds
  double averageSpeed = average(speedStorage, 20, loopCounter);

  error = pidOutput(carPosActual, loopDuration);
  
  lastCarPos = carPosActual;
  lastWheelPos = wheelPosActual;
  lastRoadPos = roadPosActual;
  lastTime = startTime;
///*
  // Send serial values to MATLab to be plotted
  Serial.println(millis());                     //print time
  Serial.println(carPosActual);                 //print car position and offset on  graph
  Serial.println(wheelPosActual);               //print wheel position and offset on  graph
  Serial.println(roadPosActual);                //print road position
  Serial.println(motorSpeed);                   //print motorSpeed
  Serial.println(averageSpeed);                 //print average speed
  Serial.println(accel.getAccelX_mss(), 4);     //print x-accel
  Serial.println(accel.getAccelY_mss(), 4);     //print y-accel
  Serial.println(accel.getAccelZ_mss(), 4);     //print z-accel
  Serial.println(accelTop.getAccelX_mss(), 4);  //print x-accel
  Serial.println(accelTop.getAccelY_mss(), 4);  //print y-accel
  Serial.println(accelTop.getAccelZ_mss(), 4);  //print z-accel
  Serial.println(carPlateVelocity);             //print car plate velocity
  Serial.println(wheelPlateVelocity);           //print wheel plate velocity
  Serial.println(roadPlateVelocity);            //print road plate velocity
  // Negative velocities account for the encoders being upsidedown or something, idk what the problem is yet
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
  Serial.println("uT"); 
/*
  // For plotting car and wheel plate velocities
  Serial.print("car plate velocity: "); Serial.println(carPlateVelocity);
  Serial.print("wheel plate velocity: "); Serial.println(wheelPlateVelocity);
  Serial.println("uT");

  // For plotting road motor enc counts
  Serial.print("Road motor enc counts"); Serial.println(motorEnc.read());
  */

  // For testing PID controller
}

///////////////////////////////////////////////////

// FUNCTIONS!!!!!!!!!

// Function that converts position from number of counts to height (mm); using counts per 
// revolution, diameter of encoder shaft (in mm), cpr=counts per rev
double encCountsToPosition(long counts, double cpr, double diameter)           
{
  double pos = counts;
  pos /= cpr;                 // = number of revolutions
  pos *= PI * diameter;       // each revolution = height change of 1 circumference
  return pos;
}

// Function that finds the speed (RPM) of the motor using the change in position from 
// the motor encoder duration in milliseconds
double findSpeed(long counts, double cpr, double duration)           
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
  double total = 0;
  if (numLoops < arraySize)
  {
    for (int index = 0; index < numLoops; index++)
      total += stored[index];
    total /= numLoops;
  }
  else {
    for (int index = 0; index < arraySize; index++)
      total += stored[index];
    total /= arraySize;
  }
  return total;
}

// Function to find velocity of car and wheel plates (m/s)
// Velocity in m/s with direction (+ up/- down)
double findPlateVelocity(double lastPos, double curPos, double duration)
{
  double velocity = curPos - lastPos;   //find dist travelled in mm
  velocity /= duration;                 //get velocity in mm/ms (which = m/s)
  return velocity;
}

// Function called by interrupt to toggle direction of sus motor
double toggleSusMotorDir(int PWM)
{
  digitalWrite(susDirA, !digitalRead(susDirA));
  analogWrite(susPWM, PWM);
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

// Function to
void resetRoadMotor()
{
  int smallDelay = 30;
  int testDelay = 50;
  double firstPos = 0;
  double secondPos = 0;
  double tol = 0.05;
  
  while (tol >= abs(firstPos-secondPos)) {
  firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  delay(smallDelay);
  secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  delay(testDelay); 
  }
  
  if (secondPos>firstPos)  // Going up
  {
    // Wait for peak
    while (secondPos>firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);  
      delay(testDelay); 
    }
    // Then wait for trough
    while (secondPos<firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      delay(testDelay);  
    }
    // Motor back at bottom so were good
  }
  else // Going down already
  {
    // Wait for trough
    while (secondPos<firstPos)
    {
      firstPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35);  
      delay(smallDelay);
      secondPos = encCountsToPosition(carEnc.read(), 4048.0, 6.35); 
      delay(testDelay);  
    }
    // Motor back at bottom so were good
  } 
  
  vShiftCar = -1*encCountsToPosition(carEnc.read(), 4048.0, 6.35);
  vShiftWheel = -1*encCountsToPosition(wheelEnc.read(), 4048.0, 6.35);
  vShiftRoad = -1*encCountsToPosition(roadEnc.read(), 4048.0, 6.35);
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

// Function to get PID adjustment output for sus motor
// measuredPos (mm), duration (ms), 
double pidOutput(double measuredPos, double duration)
{
  double error = setpoint - measuredPos; // mm
  integral += error * elapsedTime;   // mm*ms = m*s (absement)
  double derivative = (error - prevError) / elapsedTime; // mm/ms = m/s (velocity)
  double output = Kp * error + Ki * integral + Kd * derivative; // what to do with the output?!
  prevError = error;
  return error;
  //return output;
}
