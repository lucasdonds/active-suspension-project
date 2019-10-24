/*
  Final code to run road actuation motor, plot encoder and accelerometer data to serial.
  Top, middle, and bottom plates are referred to as car, wheel, and road respectively.
  Code for MPU9250 accelerometer is commented out.
  Rachel Du
  Aug 2019
  ------------------------
  Edited by: Lucas Dondertman
  Sept 2019, Oct 2019
*/

//-------------------------------

//SETUP!!!!!!!!!

//accelerometer library
#include <MPU9250.h>   

//encoder library
#include <Encoder.h>   

//initialize all pins
//color-coded list of variables: 
//https://docs.google.com/document/d/1rfLVPlw9mu2PoYqc_Dgf6W_8biOQRDDQttkra0Mk32U/edit?usp=sharing
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

//initialize array that stores previous speed values so average can be calculated 
//using fuction: average()
double speedStorage[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double averageSpeed = 0;
double lastCarPos = 0;
double lastWheelPos = 0;
double lastTime = 0;

//initialize encoder objects with numbers of pins connected
Encoder carEnc(carEncA, carEncB);               //top plate encoder
Encoder wheelEnc(wheelEncA, wheelEncB);         //road plate encoder
Encoder roadEnc(roadEncA, roadEncB);            //road plate encoder
Encoder motorEnc(motorEncA, motorEncB);         //encoder on road actuation motor

//an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 accel(Wire, 0x68);
MPU9250 accelTop(Wire, 0x69);
int status;

//counts number of times looped, used in average() function
int loopCounter = 0;

//-------------------------------

//FUNCTIONS!!!!!!!!!

//function that converts position from number of counts to height (m); using counts per 
//revolution and diameter of encoder shaft. diameter in mm, cpr=counts per rev
double encCountsToPosition(long counts, double cpr, double diameter)           
{
  double pos = counts;
  pos /= cpr;                 // = number of revolutions
  pos *= PI * diameter;       // each revolution = height change of 1 circumference
  return pos;
}

//function that finds the speed (RPM) of the motor using the change in position from 
//the motor encoder duration in milliseconds
//fix this func
double findSpeed(long counts, double cpr, double duration)           
{
  double rpm = fabs(counts);
  rpm = rpm/duration;
  rpm = rpm/cpr;                 //counts per revolution of the motor
  rpm = rpm*60000;               //number of milliseconds in one minute
  return rpm;
}

//function that finds the average of all array values parameters: array, size of array
double average(double stored[], int arraySize, int numLoops)        
{
  double total = 0;
  if (numLoops < 20)
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

//function to find velocity of car and wheel plates
//velocity in m/s with direction (+ up/- down)
double findPlateVelocity(double lastPos, double curPos, double duration)
{
  double velocity = curPos - lastPos;   //find dist travelled in mm
  velocity /= duration;                 //get velocity in mm/ms (which = m/s)
  return velocity;
}

//function called by interrupt to toggle direction of sus motor
double toggleSusMotorDir(int PWM)
{
  digitalWrite(susDirA, !digitalRead(susDirA));
  analogWrite(susPWM, PWM);
}

//-------------------------------

//INTERRUPTS!!!!!!!!!!!!!!


//interrupt calling function to toggle motor direction of sus motor
//interrupt gets set up in setup()

ISR(TIMER1_COMPA_vect) {  //change the 1 to 0 for timer0 or 2 for timer2
   toggleSusMotorDir(100);
}

//-------------------------------

void setup() {
  
  //if using serial plotter/monitor ensure that baud rate is the same as this one
  Serial.begin(115200);
  //wait while there is no serial connection
  while (!Serial) {}

  //Send HIGH to ADO port to change MPU9250 adress to 0x69
  digitalWrite(accelTopAD0, HIGH);

  //start communication with accelerometer
  status = accel.begin();
  status = accelTop.begin();

  //set up accelerometers scale factors and bias'
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

  //calibrate accelerometers
  accel.setAccelCalX(axb, axs);
  accel.setAccelCalY(ayb, ays);
  accel.setAccelCalZ(azb, azs);
  accelTop.setAccelCalX(axbTop, axsTop);
  accelTop.setAccelCalY(aybTop, aysTop);
  accelTop.setAccelCalZ(azbTop, azsTop);

  //set accelerometer ranges
  accel.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  accel.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  accel.setSrd(19);
  accelTop.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  accelTop.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  accelTop.setSrd(19);

  //set pinModes (Encoder initializer sets pinMode for encoder objects)
  pinMode(roadDirA, OUTPUT);
  pinMode(susDirA, OUTPUT);

  //set direction for road motor and initial direction for suspension motor
  digitalWrite(roadDirA, HIGH);
  digitalWrite(susDirA, HIGH);

  //PWM value corresponds to freq of bottom (road) plate and how fast motor will spin
  analogWrite(roadPWM, 225);
  
  //set up interrupt for changing the sus motor direction
  //user should change desiredHz to what they desire
  double desiredHz = 1;
  double CMR = (16*pow(10,6)) / (desiredHz*1024) - 1;
  cli();          //stops inturrupts
  TCCR1A = 0;     //set entire TCCR1A register to 0
  TCCR1B = 0;     //same for TCCR1B
  TCNT1  = 0;     //initialize counter value to 0
  //set compare match register for desiredHz increments
  OCR1A = CMR;    //must be <65536
  //turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  //enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          //allow interrupts
  
}

//-------------------------------

void loop() {
  
  //increment loop counter
  loopCounter++;
  
  //save start time
  double startTime = millis();

  //save start position of motor, in counts
  long motorStart = motorEnc.read();

  //read accelerometer
  accel.readSensor();
  accelTop.readSensor();                           

  //find amount of time that has passed since
  double loopDuration = startTime - lastTime;

  //read all encoders for counts
  long carPosInCounts = carEnc.read();
  long wheelPosInCounts = wheelEnc.read();
  long roadPosInCounts = roadEnc.read();
  long motorPosInCounts = motorEnc.read() - motorStart;

  //convert counts measured into actual positions
  //measure actual encoder diameter and update later
  double carPosActual = encCountsToPosition(carPosInCounts, 4048.0, 6.35);  
  double wheelPosActual = encCountsToPosition(wheelPosInCounts, 4048.0, 6.35);
  double roadPosActual = encCountsToPosition(roadPosInCounts, 4048.0, 6.35);

  //find velocity of top two plates, time duration is time between loops
  double carPlateVelocity = findPlateVelocity(lastCarPos, carPosActual, loopDuration);
  double wheelPlateVelocity = findPlateVelocity(lastWheelPos, wheelPosActual, loopDuration);
  
  //speed returned using motor enc counts for the past loop
  double motorSpeed = findSpeed(motorPosInCounts, 3415.92, loopDuration);

  //shift everything in speed storage array one index over
  for (int index = 0; index < 19; index++)                                
  {
    speedStorage[index] = speedStorage[index + 1];
  }
  speedStorage[0] = motorSpeed;

  //average of the last 20 motor speeds
  double averageSpeed = average(speedStorage, 20, loopCounter);
  
  lastCarPos = carPosActual;
  lastWheelPos = wheelPosActual;
  lastTime = startTime;
  /*
  //send serial values to MATLab to be plotted
  Serial.println(millis());                     //print time
  Serial.println(-carPosActual);          //print car position and offset on  graph
  Serial.println(wheelPosActual);          //print wheel position and offset on  graph
  Serial.println(roadPosActual);                //print road position
  Serial.println(motorSpeed);                   //print motorSpeed
  Serial.println(averageSpeed);                 //print average speed
  Serial.println(accel.getAccelX_mss(), 4);     //print x-accel
  Serial.println(accel.getAccelY_mss(), 4);     //print y-accel
  Serial.println(accel.getAccelZ_mss(), 4);     //print z-accel
  Serial.println(accelTop.getAccelX_mss(), 4);  //print x-accel
  Serial.println(accelTop.getAccelY_mss(), 4);  //print y-accel
  Serial.println(accelTop.getAccelZ_mss(), 4);  //print z-accel
  
  //For plotting counts from encoders on serial plotter
  Serial.print("car: "); Serial.print(-carEnc.read()); Serial.print("  ");           
  Serial.print("wheel: "); Serial.print(wheelEnc.read()); Serial.print("  ");
  Serial.print("road: "); Serial.print(roadEnc.read()); Serial.print("  ");
  Serial.println("uT");
  
  //For plotting acceleration in each axis on serial plotter
  Serial.print("x: "); Serial.print(accel.getAccelX_mss(),6); Serial.print("  ");   
  Serial.print("y: "); Serial.print(accel.getAccelY_mss(),6); Serial.print("  ");
  Serial.print("z: "); Serial.print(accel.getAccelZ_mss(),6); Serial.print("  ");
  Serial.println("uT");
  
  //For plotting motor speed and average speed on serial plotter
  Serial.print("speed: "); Serial.print(motorSpeed); Serial.print("  ");              
  Serial.print("average: "); Serial.print(averageSpeed); Serial.print("  ");
  Serial.println("uT");

  //For plotting positions from encoders on serial plotter
  Serial.print("car: "); Serial.print(-carPosActual); Serial.print("  "); 
  Serial.print("wheel: "); Serial.print(wheelPosActual); Serial.print("  ");     
  Serial.print("road: "); Serial.print(roadPosActual); Serial.print("  ");
  Serial.println("uT"); 

  //for plotting car and wheel plate velocities
  Serial.print("car plate velocity: "); Serial.println(carPlateVelocity);
  Serial.print("wheel plate velocity: "); Serial.println(wheelPlateVelocity);
  Serial.println("uT");
*/

 
}
