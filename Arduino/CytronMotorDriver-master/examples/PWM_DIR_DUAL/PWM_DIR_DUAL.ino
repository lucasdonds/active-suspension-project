/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************************
 * DESCRIPTION:
 *
 * This example shows how to drive 2 motors using the PWM and DIR pins with
 * 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Arduino D3  - Motor Driver PWM 1 Input
 * Arduino D4  - Motor Driver DIR 1 Input
 * Arduino D9  - Motor Driver PWM 2 Input
 * Arduino D10 - Motor Driver DIR 2 Input
 * Arduino GND - Motor Driver GND
 *
 *
 * AUTHOR   : Kong Wai Weng
 * COMPANY  : Cytron Technologies Sdn Bhd
 * WEBSITE  : www.cytron.io
 * EMAIL    : support@cytron.io
 *
 *******************************************************************************/

 #include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD roadMotor(PWM_DIR, 9, 6);
CytronMD susMotor(PWM_DIR, 8, 10);


// The setup routine runs once when you press reset.
void setup() {
  
}


// The loop routine runs over and over again forever.
void loop() {
  roadMotor.setSpeed(128);   // Motor 1 runs forward at 50% speed.
  susMotor.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
  delay(1000);
  
  roadMotor.setSpeed(255);   // Motor 1 runs forward at full speed.
  susMotor.setSpeed(-255);  // Motor 2 runs backward at full speed.
  delay(1000);

  roadMotor.setSpeed(0);     // Motor 1 stops.
  susMotor.setSpeed(0);     // Motor 2 stops.
  delay(1000);

  roadMotor.setSpeed(-128);  // Motor 1 runs backward at 50% speed.
  susMotor.setSpeed(128);   // Motor 2 runs forward at 50% speed.
  delay(1000);
  
  roadMotor.setSpeed(-255);  // Motor 1 runs backward at full speed.
  susMotor.setSpeed(255);   // Motor 2 runs forward at full speed.
  delay(1000);

  roadMotor.setSpeed(0);     // Motor 1 stops.
  susMotor.setSpeed(0);     // Motor 2 stops.
  delay(1000);
}
