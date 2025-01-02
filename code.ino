#include <Wire.h>
#include <mpu6050.h>
#include "SoftPWM.h"

// define function of GPIO
#define PWM1    7
#define DIR1    8
#define PWM2    9
#define DIR2    10
#define CT1     4
#define CT2     6
#define CT3     11

MPU6050 mpu6050;

void mpu6050_begin()  {
  Wire.begin();
  Serial.print("MPU6050: Starting calibration; leave device flat and still ... ");
  int error= mpu6050.begin(); 
  Serial.println(mpu6050.error_str(error));
}

float mpu6050_yaw() {
  MPU6050_t data= mpu6050.get();
  while( data.dir.error!=0 ) { 
    // I suffer from a lot of I2C problems
    Serial.println(mpu6050.error_str(data.dir.error));
    // Reset I2C
    TWCR= 0; Wire.begin();
    // Reread
    data= mpu6050.get();
  }
  return data.dir.yaw;
}


void motor_begin() {
  SoftPWMBegin();
  SoftPWMSet(PWM1, 0);
  SoftPWMSet(PWM2, 0);
  SoftPWMSetFadeTime(PWM1, 2, 2);
  SoftPWMSetFadeTime(PWM2, 2, 2);
  
  pinMode(CT1,INPUT_PULLUP);
  pinMode(CT2,INPUT_PULLUP);  
  pinMode(CT3, INPUT_PULLUP);

  pinMode(DIR1, OUTPUT); //DIR1
  pinMode(PWM1, OUTPUT); //PWM1
  pinMode(PWM2, OUTPUT); //PWM2
  pinMode(DIR2, OUTPUT); //DIR2  

  Serial.println("Motors : ok");
}

void control_motor(int dc, int dir, int spd) {
  switch (dc) {
    case 1: {
        if (dir == 1) {
          digitalWrite(DIR1, 0);
          SoftPWMSet(PWM1, spd);
        } else {
          digitalWrite(DIR1, 1);
          SoftPWMSet(PWM1, 255 - spd);
        }
        break;
      }
    case 0: {
        if (dir == 1) {
          digitalWrite(DIR2, 0);
          SoftPWMSet(PWM2, spd);
        } else {
          digitalWrite(DIR2, 1);
          SoftPWMSet(PWM2, 255 - spd);
        }
        break;
      }
  }
}
void motor_A_set( int speed ) {
  if( speed==0 ) {
    control_motor(0, 0, 0);
  } else if( speed>0 ) {
    control_motor(0, 0, speed);     
  } else {
    control_motor(0, 1, speed);        
  }
}

// Set motor B to given speed (-255..+255); 0 switches off
void motor_B_set( int speed ) {
  if( speed==0 ) {
    control_motor(1, 0, 0);
  } else if( speed>0 ) {
    control_motor(1, 0, speed);     
  } else {
    control_motor(1, 1, speed);        
  }
}

#define MOTOR_NOMINAL  180
#define MOTOR_DELTAMAX  75

// Switches both motors off
void motor_off() {
  motor_A_set(0);
  motor_B_set(0);
}

// Switches both motors to forward (to speed MOTOR_NOMINAL), but B motor 'delta' 
// faster than A motor (delta in range -MOTOR_DELTAMAX..+-MOTOR_DELTAMAX)
void motor_forward(int delta) {
  if( delta>+MOTOR_DELTAMAX ) delta= +MOTOR_DELTAMAX;
  if( delta<-MOTOR_DELTAMAX ) delta= -MOTOR_DELTAMAX;
  motor_A_set(MOTOR_NOMINAL-delta);
  motor_B_set(MOTOR_NOMINAL+delta);
}


// PID ===================================================

#define PID_K_p 40.0
#define PID_K_i  0.3
#define PID_K_d  2.0

float i_input;
float d_last;

void pid_begin() {
  i_input= 0;
  d_last= 0;  
  Serial.println("PID    : ok");
}

int pid(float error) {
  float p_input;
  float d_input;
    
  p_input= error;
  i_input= constrain(i_input+error,-50,+50);
  d_input= error-d_last; d_last=error;

  return p_input*PID_K_p + i_input*PID_K_i + d_input*PID_K_d;
}

// Main ===================================================

bool     drive_squares1 = false; // When true, drives a drive_squaress
bool     drive_straight = false;
bool     drive_squares2 = false;
uint32_t last;   // last time (in ms) we turned 90 degrees at the corner of a drive_squares 

int state;

void setup() {
  Serial.begin(9600);
  Serial.println("\n\nWelcome at PID robot");
  motor_begin();  
  pid_begin();
  mpu6050_begin();
  state = 0;
  motor_off();
  Serial.println();
}

void bam_nut() {
  if (digitalRead(CT1) == 0) {
    drive_squares1 = true;
      state = 1;
  } else if (digitalRead(CT2) == 0) {
    drive_straight = true;
      state = 1;
  } else if (digitalRead(CT3) == 0) {
    drive_squares2 = true;
      state = 1;
  }

}

float target_dir;
int count = 0;
void loop() {
  float current_dir;
  int steer;

  switch( state ) {
    case 0:
      // Motors off
      Serial.println("Readyyyyyyyyyyyyyyyyy");
      
      //bam nut
      while (state != 1) {
        bam_nut();
      }
      break;
    case 1:
      // Motors were off, user pressed button, switch on
      delay(500); // give user some time to release button
      target_dir= mpu6050_yaw(); // Take current direction as target
      if( drive_squares1) {last= millis();}
      else if (drive_squares2) {last = millis();}
      motor_forward(0); // Motors on, 0=steering straight
      Serial.println("Motors started");
      state= 2;
      break;
    case 2:
      // Motors running, PID active
      current_dir= mpu6050_yaw();
      Serial.print(" dir="); Serial.print(current_dir,2);
      Serial.print(" tgt="); Serial.print(target_dir,2);
      steer= pid( target_dir - current_dir );
      Serial.print(" steer="); Serial.println(steer);
      motor_forward(steer);

      if( drive_squares1 ) {
        if ( millis()-last>4000 ) { target_dir+= 90; count++; last= millis(); }

      } else if (drive_squares2) {
        if ( millis()-last>4000 ) { target_dir-= 90; count++;  last= millis(); }

      }

      if (drive_squares1) {
        if (count == 4) {
          motor_off();
          state = 0;
        }
      } else if (drive_squares2) {
        if (count == 4) {
          motor_off();
          state = 0;
        }
      }
      break;
  }
}

