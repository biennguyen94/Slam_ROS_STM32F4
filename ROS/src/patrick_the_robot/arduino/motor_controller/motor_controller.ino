// Author: Bien Nguyen
// Credits:
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

//ROS headers
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

//Motor Shield headers
//#include <Wire.h>			
//#include <Adafruit_MotorShield.h>
//#include <Adafruit_PWMServoDriver.h>
// encoder 1 have encodPinA1 and encodPinB1 (left wheel)
// encoder 2 have encodPinA2 and encodPinB2 (right wheel)
#define encodPinA1      3     // encoder A pin R
#define encodPinB1      9     // encoder B pin R
#define encodPinA2      2      //A of L
#define encodPinB2      8      //B of L
#define IN1 4//dir 1 R
#define IN2 5//speed 1 R
#define IN3 6//dir 2 L
#define IN4 7//speed 2 L
#define MAX_SPEED 4095 //từ 0-255
#define MIN_SPEED 0
#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)

// Create the motor shield object with the default I2C address
//Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1 & M2
//Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
//Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;
int direction1 = FORWARD;//FORWARD 1, BACKWARD 2, BRAKE 3, RELEASE 4
int direction2 = FORWARD;
//int prev_direction1 = RELEASE;
//int prev_direction2 = RELEASE;
int PWM_val1 = 0;
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0.01;
float Ki =   2;
ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x; //x = linear, that subscribe from cmd_vel
  double z = cmd_msg.angular.z; //z = angular, that subscribe from cmd_vel
  if (z == 0) {     // only go straight (go foward or go back), no rotation
    // convert m/s to rpm (vong/phut)
    rpm_req1 = -x*60/(pi*wheel_diameter);// rpm that calculate after subscribe from cmd_vel(targetValue)
    rpm_req2 = rpm_req1;                // rpm that calculate after subscribe from cmd_vel(targetValue)
  }
  else if (x == 0) { //no foward and no back, only rotation
    // convert rad/s to rpm
    rpm_req1 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = -rpm_req1;
  }
  else { //di cong
    rpm_req1 = -x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = -x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
  }
  
  if (rpm_req1 >= 0) direction1 = FORWARD;
  else direction1 = BACKWARD;
  if (rpm_req2 >= 0) direction2 = FORWARD;
  else direction2 = BACKWARD;
  
  //if (rpm_req1 == 0) motor_1_speed_dir(0,FORWARD); //FORWARD not meaning;
  //if (rpm_req2 == 0) motor_2_speed_dir(0,FORWARD); //FORWARD not meaning;
  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd); // subscribe to cmd_vel topic with geometry_msgs::Twist is message type
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg); // publish to rpm topic with geometry_msgs::Vector3Stamped is message type
ros::Time current_time;
ros::Time last_time;

void motor_1_speed_dir(int speed, bool dir) {
  //speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
  digitalWrite(IN1, dir);// chân này không có PWM
  analogWrite(IN2, speed);
}
void motor_2_speed_dir(int speed, bool dir) {
  //speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
  digitalWrite(IN3, dir);// chân này không có PWM
  analogWrite(IN4, speed); 
}

void setup() {
 //AFMS.begin();  // create with the default frequency 1.6KHz
 //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);// baud = 57600
 nh.subscribe(sub);
 nh.advertise(rpm_pub);
  
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT); 
  
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);                // turn on pullup resistor
 attachInterrupt(1, encoder1, RISING);           //encoder pin on interrupt 1 (pin 3)   refer http://arduino.vn/reference/attachinterrupt

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, encoder2, RISING);           // encoder pin on interrupt 0 (pin 2)
 motor_1_speed_dir(0,BACKWARD); //FORWARD not meaning
 //motor_1_speed_dir(0,FORWARD); //FORWARD not meaning

 motor_2_speed_dir(0,BACKWARD); //FORWARD not meaning
 //motor_2_speed_dir(0,FORWARD); //FORWARD not meaning
 //motor1->setSpeed(0);		// Set the speed to start, from 0 (off) to 4095 (max speed)
 //(only correctly for this case because I have revised Adafruit_MotorShield library with 12bit resolusion), 
 //showed here: void Adafruit_DCMotor::setSpeed(uint16_t speed)
 //motor2->setSpeed(0);
 
 //To run the motor,
 //call run(direction) where direction is FORWARD, BACKWARD or RELEASE.
 //Of course, the Arduino doesn't actually know if the motor is 'forward' or 'backward',
 //so if you want to change which way it thinks is forward,
 //simply swap the two wires from the motor to the shield.
 //motor1->run(FORWARD);
 //motor1->run(RELEASE);
 //motor2->run(FORWARD);
 //motor2->run(RELEASE);
}

void loop() {
  nh.spinOnce();// if you were to add a subscription into this application, and did not have ros::spinOnce() here, 
                  //your callbacks would never get called. So, add it for good measure.
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);
    /*
	if (rpm_req1 > MAX_RPM) 
	{rpm_req1 = MAX_RPM;
	rpm_req2 = MAX_RPM;
	}
*/
   
    //PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);            //PWM pulse that must to do control DC(command)
    //PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);            //PWM pulse that must to do control DC(command)
    PWM_val1 = updatePid1(PWM_val1, rpm_req1, rpm_act1); // compute PWM value
    PWM_val2 = updatePid2(PWM_val2, rpm_req2, rpm_act2); // compute PWM value
    /*
    if(PWM_val1 > 0) direction1 = FORWARD;
    else if(PWM_val1 < 0) direction1 = BACKWARD;
    if(PWM_val2 > 0) direction2 = FORWARD;
    else if(PWM_val2 < 0) direction2 = BACKWARD;
    */
    motor_1_speed_dir(abs(PWM_val1), direction1);
    motor_2_speed_dir(abs(PWM_val2), direction2);

    if (rpm_req1 == 0) motor_1_speed_dir(0,FORWARD); //FORWARD not meaning;
    if (rpm_req2 == 0) motor_2_speed_dir(0,FORWARD); //FORWARD not meaning;

    //motor1->run(direction1);
    //motor2->run(direction2);

    //motor1->setSpeed(abs(PWM_val1));
    //motor2->setSpeed(abs(PWM_val2));
    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}

void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);// value of rpm in actual(currentValue)
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);// value of rpm in actual(currentValue)
 countAnt1 = count1;
 countAnt2 = count2;
}
/*
int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/255.0 + pidTerm, -MAX_RPM, MAX_RPM);//refer http://arduino.vn/reference/constrain ///// MAX_RPM = 58 vong/phut
  new_cmd = 255.0*new_pwm/MAX_RPM; // in here it's 12bit, its own on-board pwm chip with 12 bit resolution,
  //but for some strange reason the dc motor library often used 8 bits.
  return int(new_cmd);
  //return constrain(double(command) + int(pidTerm), -255, 255);
}
*/
int updatePid1(int command, int targetValue, int currentValue) {
float pidTerm = 0;                            // PID correction
int error=0;                                  
static int last_error1=0;
static double int_error1 = 0;
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error1)) + (Ki*int_error1);                            
 last_error1 = error;
 return constrain(command + int(pidTerm), 0, 255);
}

int updatePid2(int command, int targetValue, int currentValue) {
float pidTerm = 0;                            // PID correction
int error=0;                                  
static int last_error2=0;    
static double int_error2 = 0;
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error2)) + (Ki*int_error2);                            
 last_error2 = error;
 return constrain(command + int(pidTerm), 0, 255);
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}
