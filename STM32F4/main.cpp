/*
 * Author: Bien Nguyen
 * Company: Ho Chi Minh City University of Technlogy
 * Email: nguyennhubientdh@gmail.com
 */

#include "mbed.h"
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)
#define encoder_pulse   13    // so xung tren 1 vong 13*19.2 = 250xung/vong   11
#define gear_ratio      19.2   //ti so giam toc			34
#define wheel_diameter  0.1   //m
#define wheel_width     0.0285   //m
#define track_width     0.390   //m
#define MAX_RPM         38      //20cm/s
#define pi              3.1415926
#define two_pi          6.2831853
#define FORWARD         0
#define BACKWARD        1
unsigned long lastMilli = 0;       // loop timing
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
int direction1 = FORWARD;
int direction2 = FORWARD;
int PWM_val1 = 0;
int PWM_val2 = 0;
static long count1=0;
static long count2=0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0.01;
float Ki =   1;

ros::NodeHandle  nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg)
{
    double x = cmd_msg.linear.x; //x = linear, that subscribe from cmd_vel
    double z = cmd_msg.angular.z; //z = angular, that subscribe from cmd_vel
    if (z == 0) {     // only go straight (go foward or go back), no rotation
        // convert m/s to rpm (vong/phut)
        rpm_req1 = -x*60/(pi*wheel_diameter);// rpm that calculate after subscribe from cmd_vel(targetValue)
        rpm_req2 = rpm_req1;                // rpm that calculate after subscribe from cmd_vel(targetValue)
    } else if (x == 0) { //no foward and no back, only rotation
        // convert rad/s to rpm
        rpm_req1 = z*track_width*60/(wheel_diameter*pi*2);
        rpm_req2 = -rpm_req1;
    } else { //di cong (no straight)
        rpm_req1 = -x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
        rpm_req2 = -x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    }

    if (rpm_req1 >= 0) direction1 = FORWARD;
    else direction1 = BACKWARD;
    if (rpm_req2 >= 0) direction2 = FORWARD;
    else direction2 = BACKWARD;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handle_cmd); // subscribe to cmd_vel topic with geometry_msgs::Twist is message type
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg); // publish to rpm topic with geometry_msgs::Vector3Stamped is message type
ros::Time current_time;
ros::Time last_time;

unsigned long time_now=0;

InterruptIn encoderA1(PA_1);
InterruptIn encoderB1(PA_3);

InterruptIn encoderA2(PA_0);
InterruptIn encoderB2(PA_2);

DigitalOut dir1(PA_4);
DigitalOut dir3(PC_5);
PwmOut pwm1(PA_6);

DigitalOut dir2(PA_5);
PwmOut pwm2(PA_7);

Timer t;
void flip_right_A_rise()
{
    if(encoderA1==encoderB1) {
        count1--;
    } else count1++;
}

void flip_right_A_fall()
{
    if(encoderA1==encoderB1) {
        count1--;
    } else count1++;
}

void flip_right_B_rise()
{
    if(encoderA1==encoderB1) {
        count1++;
    } else count1--;
}

void flip_right_B_fall()
{
    if(encoderA1==encoderB1) {
        count1++;
    } else count1--;
}

void flip_left_A_rise()
{
    if(encoderA2==encoderB2) {
        count2--;
    } else count2++;
}

void flip_left_A_fall()
{
    if(encoderA2==encoderB2) {
        count2--;
    } else count2++;
}

void flip_left_B_rise()
{
    if(encoderA2==encoderB2) {
        count2++;
    } else count2--;
}

void flip_left_B_fall()
{
    if(encoderA2==encoderB2) {
        count2++;
    } else count2--;
}
void motor_1_speed_dir(int speed, bool dir) {
	dir1=dir;
  pwm1.pulsewidth(speed*0.002/511);
}
void motor_2_speed_dir(int speed, bool dir) {
  dir2=dir;
	pwm2.pulsewidth(speed*0.002/511);
}
void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio*4);// value of rpm in actual(currentValue)
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio*4);// value of rpm in actual(currentValue)
 countAnt1 = count1;
 countAnt2 = count2;
}
int updatePid1(int command, double targetValue, double currentValue) {
double pidTerm = 0;                            // PID correction
double error=0;                                  
static double last_error1=0;                             
error = abs(targetValue) - abs(currentValue); 
pidTerm = (Kp * error) + (Kd * (error - last_error1));                            
last_error1 = error;
if(command + int(pidTerm)>=511)
		return 511;
else if(command + int(pidTerm)<=0)
		return 0;
else
	return int(command + int(pidTerm));
}

int updatePid2(int command, double targetValue, double currentValue) {
double pidTerm = 0;                            // PID correction
double error=0;                                  
static double last_error2=0;                             
error = abs(targetValue) - abs(currentValue); 
pidTerm = (Kp * error) + (Kd * (error - last_error2));                            
last_error2 = error;
if(command + int(pidTerm)>=511)
		return 511;
else if(command + int(pidTerm)<=0)
		return 0;
else
	return int(command + int(pidTerm));
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}
int main()
{
		count1 = 0;
	  count2 = 0;
	  countAnt1 = 0;
	  countAnt2 = 0;
	  rpm_act1 = 0;
	  rpm_act2 = 0;
	  PWM_val1 = 0;
	  PWM_val2 = 0;
    t.start();
    nh.initNode();
		nh.getHardware()->setBaud(57600);
		nh.subscribe(sub);
		nh.advertise(rpm_pub);
		
			
    encoderA1.mode(PullUp);
    encoderB1.mode(PullUp);
    encoderA1.rise(&flip_right_A_rise);
	  encoderA1.fall(&flip_right_A_fall);
	  encoderB1.rise(&flip_right_B_rise);
	  encoderB1.fall(&flip_right_B_fall);

    encoderA2.mode(PullUp);
    encoderB2.mode(PullUp);
    encoderA2.rise(&flip_left_A_rise);
	  encoderA2.fall(&flip_left_A_fall);
	  encoderB2.rise(&flip_left_B_rise);
	  encoderB2.fall(&flip_left_B_fall);

    pwm1.period_ms(2);      // 2 milisecond period
		pwm2.period_ms(2);      // 2 milisecond period
		motor_1_speed_dir(0,BACKWARD); //BACKWARD not meaning
		motor_2_speed_dir(0,BACKWARD);
    while (1) {
        nh.spinOnce();
				time_now = t.read_ms();
				if(time_now-lastMilli>= LOOPTIME)   
				{     
						getMotorData(time_now-lastMilli);
							
						PWM_val1 = updatePid1(PWM_val1, rpm_req1, rpm_act1); // compute PWM value
						PWM_val2 = updatePid2(PWM_val2, rpm_req2, rpm_act2); // compute PWM value
						motor_1_speed_dir(abs(PWM_val1), direction1);
						motor_2_speed_dir(abs(PWM_val2), direction2);

						if (rpm_req1 == 0) motor_1_speed_dir(0,FORWARD); //FORWARD not meaning;
						if (rpm_req2 == 0) motor_2_speed_dir(0,FORWARD); //FORWARD not meaning;
						publishRPM(time_now-lastMilli);
						lastMilli = time_now;
				}		
    }
}

