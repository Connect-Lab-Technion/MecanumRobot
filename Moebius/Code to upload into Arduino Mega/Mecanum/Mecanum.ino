#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "MPU6050.h"
#include "config.h"

//-----------------include ros msgs-----------------------
#include "mbs_msgs/Velocities.h"
#include "mbs_msgs/RawImu.h"
#include "mbs_msgs/PID.h"

#include <Wire.h>
#include <TimerFive.h>
#include "Mecanum_Car.hpp"


#define ENCODER_OPTIMIZE_INTERRUPTS

//定义电机端口、小车类型
DC_Motor   LF_Wheel_Motor(PORT_LF), RF_Wheel_Motor(PORT_RF),LR_Wheel_Motor(PORT_LR), RR_Wheel_Motor(PORT_RR);
Mecanum_Car  Robot(&LF_Wheel_Motor, &RF_Wheel_Motor, &LR_Wheel_Motor, &RR_Wheel_Motor);
MPU6050 Mpu6050;

double g_req_angular_vel_z;
double g_req_linear_vel_x;
double g_req_linear_vel_y;

unsigned long g_prev_command_time = 0;
unsigned long g_prev_control_time = 0;
unsigned long g_publish_vel_time = 0;
unsigned long g_prev_imu_time = 0;
unsigned long g_prev_debug_time = 0;

bool g_is_first = true;

char g_buffer[50];

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const mbs_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<mbs_msgs::PID> pid_sub("pid", PIDCallback);

mbs_msgs::Imu  raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

mbs_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void PID_control()
{
  Robot.Increment_PID();  //PID控制器
}

void setup()
{
    Serial.begin(115200);
    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
//    version_info();
    Mpu6050.initialize();           //初始化MPU6050
    Timer5.initialize(1000);
    Timer5.attachInterrupt(PID_control); // blinkLED to run every 0.1 seconds
    Robot.Init();
    //while (!nh.connected())
    {
      nh.spinOnce();
    }

    nh.loginfo("Ros Connected!");
    
    Wire.begin();
    delay(5);

}


void loop()
{
    //this block drives the robot based on defined rate
    if ((millis() - g_prev_control_time) >= (1000 / COMMAND_RATE))
    {
        Robot.ROS_MoveBase(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
        // Robot.ROS_MoveBase(0, 0.5, 0);
        g_prev_control_time = millis();  
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }
  
    //this block publishes velocity based on defined rate
    if ((millis() - g_publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
    {
        publishVelocities();
        g_publish_vel_time = millis();
    }
  
    //this block publishes the IMU data based on defined rate
    if ((millis() - g_prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        if (g_is_first)
        {
            Robot.ClearOdom();
            g_is_first = false;
        }
        else
        {
            publishIMU();
        }
        g_prev_imu_time = millis();
    }
  
    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if ((millis() - g_prev_debug_time) >= (1000 / DEBUG_RATE)){
//        printDebug();
        g_prev_debug_time = millis();
        Robot.ReadOdom();
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}

void PIDCallback(const mbs_msgs::PID& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    Robot.Update_PID(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
    //Serial.println(g_req_linear_vel_x);
    //Serial.println(g_req_linear_vel_y);
    //Serial.println(g_req_angular_vel_z);
    
}

void stopBase()
{
    g_req_linear_vel_x = 0.0;
    g_req_linear_vel_y = 0.0;
    g_req_angular_vel_z = 0.0;
}

void publishVelocities()
{
    long LF_Wheel_Spd = LF_Wheel_Motor.getMotorRPM();
    long RF_Wheel_Spd = RF_Wheel_Motor.getMotorRPM();
    long LR_Wheel_Spd = LR_Wheel_Motor.getMotorRPM();
    long RR_Wheel_Spd = RR_Wheel_Motor.getMotorRPM();
    
    double average_rpm_x = (LF_Wheel_Spd + LR_Wheel_Spd + RF_Wheel_Spd + RR_Wheel_Spd) / 4;
    raw_vel_msg.linear_x = (average_rpm_x * (0.068 * PI)); // m/s
    
    double average_rpm_y = (-LF_Wheel_Spd + LR_Wheel_Spd + RF_Wheel_Spd - RR_Wheel_Spd) / 4;
    raw_vel_msg.linear_y = (average_rpm_y * (0.068 * PI)); // m/s
    
    double average_rpm_a = (-LF_Wheel_Spd + LR_Wheel_Spd - RF_Wheel_Spd + RR_Wheel_Spd) / 4;
    raw_vel_msg.angular_z =  (average_rpm_a * (0.068 * PI)) / 0.26;
    
    //publish raw_vel_msg object to ROS
    raw_vel_pub.publish(&raw_vel_msg);
}

void publishIMU()
{
  int16_t ax, ay, az, gx, gy, gz;             //MPU6050的三轴加速度和三轴陀螺仪数据
  
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据

  long LF_Wheel_Spd = LF_Wheel_Motor.getMotorRPM();
  long RF_Wheel_Spd = RF_Wheel_Motor.getMotorRPM();
  long LR_Wheel_Spd = LR_Wheel_Motor.getMotorRPM();
  long RR_Wheel_Spd = RR_Wheel_Motor.getMotorRPM();

  struct inputt
  {
    float LF;
    float RF;
    float LR;
    float RR;
  };
  //struct inputt expected_speed;
  // struct inputt expected_speed=Robot.ROS_MoveBase(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  //陀螺仪量程 ±250°/s, 加速度量程 : ±8g
  //this function publishes raw IMU reading
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";
  //measure accelerometer
  raw_imu_msg.raw_linear_acceleration.x = (float)ax / 360.072;
  // raw_imu_msg.raw_linear_acceleration.x=LF_Wheel_Spd;
  raw_imu_msg.raw_linear_acceleration.y = (float)ay / 360.072;
  // raw_imu_msg.raw_linear_acceleration.y =RF_Wheel_Spd;
  raw_imu_msg.raw_linear_acceleration.z = (float)az / 360.072;
  // raw_imu_msg.raw_linear_acceleration.z=LR_Wheel_Spd;
  //measure gyroscope
  raw_imu_msg.raw_angular_velocity.x = (float)gx / 4096.0;
  // raw_imu_msg.raw_angular_velocity.x = RR_Wheel_Spd;
  raw_imu_msg.raw_angular_velocity.y = (float)gy / 4096.0*0;
  raw_imu_msg.raw_angular_velocity.z = (float)gz / 4096.0*0;
  //measure magnetometer
  raw_imu_msg.raw_magnetic_field.x = LF_Wheel_Motor.getEncoderPosition();
  raw_imu_msg.raw_magnetic_field.y = RF_Wheel_Motor.getEncoderPosition();
  raw_imu_msg.raw_magnetic_field.z = LR_Wheel_Motor.getEncoderPosition();
  //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);

  }
//}

void printDebug()
{
  sprintf (g_buffer, "Encoder LeftFront: %ld", LF_Wheel_Motor.getEncoderPosition());
  nh.loginfo(g_buffer);
  sprintf (g_buffer, "Encoder RightFront: %ld", RF_Wheel_Motor.getEncoderPosition());
  nh.loginfo(g_buffer);
  sprintf (g_buffer, "Encoder LeftRear: %ld", LR_Wheel_Motor.getEncoderPosition());
  nh.loginfo(g_buffer);
  sprintf (g_buffer, "Encoder RightRear: %ld", RR_Wheel_Motor.getEncoderPosition());
  nh.loginfo(g_buffer);
}
