#ifndef _ONBOARD_MAH01_H_
#define _ONBOARD_MAH01_H_

#include <Encoder.h>


//Motor param config
	#define MotorType					DC_Motor

// define your robot' specs here
	#define MOTOR_MAX_PWM 255 // motor's maximum RPM
	#define MAX_RPM 158 // motor's maximum RPM
	#define COUNTS_PER_REV 2300 // wheel encoder's no of ticks per rev
	#define WHEEL_DIAMETER 0.068 // wheel's diameter in meters
	#define PWM_BITS 8 // PWM Resolution of the microcontroller
	#define BASE_WIDTH 0.26 // width of the plate you are using
	
//PID parmeter
	#define K_P 2.0 // P constant
	#define K_I 0.2 // I constant
	#define K_D 0.2 // D constant
	
struct port_t
{
   int Dir_A;
   int Dir_B;
   int PWM_Pin;
   int Encoder_A;
   int Encoder_B;
  //  int Encoder_A;
  //  int Dir_A;
  //  int Encoder_B;
  //  int PWM_Pin;
  //  int Dir_B;
};

//====================
//====Mecanum car=====
//                      
//  A\\-(X+)-//(B)
//    |      |
//   (Y-)   (Y+) Right
//    |      |
//  C//-(X-)-\\(D)
//
//===================
// Define motor port
#define PORT_LF   0
#define PORT_RF   1
#define PORT_LR   2
#define PORT_RR   3

#ifdef HIGH_POWER_DRIVE
// port_t port[4] =
// {
// A    B   PWM  EN_A  EN_B
 // {A4,  A5,   4,  18,  31},   // A-- Left  Front(LF)
  //{42,  43,   6,  19,  38},   // B-- Right Front(RF)
  //{36,  37,   7,   2,  A1},   // C-- Left  Rear (LR)
  //{34,  35,  11,   3,  49},   // D-- Right Rear (RR)

  // {A5,  A4,   5,   23,  A1},   // D-- Left  Front M4  //the encoder looks fine and PWM looks fine, but dir A and B is not good.
  // {42,  43,   6,   24,  49},   // C-- Right Front M2  
  // {36,  37,   8,  25,  38},   // B-- Left  Rear  M3
  // {35,  34,  12,  26,  31},   // A-- Right Rear  M1
  // Serial.print("Motor Prot : ");
// };
#else
// Serial.print("Motor Prot : ");
port_t port[4] =
 
{

//correct port   of first robot
// A    B   PWM  EN_A  EN_B
//    M+   M-   PWM  EN_A  EN_B
  {A5,  A4,   5,   2,  A1},   // D-- Left  Front M4
  {42,  43,   6,   3,  49},   // C-- Right Front M2  
  {36,  37,   8,  19,  38},   // B-- Left  Rear  M3
  {35,  34,  12,  18,  31},   // A-- Right Rear  M1
// EN_A  M+   EN_B   PWM  M-   
  // {2,   A5,   A1,    5,   A4},   // D-- Left  Front M4
  // {3,   42,   49,    6,   43 },   // C-- Right Front M2  
  // {19,  36,   38,    8 ,  37},   // B-- Left  Rear  M3
  // {18,  35,   31,    12 , 34  },   // A-- Right Rear  M1

  ///

//REVERSE IT
  // { A1,   2,   5,  A4,A5 },   // D-- Left  Front M4
  // {49,   3,   6,  43, 42},   // C-- Right Front M2  
  // {38,19,  8,  37,  36    },   // B-- Left  Rear  M3
  // {31,  18,  12,  34,  35},   // A-- Right Rear  M1
// test port
// #define PORT_LF   0
// #define PORT_RF   1
// #define PORT_LR   2
// #define PORT_RR   3
// A    B   PWM  EN_A  EN_B
  // {27,  26,   5,   2,  A1},   // D--  
  // {42,  43,   6,   3,  49},   // C-- Right Front M2  
  // {36,  37,   8,  19,  38},   // B-- Left  Rear  M3
  // {35,  34,  12,  18,  31},   // A-- Right Rear  M1
  //Serial.print("Motor Prot : ")
//port from diagram   
  // {27,  26,   5,   2,  A1},   // D-- Left  Front M4
  // {42,  43,   9,   3,  49},   // C-- Right Front M2  
  // {36,  37,   8,  19,  38},   // B-- Left  Rear  M3
  // {35,  34,  12,  18,  31},   // A-- Right Rear  M1

};
#endif

class DC_Motor
{
private:
    int Dir_A, Dir_B;
    int PWM_Pin;
    int Encoder_A, Encoder_B;
    Encoder *_encoder;
    int dir;
    float kp, ki, kd;
	int Port;
    long prev_encoder_ticks_;
    int Last_tar, Bias, Last_bias, Pwm, Last_Pwm, Out_Pwm;
    int rpm;

public:
    DC_Motor(int Motor_Port)    //实例化电机端口
    {
        Port = Motor_Port;
        Dir_A = port[Port].Dir_A;
        Dir_B = port[Port].Dir_B;
        PWM_Pin = port[Port].PWM_Pin;
        Encoder_A = port[Port].Encoder_A;
        Encoder_B = port[Port].Encoder_B;
    }
    int Init(int _dir)
    {
        pinMode(Dir_A, OUTPUT);
        pinMode(Dir_B, OUTPUT);
        pinMode(PWM_Pin, OUTPUT);
        _encoder = new Encoder(Encoder_A, Encoder_B);
        kp = K_P;
        ki = K_I;
        kd = K_D;
        dir = _dir;
    }
	
	void PrintPortConfig()
	{
    /*
        Serial.print("Motor Prot : ");
        Serial.print(Port);   //左轮编码器
        Serial.print(", Dir : ");
        Serial.print(Dir_A);
        Serial.print(", ");
        Serial.print(Dir_B);
        Serial.print("\t, PWM : ");
        Serial.print(PWM_Pin);
        Serial.print(", Encoder : ");
        Serial.print(Encoder_A);
        Serial.print(", ");
        Serial.println(Encoder_B);
        */
	}
	
    int setSpd(int spd)
    {
        spd = dir?(-spd):(spd);
        if(spd > 0)
        {
            digitalWrite(Dir_A, HIGH);
            digitalWrite(Dir_B, LOW);
        }
        else if(spd < 0)
        {
            digitalWrite(Dir_A, LOW);
            digitalWrite(Dir_B, HIGH);
        }
        else
        {
            digitalWrite(Dir_A, LOW);
            digitalWrite(Dir_B, LOW);
        }
		analogWrite(PWM_Pin, abs(spd));  
    }

    long getEncoderPosition()
    {
        long position = _encoder->read();
        return dir ? -position : position;
    }
	
    void clrEncoderPosition()
    {
        _encoder->write(0);
    }
	
	int getMotorRPM()
	{
		return dir ? -rpm : rpm;
	}

  int getMotorDirection()
	{
		return Dir_A;
	}

  /*uint8_t getState(){
  uint8_t state = _encoder->stateread();
    return state;
  }

  volatile uint8_t getBitmask(){
    volatile uint8_t bitmask1,bitmask2 = _encoder->bitmaskread();
  }

  int getVoltage(){
    int voltage1,voltage2 = _encoder->voltageread();
  }

  uint8_t getSignal1(){
    uint8_t signal1 = _encoder->signalRead1();
    return signal1;
  }
  
  uint8_t getSignal2(){
    uint8_t signal2 = _encoder->signalRead2();
    return signal2;
  }*/
  
	
    void updateSpd()
    {
//      this function calculates the motor's RPM based on encoder ticks and delta time
//      convert the time from milliseconds to minutes
//      这里没有用浮点运算, dt固定为10ms, 降低MCU处理负荷
//      unsigned long current_time = millis();
//      unsigned long dt = current_time - prev_update_time_;
//      double dtm = (double)dt / 60000;
        double dtm = 0.000167;
        double delta_ticks = getEncoderPosition() - prev_encoder_ticks_;

        rpm = (delta_ticks / COUNTS_PER_REV) / dtm;    //calculate wheel's speed (RPM)
//      prev_update_time_ = current_time;
        prev_encoder_ticks_ = getEncoderPosition();
    }

    //PID控制器必须使用中断调用，10ms执行一次
    void Incremental_PID(int target)
    { 
        updateSpd();
        if(Last_tar > target)       //微分缓冲器
          Last_tar-=2;
        else if(Last_tar < target)
          Last_tar+=2;
        Bias = rpm - Last_tar;          //计算增量偏差
        Pwm += kp * (Bias - Last_bias) + ki * Bias;       //增量PI控制器
        if(Pwm > MOTOR_MAX_PWM)  Pwm = MOTOR_MAX_PWM;   //输出限幅
        if(Pwm < -MOTOR_MAX_PWM) Pwm = -MOTOR_MAX_PWM;
        Last_bias=Bias;                             //保存上一次偏差 
        Out_Pwm *= 0.7;                             //一阶低通滤波器
        Out_Pwm += Last_Pwm * 0.3;
        Last_Pwm = Pwm;
        if(Out_Pwm < 6 && Out_Pwm > -6) Out_Pwm = 0;
        setSpd(Out_Pwm);
    }

    void Update_PID(float _kp, float _ki, float _kd)
    {
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }
};

#endif
