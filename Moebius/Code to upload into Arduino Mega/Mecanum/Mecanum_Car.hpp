#include "onboard_Mega2560.h"

typedef DC_Motor Motor_Class;

class Mecanum_Car
{
public:
    int Now_spd;
    Mecanum_Car(Motor_Class *_LF_Wheel, Motor_Class *_RF_Wheel, Motor_Class *_LR_Wheel, Motor_Class *_RR_Wheel)
    {
        this->LF_Wheel = _LF_Wheel;
        this->RF_Wheel = _RF_Wheel;
        this->LR_Wheel = _LR_Wheel;
        this->RR_Wheel = _RR_Wheel;
    }
    void Init(void)
    {
        LF_Wheel->Init(1); 
        RF_Wheel->Init(1); 
        LR_Wheel->Init(1); 
        RR_Wheel->Init(1); //初始化电机PWM
    }
    void SetSpd(int Spd)
    {
        LF_Wheel_Spd = RF_Wheel_Spd = LR_Wheel_Spd = RR_Wheel_Spd = constrain(Spd, -MAX_RPM, MAX_RPM);  
    }

    struct input
    {
      float LF;
      float RF;
      float LR;
      float RR;
    };

    struct input ROS_MoveBase(float Line_vel, float Pan_vel, float Angle_vel)
    {
        Line_vel *= cos(45);
        Pan_vel *= sin(45);
        // LF_Wheel_Spd = constrain((Line_vel - Pan_vel + Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        // RF_Wheel_Spd = constrain((Line_vel + Pan_vel + Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        // LR_Wheel_Spd = constrain((Line_vel - Pan_vel - Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        // RR_Wheel_Spd = constrain((Line_vel + Pan_vel - Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        LF_Wheel_Spd = constrain((Line_vel - Pan_vel - Angle_vel * 0.25) * -30, -MAX_RPM, MAX_RPM);
        RF_Wheel_Spd = constrain((Line_vel + Pan_vel + Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        LR_Wheel_Spd = constrain((Line_vel + Pan_vel - Angle_vel * 0.25) * -30, -MAX_RPM, MAX_RPM);
        RR_Wheel_Spd = constrain((Line_vel - Pan_vel + Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);

        // struct input input_speed;
        // input_speed.LF=LF_Wheel_Spd;
        // input_speed.RF=RF_Wheel_Spd;
        // input_speed.LR=LR_Wheel_Spd;
        // input_speed.RR=RR_Wheel_Spd;
        // return input_speed;
    }

    void ClearOdom()
    {
         LF_Wheel->clrEncoderPosition(); //RF in actual
         RF_Wheel->clrEncoderPosition(); //LF in actual
         LR_Wheel->clrEncoderPosition(); ///RR in actaul
         RR_Wheel->clrEncoderPosition(); //LR in actual
    }
    
    void ReadOdom()
    {
        Serial.print(LF_Wheel->getEncoderPosition());
        Serial.print(", ");
        Serial.print(RF_Wheel->getEncoderPosition());
        Serial.print(", ");
        Serial.print(LR_Wheel->getEncoderPosition());
        Serial.print(", ");
        Serial.print(RR_Wheel->getEncoderPosition());
        Serial.print("\n ");

        // Serial.print(RF_Wheel->getMotorRPM());
        // Serial.println("\n ");
        
        // LR_Wheel->PrintPortConfig();
        // Serial.print(LR_Wheel->setSpd(5));

        // Serial.println(RF_Wheel->getState());
        // Serial.println(RR_Wheel->getState());
        
        // uint8_t a,b=RF_Wheel->getBitmask();   
        // Serial.println(a);  
        // Serial.println(b);   
        
        // int V1,V2=RR_Wheel->getVoltage();
        // Serial.println(V1);  
        // Serial.println(V2); 

        // Serial.print(LF_Wheel->getSignal1());  
        // Serial.print(",");
        // Serial.print(RF_Wheel->getSignal1());  
        // Serial.print(",");
        // Serial.print(LR_Wheel->getSignal1());  
        // Serial.print(",");
        // Serial.print(RR_Wheel->getSignal1());          
        // Serial.print("\n");
        // Serial.print(LR_Wheel->getMotorSpd());  
        // Serial.print("\n");
        // Serial.print(RF_Wheel->getMotorDirection());  
        // Serial.print("\n");


    }

    
    void Increment_PID(void)
    {
        LF_Wheel->Incremental_PID(LF_Wheel_Spd);
        RF_Wheel->Incremental_PID(RF_Wheel_Spd);
        LR_Wheel->Incremental_PID(LR_Wheel_Spd);
        RR_Wheel->Incremental_PID(RR_Wheel_Spd);
                
    }
    void Update_PID(float _kp, float _ki, float _kd)
    {
          LF_Wheel->Update_PID(_kp, _ki, _kd);
          RF_Wheel->Update_PID(_kp, _ki, _kd);
          LR_Wheel->Update_PID(_kp, _ki, _kd);
          RR_Wheel->Update_PID(_kp, _ki, _kd);
    }

private:
    Motor_Class *LF_Wheel;
    Motor_Class *RF_Wheel;
    Motor_Class *LR_Wheel;
    Motor_Class *RR_Wheel;
    
    int LR_Wheel_Spd;
    int RR_Wheel_Spd;
    int RF_Wheel_Spd;
    int LF_Wheel_Spd;
};
