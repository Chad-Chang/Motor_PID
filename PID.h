#ifndef PIDBASIC_h
#define PIDBASIC_h
#include "mbed.h"

class PIDBasic{
    public:
        
        PIDBasic(float kp = 0., float ki = 0.,float kd = 0. ,float imax = 0):
        kp_(kp), ki_(ki), kd_(kd), imax_(fabs(imax))
            {
                I_term_=0.0f, last_y_=0.0f;
            }
        void operator()(float kp= 0., float ki = 0., float kd = 0.,float imax =0.)
            {
                kp_ = kp; ki_ = ki, kd_ = kd, imax = fabs(imax);
            }
        float constrain(float x, float x_min, float x_max)
            {
                return (x>x_max)?x_max:(x<x_min)? x_min:x;
                /* 3항 연산자 : x>x_max가 참이면 x_max를 넣고, 
                            거짓이면 (x<x_min)?x_min:x 실행
                            1) x< x_min=1 이면 x = x_min
                            2) x< x_min=0 이면 x = x 대입
                */
            }
        float computePID(float r, float y, float dt)
            {
                float error = r-y;
                float P_term_ = kp_*error;
                I_term_ = I_term_+ ki_*error*dt;
                I_term_ = constrain(I_term_, -imax_,imax_);
                float D_term_ = -kd_*(y-last_y_)/dt;
                last_y_ = y;
                return P_term_ + I_term_ +D_term_;
            }
        void SetNewGain(float kp, float ki, float kd)
        {
            kp_ = kp;  ki_ = ki;  kd_ = kd; 
        }
        void GetPID(float &kp, float &ki, float &kd){
            kp = kp_; ki = ki_; kd = kd_;

        }
        
    private:
        float kp_, ki_, kd_;
        float imax_;
        float I_term_;
        float last_y_;
};

#endif