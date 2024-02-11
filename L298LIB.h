#ifndef L298LIB_h
#define L298LIB_h
#include "mbed.h"

class L298LIB{
    public:
        L298LIB(PinName pinEnable, PinName pinIN1, PinName);
        void setSpeed(float pwmVal);
        float getSpeed();
        void forward();
        void backward();
        void stop();
        void run(float velocity);
        
    private:
        PwmOut pwm_;
        DigitalOut pinIN1_;
        DigitalOut pinIN2_;
};


L298LIB::L298LIB(PinName pinEnable, PinName pinIN1, PinName pinIN2):
            pwm_(pinEnable), pinIN1_(pinIN1), pinIN2_(pinIN2)
            {
                pwm_.period_us(25);  pwm_ = 0.; pinIN1_ = 0; pinIN2_=0;
            }
void L298LIB::setSpeed(float pwmVal)
{
    pwm_ = pwmVal;
}
float L298LIB::getSpeed()
{
    return pwm_;
}
void L298LIB::forward()
{
    pinIN1_ = 1; pinIN2_ = 0;
}
void L298LIB::backward()
{
    pinIN1_ = 0; pinIN2_ = 1; 
}
void L298LIB::stop()
{
    pinIN1_ = 0 ; pinIN2_= 0; 
}
void L298LIB::run(float vel)
{
    if(vel >= 0)
    {
        setSpeed((vel>1.0f)?1.0f:vel);
        forward();
    }
    else
    {
        setSpeed((vel<-1.0f) ? 1.0f : -vel);
        backward();
    }
}
#endif

