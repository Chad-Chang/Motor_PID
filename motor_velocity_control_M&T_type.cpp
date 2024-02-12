#include "RawSerial.h"
#include "Timer.h"
#include "mbed.h"
#include "encoder_names.h"
#include "PID.h"
#include "QEncoder.h"
#include "L298LIB.h"
#include <cstdlib>
#include "math.h"
#define EN D10
#define IN1 D8
#define IN2 D9
#define DEG_TO_RAD (4.*atan(1.)/180.)
#define SAMPLING_INTERVAL 1000.f // us 단위로 표현 => 1ms의 루프타임 
#define PRINT_INTERVAL 10000.f // us 단위로 표현 =>10ms의 루프타임
#define DT (SAMPLING_INTERVAL/1000000.f) // 1ms의 루프타임
float Kp =3.0f;float Ki =0.0f;float Kd =0.1f; float I_max = 100.0f;
PIDBasic pid(Kp,Ki, Kd,I_max); // kp, ki, kd, imax
L298LIB motor(EN,IN1,IN2);
QEncoder  enco(D2,D3);
RawSerial pc(USBTX,USBRX,230400);
Timer tmr,tmr2,tmr3, tmr_prt;
float r= 0.0f, y = 0.0f;        // referenc input, output
float u;                        //control effor 
volatile bool gotPacket = false;
volatile float data[4];
bool pid_flag = false;
bool refer_flag = false;
float y_prev= 0.f; // 현재 엔코더와 이전 엔코더의 차이를 구하기 위해
float y_rpm = 0.f;
void serialEvent()
{   
    static char serialInBuffer[32];
    static int data_cnt= 0, buff_cnt = 0;
    if(pc.readable())
    {   
        char byteIn = pc.getc();

        // flag 설정
        if(byteIn == 'a')
        { 
            pid_flag = true;// gain 튜닝
            refer_flag = false;
        }
        else if(byteIn == 'b')
        {
            refer_flag = true;
            pid_flag = false;
        }

        // serial packet
        if(byteIn == ',')
        {
            serialInBuffer[buff_cnt] = '\0';
            data[data_cnt++] = atof(serialInBuffer); // string을 float으로 변환
            buff_cnt = 0;
        }
        else if(byteIn == '\n')
        {
            serialInBuffer[buff_cnt] = '\0';
            data[data_cnt] = atof(serialInBuffer);
            buff_cnt =0 ; data_cnt = 0;
            gotPacket = true;
        }
        else
        {
            serialInBuffer[buff_cnt++] = byteIn;
        }
    }
}

//             /* M&T type velocity*/
uint32_t M_s = 30;
float pulse = 0.f;
float pulse_prev = 0.f;
float T_s;
bool is_T = false; bool is_M = true;
float pulse_x;
int main()
{ 
    float kp_prt; float ki_prt;float kd_prt;
    pid.GetPID(kp_prt, ki_prt, kd_prt);
    motor.setSpeed(0);
    tmr.start(); tmr_prt.start();
    enco.init();
    enco.setCount(0);
    pc.attach(&serialEvent);
    y_prev = enco.getCount()*Encoder::ratio;
    pulse = enco.getCount();
    while (1) 
    {
        
        if(gotPacket)
        {
            gotPacket = false;
            if(pid_flag)
            {
                Kp = data[1];Ki = data[2];Kd = data[3];
                pid.SetNewGain(Kp,Ki, Kd);
                pid.GetPID(kp_prt, ki_prt, kd_prt);
                pid_flag = false;
            }
            else if(refer_flag)
            {
                refer_flag = false;
                r = data[1]; 
            }
        }
        if(is_T)
        {
            tmr2.start();
            if(abs(pulse-pulse_prev) > M_s)
            {   
                is_T = false; is_M = true;
                float delta = tmr2.read_us();
                tmr2.reset();
                pulse_x += M_s;
                y_rpm = pulse_x*Encoder::ratio/360*(60*1000000/(SAMPLING_INTERVAL+delta)); // 1000000/(SAMPLING_INTERVAL+delta) : 마이크로초 -> 초단위
                pulse_prev = pulse;
                pc.printf("y_rpm_T:%f, DT : %f, delta : %f\n", y_rpm,DT,delta/1000000.);
            }
        }

        if(is_M)
        {   tmr3.start();
            if(tmr3.read_us()>SAMPLING_INTERVAL)
            {
                tmr3.reset();   
                is_T = true;is_M = false;
                pulse_x = (pulse - pulse_prev);
                y_rpm = (pulse_x)*Encoder::ratio/360 * (60/DT);
                
                pulse_prev = pulse;
                
            }
        }

        if(tmr.read_us() > SAMPLING_INTERVAL)
        {   
            tmr.reset();   
            pulse_x = (pulse - pulse_prev);
            y_rpm = (pulse_x)*Encoder::ratio/360 * (60/DT);
            u = pid.computePID(r, y_rpm, DT);
            motor.run(u/255.f);
            pulse_prev = pulse; 
        }
        if(tmr_prt.read_us()>SAMPLING_INTERVAL)
        {
            tmr_prt.reset();
            // pc.printf("y_rpm_M:%f, Dpulse : %d, pulse : %d, pulse_prev : %d\n", y_rpm, pulse_x, pulse,pulse_prev);
            pc.printf("ref_rpm %.1f, output : %.1f,error : %.1f, kp : %.2f, ki : %.2f, kd : %.2f\n", r,y_rpm, u/255.f,kp_prt, ki_prt, kd_prt);
            
            //plotting 용
            // pc.printf("ref_rpm %.1f,output : %.1f\n",r,y_rpm);
            // pc.printf("%.1f,%.1f\n",r,y_rpm);
        }
    }
}
