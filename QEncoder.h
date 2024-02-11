/* Qencoder Version 2.0 */ 
// 4fold Encoder code
#pragma once
#include <mbed.h>

class QEncoder {
    public:
    uint8_t _previousState;
    volatile int32_t _count;
    volatile uint32_t _errorCount;
    BusIn _bi;
        // 멤버변수 초기화 목록 설정
        QEncoder(PinName pinA, PinName pinB): _pinA(pinA), _pinB(pinB),_bi(pinA,pinB){}
    void init()    {  // callbacks for all edges
        _pinA.rise(callback(this, &QEncoder::decode)); 
        _pinA.fall(callback(this, &QEncoder::decode));
        _pinB.rise(callback(this, &QEncoder::decode));
        _pinB.fall(callback(this, &QEncoder::decode));
        _previousState = _bi;
        _count = 0;
        _errorCount=0;
    }

    void decode()    {
        uint8_t newState = _bi[0].read() +2*_bi[1].read();
        //uint8_t newState = _bi; // busin instance를 그대로 사용할 경우 주소 또는 쓰레기가 전달됨  -> (busin.read()로해도 안됬음...)
        switch((_previousState << 2) | newState) {
            case 0b0001: // 0x01 PREV|CURR
            case 0b0111: // 0x07
            case 0b1110: // 0x0E
            case 0b1000: // 0x08
                _count --; break;
            case 0b0010: // 0x02
            case 0b1011: // 0x0B
            case 0b1101: // 0x0D
            case 0b0100: // 0x04
                _count ++; break;
            default:
                _errorCount++; break;
        } 
        _previousState = newState; 
    } 

    int32_t getCount()    {
        return _count;
    }
    void setCount(int32_t EncoderVal){
        _pinA.disable_irq (); _pinB.disable_irq ();
        _count = EncoderVal;
        _pinA.enable_irq ();_pinB.enable_irq ();
    }
    int32_t getErrorCount()    {
        return _errorCount;
    }     
  private:
    InterruptIn _pinA;
    InterruptIn _pinB;
    
    
    
};