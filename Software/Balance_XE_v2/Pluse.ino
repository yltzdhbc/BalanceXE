/*
 * @Author: ylt
 * @Date: 2019-12-29 18:10:12
 * @LastEditors  : ylt
 * @LastEditTime : 2019-12-29 20:10:54
 * @FilePath: \BBOT\PWM.ino
 */
#include "core_esp8266_waveform.h"

/*为了产生脉冲信号给步进电机驱动器，重写了
**\esp8266\hardware\esp8266\2.6.3\cores\esp8266\core_esp8266_wiring_pwm.cpp
**去除了占空比变化，将占空比恒定在1/2减少系统处理时间 
**产生占空比为1/2的方波，一个周期即为一个脉冲
**频率范围为 100HZ 到 40,000HZ 
**要改变频率则重新调用该函数写入新的频率值即可*/
void Pluse_Generate(uint8_t pin, uint16_t freq)
{
    uint8_t analogScale = 2;
    uint8_t val = 1;
    uint16_t analogFreq = freq;
    uint32_t analogMap = 0;

    if (freq < 100)
        analogFreq = 100;
    else if (freq > 40000)
        analogFreq = 40000;
    else
        analogFreq = freq;

    if (pin > 16)
        return;

    uint32_t analogPeriod = 1000000L / analogFreq;

    analogMap &= ~(1 << pin);
    uint32_t high = (analogPeriod * val) / analogScale;
    uint32_t low = analogPeriod - high;
    pinMode(pin, OUTPUT);
    if (low == 0)
    {
        stopWaveform(pin);
        digitalWrite(pin, HIGH);
    }
    else if (high == 0)
    {
        stopWaveform(pin);
        digitalWrite(pin, LOW);
    }
    else
    {
        if (startWaveform(pin, high, low, 0))
        {
            analogMap |= (1 << pin);
        }
    }
}

void Pluse_Stop(uint8_t pin)
{
    stopWaveform(pin);
}