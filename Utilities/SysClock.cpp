#include "SysClock.hpp"

void Exodus::SysClock::initialize() {
    clockFrequency = SysCtlClockGet();
}

void Exodus::SysClock::delayBy1s(int count) {
    SysCtlDelay(clockFrequency/6*count);
}

void Exodus::SysClock::delayBy1ms(int count) {
    SysCtlDelay(clockFrequency/6000*count);
}

void Exodus::SysClock::delayBy1us(int count) {
    SysCtlDelay(clockFrequency/6000000*count);
}
