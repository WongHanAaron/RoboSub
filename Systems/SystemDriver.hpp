/**
 * The system driver library is used in by all objects in the System/Sensor folders
 * to make it easer to integrate into the system. This is the parent class of all drivers
 * in this project.
 */
#ifndef SYSTEM_DRIVER_HPP_
#define SYSTEM_DRIVER_HPP_

namespace Exodus {

class SystemDriver {

public:
    SystemDriver() : triggerFlag(false), running(false) {}
    ~SystemDriver() {}
    bool isTriggered() { return triggerFlag; }
    void setTriggerFlag(bool triggerFlag) { this -> triggerFlag = triggerFlag;}
    bool isRunning() { return running; }
    void setDeviceRunning(bool deviceRunning) { this -> running = deviceRunning;}


protected:
    bool triggerFlag;
    bool running;
};

}

#endif /* UTILITIES_SYSTEMDRIVER_HPP_ */
