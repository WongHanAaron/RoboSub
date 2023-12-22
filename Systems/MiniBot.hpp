/**
 * The MiniBot class interacts with the MiniBot.
 * The MiniBot is a smaller scaled-down version of the submarine
 * which will be used for testing purposes.
 */
#ifndef SYSTEMS_MINIBOT_HPP_
#define SYSTEMS_MINIBOT_HPP_

#include "Definitions.hpp"
#include "PWM.hpp"
#include "JetsonLink.hpp"
#include "SystemDriver.hpp"

namespace Exodus {

class MiniBot : public SystemDriver {

public:
    void initialize(JetsonLink *tilink, PWMPinOptions rightForward, PWMPinOptions rightBackward, PWMPinOptions leftForward, PWMPinOptions leftBackward);
    void setRightWidth(bool isForward, double percentage);
    void setLeftWidth(bool isForward, double percentage);
    void setRightWidth(uint32_t width);
    void setLeftWidth(uint32_t width);
    uint32_t getRightWidth();
    uint32_t getLeftWidth();

private:
    PWMPin rightMotorForward;
    PWMPin rightMotorBackward;
    PWMPin leftMotorForward;
    PWMPin leftMotorBackward;

    uint32_t leftWidth;
    uint32_t rightWidth;
    JetsonLink* link;

    uint32_t calculateWidth(double percentage);
};
}
#endif /* SYSTEMS_MINIBOT_HPP_ */
