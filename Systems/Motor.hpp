/**
 * The Motor library operates the submarine motors over PWM operation
 */
#include "PWM.hpp"

namespace Exodus {

class Motor {

public:
    /**
     * intializes all motors to percentage 0 (PWM 1500)
     * @param: none
     * @return: none
     */
    void initialize(PWMPinOptions leftVertical, PWMPinOptions rightVertical, PWMPinOptions leftHorizontal, PWMPinOptions rightHorizontal);

    /**
     * intializes the left horizontal motor to the given percentage value
     * @param: percentage (given percentage to set motors to (0-100))
     * @return: none
     */
    void setLeftHorizontalWidth(bool isForward, double percentage);

    /**
     * intializes the left horizontal motor to the given percentage value
     * @param: percentage (given percentage to set motors to (0-100))
     * @return: none
     */
    void setRightHorizontalWidth(bool isForward, double percentage);

    /**
     * intializes the left horizontal motor to the given percentage value
     * @param: percentage (given percentage to set motors to (0-100))
     * @return: none
     */
    void setLeftVerticalWidth(bool isForward, double percentage);

    /**
     * intializes the left horizontal motor to the given percentage value
     * @param: percentage (given percentage to set motors to (0-100))
     * @return: none
     */
    void setRightVerticalWidth(bool isForward, double percentage);


    /**
     * Get the specified motor width
     * @params: none
     * @return: width relative to the PWM period
     */
    uint32_t getLeftHorizontalWidth();
    uint32_t getRightHorizontalWidth();
    uint32_t getLeftVerticalWidth();
    uint32_t getRightVerticalWidth();

    /**
     * Set the width of the specified motor
     * @params: Desired width to set relative to PWM period (width)
     * @return: none
     */
    void setLeftHorizontalWidth(uint32_t width);
    void setRightHorizontalWidth(uint32_t width);
    void setLeftVerticalWidth(uint32_t width);
    void setRightVerticalWidth(uint32_t width);

    /**
     * Set the width of all motors
     * @params: Desired width to set relative to PWM period (width)
     * @return: none
     */
    void setAllWidths(uint32_t width);

private:
    PWMPin LeftHorizontal;
    PWMPin RightHorizontal;
    PWMPin LeftVertical;
    PWMPin RightVertical;

    uint32_t calculateWidth(double percentage);
};
	
}




