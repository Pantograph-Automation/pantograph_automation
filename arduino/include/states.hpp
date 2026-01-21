#pragma once

struct State
{
    State() {
        theta1 = 0;
        theta2 = 0;
        x = 0;
        y = 0;
    };

    /**
     * @brief The arm angles, in radians
     */
    float theta1;
    float theta2;

    /**
     * @brief The cartesian end effector point, in meters
     */
    float x;
    float y;

    /**
     * @brief If any values are still zero
     */
    bool isAnyZero() {
        
        if (x == 0 || y == 0 || theta1 == 0 || theta2 == 0) {
            return true;
        } else {
            return false;
        }
    }


};