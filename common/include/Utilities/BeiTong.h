//
// Created by mebius on 24-5-9.
//

#ifndef LEOPARD_BEITONG_H
#define LEOPARD_BEITONG_H

#include "cppTypes.h"

struct BeiTong {
    /*
     * call zero command when gamepad is built
     */
    BeiTong() { zero(); }

    bool leftBumper, rightBumper, leftTriggerButton, rightTriggerButton ,
            start, back, a, b, x, y;
    Vec2<float> leftStick, rightStick;

    void zero() {
        leftBumper = false;
        rightBumper = false;
        leftTriggerButton = false;
        rightTriggerButton = false;
        start = false;
        back = false;
        a = false;
        b = false;
        x = false;
        y = false;
        leftStick = Vec2<float>::Zero();
        rightStick = Vec2<float>::Zero();
    };

    void printBeiTong() const {
        std::cout << "leftBumper: " << leftBumper << std::endl;
        std::cout << "rightBumper: " << rightBumper << std::endl;
        std::cout << "leftTriggerButton: " << leftTriggerButton << std::endl;
        std::cout << "rightTriggerButton: " << rightTriggerButton << std::endl;
        std::cout << "start: " << start << std::endl;
        std::cout << "back: " << back << std::endl;
        std::cout << "a: " << a << std::endl;
        std::cout << "b: " << b << std::endl;
        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "leftStick_x: " << leftStick[0] << std::endl;
        std::cout << "leftStick_y: " << leftStick[1] << std::endl;
        std::cout << "rightStick_x: " << rightStick[0] << std::endl;
        std::cout << "rightStick_y: " << rightStick[1] << std::endl;
    }
};

#endif //LEOPARD_BEITONG_H
