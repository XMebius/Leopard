//
// Created by mebius on 24-4-23.
//

#ifndef LEOPARD_TJU_CONTROLLER_HPP
#define LEOPARD_TJU_CONTROLLER_HPP

#include <RobotController.h>
#include "TJU_UserParameters.hpp"
#include "FSM_States/ControlFSM.h"

class TJU_Controller : public RobotController {
public:
    TJU_Controller();
    virtual ~TJU_Controller() {}

    // abstract base func
    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
        // 被HardwareBridge的析构函数调用
        return &userParameters;
    }


private:
    int iter = 0;

    ControlFSM<float>* _controlFSM;
    TJU_UserParameters userParameters;

};


#endif //LEOPARD_TJU_CONTROLLER_HPP
