//
// Created by mebius on 24-5-13.
//

#ifndef LEOPARD_FSM_STATE_LOCOMOTION_H
#define LEOPARD_FSM_STATE_LOCOMOTION_H

#include "Controllers/convexMPC/ConvexMPCLocomotion.h"
#include "Controllers/WBC_Ctrl/WBC_Ctrl.hpp"
#include "Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"

//template<typename T> class WBC_Ctrl;
//template<typename T> class LocomotionCtrlData;

#include "FSM_State.h"

template<typename T>
class FSM_State_Locomotion :public FSM_State<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);

    void onEnter() override;

    void run() override;

    bool isBusy() override;
private:
    ConvexMPCLocomotion* cMPC;
    WBC_Ctrl<T>* _wbc_ctrl;
    LocomotionCtrlData<T>* _wbc_data;
};


#endif //LEOPARD_FSM_STATE_LOCOMOTION_H
