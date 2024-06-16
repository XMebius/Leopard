/*
 * @file FSM_State_BalanceStand.h
 * @brief 
 */

#ifndef LEOPARD_FSM_STATE_BALANCESTAND_H
#define LEOPARD_FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

template<typename T>
class FSM_State_BalanceStand : public FSM_State<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);

    void onEnter();

    void run();

    bool isBusy();

private:

    void BalanceStandStep();

    WBC_Ctrl<T> * _wbc_ctrl;
    LocomotionCtrlData<T> * _wbc_data;

    T last_height_command = 0;

    Vec3<T> _ini_body_pos;
    Vec3<T> _ini_body_ori_rpy;
    T _body_weight;
};


#endif //LEOPARD_FSM_STATE_BALANCESTAND_H
