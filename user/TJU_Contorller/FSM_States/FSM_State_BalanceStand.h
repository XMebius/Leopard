/*
 * @file FSM_State_BalanceStand.h
 * @brief 
 */

#ifndef LEOPARD_FSM_STATE_BALANCESTAND_H
#define LEOPARD_FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"
#include "Controllers/JointSwingTrajectory.h"

template<typename T>
class FSM_State_BalanceStand : public FSM_State<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);

    void onEnter();

    void run();

    bool isBusy();

private:

    int _progress = 0;
    Vec3<T> standJointPos[4];

    JointSwingTrajectory<T> _jointSwingTrajectories[4];
    int swingState[4] = {1, 0, 0, 1};
    bool enterSwing[4] = {true, true, true, true};
    bool enterStand[4] = {true, true, true, true};

    int count = 0;

    Vec3<T> forward_stance_ini_angle;
    Vec3<T> forward_stance_final_angle;
    Vec3<T> forward_swing_mid;

    Vec3<T> stance_ini;
    Vec3<T> stance_fin;
    Vec3<T> swing_ini;
    Vec3<T> swing_fin;
    Vec3<T> swing_mid;
};


#endif //LEOPARD_FSM_STATE_BALANCESTAND_H
