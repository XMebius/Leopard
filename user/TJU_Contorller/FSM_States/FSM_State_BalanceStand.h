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
    // Keep track of the control iterations
    int _iter = 0;
    int _progress = 0;
    Vec3<T> pStand[4], pStandBack[4], pForward[4], pBackward[4], pSwingAir[4];
    JointSwingTrajectory<T> _firstStepSwing[4], _firstStepStance[4], _SwingTraj[4], _StanceTraj[4];
    bool firstMove[4] = {true, true, true, true};

    int swingState[4] = {1, 0, 0, 1};


//private:
//
//    int _progress = 0;
//    Vec3<T> standJointPos[4];
//
//    JointSwingTrajectory<T> _jointSwingTrajectories[4];
//    int swingState[4] = {1, 0, 0, 1};
};


#endif //LEOPARD_FSM_STATE_BALANCESTAND_H
