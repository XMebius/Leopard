/*
 * @file FSM_State_BalanceStand.h
 * @brief 
 */

#ifndef LEOPARD_FSM_STATE_BALANCESTAND_H
#define LEOPARD_FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"
#include "Controllers/JointSwingTrajectory.h"

enum mode {
    STAND = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4,
};


template<typename T>
class FSM_State_BalanceStand : public FSM_State<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);

    void onEnter();

    void run();

    void setupCommand();

    bool isBusy();

private:
    // Keep track of the control iterations
    int _iter = 0;
    int _progress = 0;
    int mode = 0;
    Vec3<T> qStand[4];
    Vec3<T> pStand[4], pStandForward[4], pStandBack[4], pForward[4], pBackward[4], pSwingAir[4];
    JointSwingTrajectory<T> _firstStandSwing[4], _firstStandStance[4] ,_StandSwing[4], _StandStance[4];
    JointSwingTrajectory<T> _firstForwardSwing[4], _firstForwardStance[4], _forwardSwingTraj[4], _forwardStanceTraj[4];
    JointSwingTrajectory<T> _firstBackwardSwing[4], _firstBackwardStance[4], _backwardSwingTraj[4], _backwardStanceTraj[4];
    JointSwingTrajectory<T>* SwingPtr[4];
    JointSwingTrajectory<T>* StancePtr[4];

    bool firstMove[4] = {true, true, true, true};

    int swingState[4] = {1, 0, 0, 1};

    bool freeze = false;

//private:
//
//    int _progress = 0;
//    Vec3<T> standJointPos[4];
//
//    JointSwingTrajectory<T> _jointSwingTrajectories[4];
//    int swingState[4] = {1, 0, 0, 1};
};


#endif //LEOPARD_FSM_STATE_BALANCESTAND_H
