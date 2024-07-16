/*
 * @file FSM_State_BalanceStand.cpp
 * @brief 
 */

#include "FSM_State_BalanceStand.h"

template<typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND, "BALANCE_STAND") {
    this->turnOffAllSafetyChecks();

}

template<typename T>
void FSM_State_BalanceStand<T>::onEnter() {
    _progress = 0;
    freeze = false;
    mode = 0;

    // 规划四条固定的曲线
    for (int leg = 0; leg < 4; leg++) {
        qStand[leg] = this->_data->_legController->datas[leg].q;

        pStand[leg] = this->_data->_legController->datas[leg].p;
        pStand[leg][1] = this->_data->_quadruped->_abadLinkLength * this->_data->_quadruped->getSideSign(leg);
        pStandForward[leg] = pStand[leg];
        pStandBack[leg] = pStand[leg];
        pForward[leg] = pStand[leg];
        pBackward[leg] = pStand[leg];
        pSwingAir[leg] = pStand[leg];

        pStandForward[leg][0] += 0.02;
        pStandBack[leg][0] -= 0.02;
        pForward[leg][0] += 0.04;
        pBackward[leg][0] -= 0.04;
        pSwingAir[leg][2] += 0.08;

        // print all p
        printf("leg: %d pStand: %f %f %f\n", leg, pStand[leg][0], pStand[leg][1], pStand[leg][2]);
        printf("leg: %d pStandBack: %f %f %f\n", leg, pStandBack[leg][0], pStandBack[leg][1], pStandBack[leg][2]);
        printf("leg: %d pForward: %f %f %f\n", leg, pForward[leg][0], pForward[leg][1], pForward[leg][2]);
        printf("leg: %d pBackward: %f %f %f\n", leg, pBackward[leg][0], pBackward[leg][1], pBackward[leg][2]);
        printf("leg: %d pSwingAir: %f %f %f\n", leg, pSwingAir[leg][0], pSwingAir[leg][1], pSwingAir[leg][2]);

        auto qStand0 = JointSwingTrajectory<T>::inverseKinematics(pStand[leg], leg);
        auto qStandBack = JointSwingTrajectory<T>::inverseKinematics(pStandBack[leg], leg);
        auto qForward = JointSwingTrajectory<T>::inverseKinematics(pForward[leg], leg);
        auto qBackward = JointSwingTrajectory<T>::inverseKinematics(pBackward[leg], leg);
        auto qSwingAir = JointSwingTrajectory<T>::inverseKinematics(pSwingAir[leg], leg);

        // print
        printf("leg: %d qStand: %f %f %f\n", leg, qStand0[0], qStand0[1], qStand0[2]);
        printf("leg: %d qStandBack: %f %f %f\n", leg, qStandBack[0], qStandBack[1], qStandBack[2]);
        printf("leg: %d qForward: %f %f %f\n", leg, qForward[0], qForward[1], qForward[2]);
        printf("leg: %d qBackward: %f %f %f\n", leg, qBackward[0], qBackward[1], qBackward[2]);
        printf("leg: %d qSwingAir: %f %f %f\n", leg, qSwingAir[0], qSwingAir[1], qSwingAir[2]);

        /*************** stand mode ***************/
        _firstStandSwing[leg].setInitialJointPosition(pStand[leg], leg);
        _firstStandSwing[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _firstStandSwing[leg].setFinalJointPosition(pStandForward[leg], leg);

        _firstStandStance[leg].setInitialJointPosition(pStand[leg], leg);
        _firstStandStance[leg].setFinalJointPosition(pStandBack[leg], leg);

        _StandSwing[leg].setInitialJointPosition(pStandBack[leg], leg);
        _StandSwing[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _StandSwing[leg].setFinalJointPosition(pStandForward[leg], leg);

        _StandStance[leg].setInitialJointPosition(pStandForward[leg], leg);
        _StandStance[leg].setMiddleJointPosition(pStand[leg], leg);
        _StandStance[leg].setFinalJointPosition(pStandBack[leg], leg);

        /*************** forward mode ***************/
        _firstForwardSwing[leg].setInitialJointPosition(pStand[leg], leg);
        _firstForwardSwing[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _firstForwardSwing[leg].setFinalJointPosition(pForward[leg], leg);

        _firstForwardStance[leg].setInitialJointPosition(pStand[leg], leg);
        _firstForwardStance[leg].setFinalJointPosition(pBackward[leg], leg);

        _forwardSwingTraj[leg].setInitialJointPosition(pBackward[leg], leg);
        _forwardSwingTraj[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _forwardSwingTraj[leg].setFinalJointPosition(pForward[leg], leg);

        _forwardStanceTraj[leg].setInitialJointPosition(pForward[leg], leg);
        _forwardStanceTraj[leg].setMiddleJointPosition(pStand[leg], leg);
        _forwardStanceTraj[leg].setFinalJointPosition(pBackward[leg], leg);

        /*************** backward mode ***************/
        _firstBackwardSwing[leg].setInitialJointPosition(pStand[leg], leg);
        _firstBackwardSwing[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _firstBackwardSwing[leg].setFinalJointPosition(pBackward[leg], leg);

        _firstBackwardStance[leg].setInitialJointPosition(pStand[leg], leg);
        _firstBackwardStance[leg].setFinalJointPosition(pForward[leg], leg);

        _backwardSwingTraj[leg].setInitialJointPosition(pForward[leg], leg);
        _backwardSwingTraj[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _backwardSwingTraj[leg].setFinalJointPosition(pBackward[leg], leg);

        _backwardStanceTraj[leg].setInitialJointPosition(pBackward[leg], leg);
        _backwardStanceTraj[leg].setMiddleJointPosition(pStand[leg], leg);
        _backwardStanceTraj[leg].setFinalJointPosition(pForward[leg], leg);
    }
}

template<typename T>
void FSM_State_BalanceStand<T>::setupCommand() {
    // 原地不动
    auto cmd = this->_data->_desiredStateCommand;

    if (cmd->leftAnalogStick[1] > 0.05f) {    // move forward
        mode = mode::FORWARD;
        printf("move forward\n");
    } else if (cmd->leftAnalogStick[1] < -0.05f) {
        mode = mode::BACKWARD;
    } else if (cmd->rightAnalogStick[0] > 0.05f) {
        mode = mode::LEFT;
    } else if (cmd->rightAnalogStick[0] < -0.05f) {
        mode = mode::RIGHT;
    } else {
        mode = mode::STAND;
    }
}


template<typename T>
void FSM_State_BalanceStand<T>::run() {
    setupCommand();
    float phase = 5 * _progress * this->_data->controlParameters->controller_dt;
    float swingTime = 2.f;
    if (freeze) {
        for (int leg = 0; leg < 4; leg++) {
            this->_data->_legController->commands[leg].qDes = qStand[leg];
            this->_data->_legController->commands[leg].qdDes = Vec3<T>(0., 0., 0.);
            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.8, 1.8, 1.8).asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.016, 0.016, 0.016).asDiagonal();
        }
    } else {
        for (int leg = 0; leg < 4; leg++) {
            if (swingState[leg]) {
                if (leg == 1 || leg == 2) {
                    firstMove[leg] = false;
                }
                if (firstMove[leg]) {
//                _firstStepSwing[leg].computeSwingTrajectoryBezier(phase, swingTime);
//                SwingPtr[leg] = &_firstStepSwing[leg];
                    switch (mode) {
                        case mode::STAND:
//                            printf("first swing leg:%d phase:%f\n", leg, phase);
                            _StandSwing[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            SwingPtr[leg] = &_StandSwing[leg];
                            break;
                        case mode::FORWARD:
                            _firstForwardSwing[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            SwingPtr[leg] = &_firstForwardSwing[leg];
                            break;
                        case mode::BACKWARD:
                            _firstBackwardSwing[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            SwingPtr[leg] = &_firstBackwardSwing[leg];
                            break;
                    }
                } else {
                    switch (mode) {
                        case mode::STAND:
                            _StandSwing[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            SwingPtr[leg] = &_StandSwing[leg];
                            break;
                        case mode::FORWARD:
                            _forwardSwingTraj[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            SwingPtr[leg] = &_forwardSwingTraj[leg];
                            break;
                        case mode::BACKWARD:
                            _backwardSwingTraj[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            SwingPtr[leg] = &_backwardSwingTraj[leg];
                            break;
                    }
                }

                // print JointPosition
//                printf("swing leg: %d, qDes: %f, %f, %f\n", leg, SwingPtr[leg]->getJointPosition()[0],
//                       SwingPtr[leg]->getJointPosition()[1], SwingPtr[leg]->getJointPosition()[2]);
                this->_data->_legController->commands[leg].qDes = SwingPtr[leg]->getJointPosition();
                this->_data->_legController->commands[leg].qDes[0] = 0.f;
                this->_data->_legController->commands[leg].qdDes = SwingPtr[leg]->getJointVelocity();
                this->_data->_legController->commands[leg].qdDes[0] = 0.f;
                if(leg == 0){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(0.8, 0.8, 0.8).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.006, 0.006, 0.006).asDiagonal();
                } else if(leg == 1){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.6, 1.6, 1.6).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.014, 0.014, 0.014).asDiagonal();
                } else if(leg == 2){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.4, 1.4, 1.4).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.012, 0.012, 0.012).asDiagonal();
                } else if(leg == 3){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.2, 1.2, 1.2).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.010, 0.010, 0.010).asDiagonal();
                }

            } else {
                if (leg == 0 || leg == 3) {
                    firstMove[leg] = false;
                }
                if (firstMove[leg]) {
                    switch (mode) {
                        case mode::STAND:
                            _StandStance[leg].computeSwingTrajectoryBezierWithoutMid(phase, swingTime);
                            StancePtr[leg] = &_StandStance[leg];
                            break;
                        case mode::FORWARD:
                            _firstForwardStance[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            StancePtr[leg] = &_firstForwardStance[leg];
                            break;
                        case mode::BACKWARD:
                            _firstBackwardStance[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            StancePtr[leg] = &_firstBackwardStance[leg];
                            break;
                    }
                } else {
                    switch (mode) {
                        case mode::STAND:
                            _StandStance[leg].computeSwingTrajectoryBezierWithoutMid(phase, swingTime);
                            StancePtr[leg] = &_StandStance[leg];
                            break;
                        case mode::FORWARD:
                            _forwardStanceTraj[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            StancePtr[leg] = &_forwardStanceTraj[leg];
                            break;
                        case mode::BACKWARD:
                            _backwardStanceTraj[leg].computeSwingTrajectoryBezier(phase, swingTime);
                            StancePtr[leg] = &_backwardStanceTraj[leg];
                            break;
                    }
                }
                this->_data->_legController->commands[leg].qDes = StancePtr[leg]->getJointPosition();
                this->_data->_legController->commands[leg].qDes[0] = 0.f;
                this->_data->_legController->commands[leg].qdDes = StancePtr[leg]->getJointVelocity();
                this->_data->_legController->commands[leg].qdDes[0] = 0.f;

                if(leg == 0){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.6, 1.6, 1.6).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.014, 0.014, 0.014).asDiagonal();
                } else if(leg == 1){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.6, 1.6, 1.6).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.014, 0.014, 0.014).asDiagonal();
                } else if(leg == 2){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.6, 1.6, 1.6).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.014, 0.014, 0.014).asDiagonal();
                } else if(leg == 3){
                    this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.6, 1.6, 1.6).asDiagonal();
                    this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.014, 0.014, 0.014).asDiagonal();
                }


            }
        }
    }

    if (_progress >= 100) {
        _progress = 0;
        if (this->_data->_desiredStateCommand->beiTong->x) {
            printf("freeze\n");
            freeze = true;
        } else {
            for (int i = 0; i < 4; i++)
                swingState[i] = !swingState[i];
        }
    } else {
        _progress++;
    }

}

//
//template<typename T>
//void FSM_State_BalanceStand<T>::run() {
//
//    float phase = 5 * _progress * this->_data->controlParameters->controller_dt;
//
//    for (int leg = 0; leg < 4; leg++) {
//        if (swingState[leg]) { // leg is in swing
//            _jointSwingTrajectories[leg].computeSwingTrajectoryBezier(phase, 0.3f);
//            this->_data->_legController->commands[leg].qDes = _jointSwingTrajectories[leg].getJointPosition();
//            this->_data->_legController->commands[leg].qdDes = _jointSwingTrajectories[leg].getJointVelocity();
//            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(2.2, 2.2, 2.2).asDiagonal();
//            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.022, 0.022, 0.022).asDiagonal();
//            printf("leg: %d, swingState: %d, qDes: %f, %f, %f\n", leg, swingState[leg],
//                   this->_data->_legController->commands[leg].qDes[0],
//                   this->_data->_legController->commands[leg].qDes[1],
//                   this->_data->_legController->commands[leg].qDes[2]);
//        } else {    // leg is in stance
//            this->_data->_legController->commands[leg].qDes = standJointPos[leg];
//            this->_data->_legController->commands[leg].qdDes = Vec3<T>(0.f, 0.f, 0.f);
//            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(3.2, 3.2, 3.2).asDiagonal();
//            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.030, 0.030, 0.030).asDiagonal();
//            printf("leg: %d, swingState: %d, qDes: %f, %f, %f\n", leg, swingState[leg],
//                   this->_data->_legController->commands[leg].qDes[0],
//                   this->_data->_legController->commands[leg].qDes[1],
//                   this->_data->_legController->commands[leg].qDes[2]);
//        }
//    }
//
//    if (_progress >= 100) {
//        _progress = 0;
//        for (int i = 0; i < 4; i++)
//            swingState[i] = !swingState[i];
//
////        for (int i = 0; i < 4; i++) {
////            swingState[i] = 0;
////        }
////        switch (count % 4) {
////            case 0:
////                swingState[0] = 1;
////                break;
////            case 1:
////                swingState[2] = 1;
////                break;
////            case 2:
////                swingState[1] = 1;
////                break;
////            case 3:
////                swingState[3] = 1;
////                break;
////            default:
////                break;
////        }
//////        swingState[count % 4] = 1;
////        count++;
//    } else {
//        _progress++;
//    }
//}

template<typename T>
bool FSM_State_BalanceStand<T>::isBusy() {
    return false;
}

template
class FSM_State_BalanceStand<float>;