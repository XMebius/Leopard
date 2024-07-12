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

    // 规划四条固定的曲线
    for (int leg = 0; leg < 4; leg++) {
        pStand[leg] = this->_data->_legController->datas[leg].p;
        pStand[leg][1] = this->_data->_quadruped->_abadLinkLength * this->_data->_quadruped->getSideSign(leg);

        pStandBack[leg] = pStand[leg];
        pForward[leg] = pStand[leg];
        pBackward[leg] = pStand[leg];
        pSwingAir[leg] = pStand[leg];

        pStandBack[leg][0] += 0.005;
        pForward[leg][0] += 0.04;
        pBackward[leg][0] -= 0.04;
        pSwingAir[leg][2] += 0.08;

        // print all p
        printf("leg: %d pStand: %f %f %f\n", leg, pStand[leg][0], pStand[leg][1], pStand[leg][2]);
        printf("leg: %d pStandBack: %f %f %f\n", leg, pStandBack[leg][0], pStandBack[leg][1], pStandBack[leg][2]);
        printf("leg: %d pForward: %f %f %f\n", leg, pForward[leg][0], pForward[leg][1], pForward[leg][2]);
        printf("leg: %d pBackward: %f %f %f\n", leg, pBackward[leg][0], pBackward[leg][1], pBackward[leg][2]);
        printf("leg: %d pSwingAir: %f %f %f\n", leg, pSwingAir[leg][0], pSwingAir[leg][1], pSwingAir[leg][2]);

        auto qStand = JointSwingTrajectory<T>::inverseKinematics(pStand[leg], leg);
        auto qStandBack = JointSwingTrajectory<T>::inverseKinematics(pStandBack[leg], leg);
        auto qForward = JointSwingTrajectory<T>::inverseKinematics(pForward[leg], leg);
        auto qBackward = JointSwingTrajectory<T>::inverseKinematics(pBackward[leg], leg);
        auto qSwingAir = JointSwingTrajectory<T>::inverseKinematics(pSwingAir[leg], leg);

        // print
        printf("leg: %d qStand: %f %f %f\n", leg, qStand[0], qStand[1], qStand[2]);
        printf("leg: %d qStandBack: %f %f %f\n", leg, qStandBack[0], qStandBack[1], qStandBack[2]);
        printf("leg: %d qForward: %f %f %f\n", leg, qForward[0], qForward[1], qForward[2]);
        printf("leg: %d qBackward: %f %f %f\n", leg, qBackward[0], qBackward[1], qBackward[2]);
        printf("leg: %d qSwingAir: %f %f %f\n", leg, qSwingAir[0], qSwingAir[1], qSwingAir[2]);

        _firstStepSwing[leg].setInitialJointPosition(pStand[leg], leg);
        _firstStepSwing[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _firstStepSwing[leg].setFinalJointPosition(pForward[leg], leg);

        _firstStepStance[leg].setInitialJointPosition(pStand[leg], leg);
        _firstStepStance[leg].setFinalJointPosition(pBackward[leg], leg);

        _SwingTraj[leg].setInitialJointPosition(pBackward[leg], leg);
        _SwingTraj[leg].setMiddleJointPosition(pSwingAir[leg], leg);
        _SwingTraj[leg].setFinalJointPosition(pForward[leg], leg);

        _StanceTraj[leg].setInitialJointPosition(pForward[leg], leg);
        _StanceTraj[leg].setMiddleJointPosition(pStand[leg], leg);
        _StanceTraj[leg].setFinalJointPosition(pBackward[leg], leg);
    }

//    for (int leg(0); leg < 4; leg++) {
//        _jointSwingTrajectories[leg].setInitialJointPosition(this->_data->_legController->datas[leg].p, leg);
//        _jointSwingTrajectories[leg].setFinalJointPosition(this->_data->_legController->datas[leg].p, leg);
//        _jointSwingTrajectories[leg].setHeight(-0.19, leg);
//        standJointPos[leg] = this->_data->_legController->datas[leg].q;
//    }
}

template<typename T>
void FSM_State_BalanceStand<T>::run() {
    float phase = 5 * _progress * this->_data->controlParameters->controller_dt;

    for (int leg = 0; leg < 4; leg++) {
        if (swingState[leg]) {
            if (leg == 1 || leg == 2){
                firstMove[leg] = false;
            }
            if (firstMove[leg]) {
                _firstStepSwing[leg].computeSwingTrajectoryBezier(phase, 0.3f);
                printf("firstMove[%d] phase: %f q0: %f q1: %f q2: %f\n", leg, phase, _firstStepSwing[leg].getJointPosition()[0],
                       _firstStepSwing[leg].getJointPosition()[1], _firstStepSwing[leg].getJointPosition()[2]);
                this->_data->_legController->commands[leg].qDes = _firstStepSwing[leg].getJointPosition();
//                this->_data->_legController->commands[leg].qDes[0] = 0.;
                this->_data->_legController->commands[leg].qdDes = _firstStepSwing[leg].getJointVelocity();
                this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.2, 1.2, 1.2).asDiagonal();
                this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.010, 0.010, 0.010).asDiagonal();
            } else {
                _SwingTraj[leg].computeSwingTrajectoryBezier(phase, 0.3f);
                this->_data->_legController->commands[leg].qDes = _SwingTraj[leg].getJointPosition();
//                this->_data->_legController->commands[leg].qDes[0] = 0.;
                this->_data->_legController->commands[leg].qdDes = _SwingTraj[leg].getJointVelocity();
                this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.2, 1.2, 1.2).asDiagonal();
                this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.010, 0.010, 0.010).asDiagonal();
            }
        } else{
            if(leg == 0 || leg == 3){
                firstMove[leg] = false;
            }
            if (firstMove[leg]) {
                _firstStepStance[leg].computeSwingTrajectoryBezierWithoutMid(phase, 0.3f);
                printf("firstStance[%d] phase: %f q0: %f q1: %f q2: %f\n", leg, phase, _firstStepStance[leg].getJointPosition()[0],
                       _firstStepStance[leg].getJointPosition()[1], _firstStepStance[leg].getJointPosition()[2]);
                this->_data->_legController->commands[leg].qDes = _firstStepStance[leg].getJointPosition();
//                this->_data->_legController->commands[leg].qDes[0] = 0.;
                this->_data->_legController->commands[leg].qdDes = _firstStepStance[leg].getJointVelocity();
                this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.8, 1.8, 1.8).asDiagonal();
                this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.016, 0.016, 0.016).asDiagonal();
            } else {
                _StanceTraj[leg].computeSwingTrajectoryBezier(phase, 0.3f);
                this->_data->_legController->commands[leg].qDes = _StanceTraj[leg].getJointPosition();
//                this->_data->_legController->commands[leg].qDes[0] = 0.;
                this->_data->_legController->commands[leg].qdDes = _StanceTraj[leg].getJointVelocity();
                this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.8, 1.8, 1.8).asDiagonal();
                this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.016, 0.016, 0.016).asDiagonal();
            }
        }
    }

    if (_progress >= 100) {
        _progress = 0;
        for (int i = 0; i < 4; i++)
            swingState[i] = !swingState[i];
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