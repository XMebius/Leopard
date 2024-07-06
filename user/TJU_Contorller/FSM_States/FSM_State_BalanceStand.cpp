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
    for (int leg(0); leg < 4; leg++) {
        _jointSwingTrajectories[leg].setInitialJointPosition(this->_data->_legController->datas[leg].p, leg);
        _jointSwingTrajectories[leg].setFinalJointPosition(this->_data->_legController->datas[leg].p, leg);
        _jointSwingTrajectories[leg].setHeight(-0.19, leg);
        standJointPos[leg] = this->_data->_legController->datas[leg].q;
    }
}

template<typename T>
void FSM_State_BalanceStand<T>::run() {

    float phase = 5 * _progress * this->_data->controlParameters->controller_dt;

    for (int leg = 0; leg < 4; leg++) {
        if (swingState[leg]) { // leg is in swing
            _jointSwingTrajectories[leg].computeSwingTrajectoryBezier(phase, 0.3f);
            this->_data->_legController->commands[leg].qDes = _jointSwingTrajectories[leg].getJointPosition();
            this->_data->_legController->commands[leg].qdDes = _jointSwingTrajectories[leg].getJointVelocity();
            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(2.2, 2.2, 2.2).asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.022, 0.022, 0.022).asDiagonal();
            printf("leg: %d, swingState: %d, qDes: %f, %f, %f\n", leg, swingState[leg],
                   this->_data->_legController->commands[leg].qDes[0],
                   this->_data->_legController->commands[leg].qDes[1],
                   this->_data->_legController->commands[leg].qDes[2]);
        } else {    // leg is in stance
            this->_data->_legController->commands[leg].qDes = standJointPos[leg];
            this->_data->_legController->commands[leg].qdDes = Vec3<T>(0.f, 0.f, 0.f);
            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(3.2, 3.2, 3.2).asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.030, 0.030, 0.030).asDiagonal();
            printf("leg: %d, swingState: %d, qDes: %f, %f, %f\n", leg, swingState[leg],
                   this->_data->_legController->commands[leg].qDes[0],
                   this->_data->_legController->commands[leg].qDes[1],
                   this->_data->_legController->commands[leg].qDes[2]);
        }
    }

    if (_progress >= 100) {
        _progress = 0;
        for (int i = 0; i < 4; i++)
            swingState[i] = !swingState[i];

//        for (int i = 0; i < 4; i++) {
//            swingState[i] = 0;
//        }
//        switch (count % 4) {
//            case 0:
//                swingState[0] = 1;
//                break;
//            case 1:
//                swingState[2] = 1;
//                break;
//            case 2:
//                swingState[1] = 1;
//                break;
//            case 3:
//                swingState[3] = 1;
//                break;
//            default:
//                break;
//        }
////        swingState[count % 4] = 1;
//        count++;
    } else {
        _progress++;
    }
}

template<typename T>
bool FSM_State_BalanceStand<T>::isBusy() {
    return false;
}

template
class FSM_State_BalanceStand<float>;