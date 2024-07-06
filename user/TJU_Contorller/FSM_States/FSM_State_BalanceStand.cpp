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

        standJointPos[leg] = this->_data->_legController->datas[leg].q; // 站立的关节角度

        forward_stance_ini_angle = standJointPos[leg];  // swing的init角度
        forward_stance_ini_angle[1] -= 0.17;
        forward_stance_ini_angle[2] -= 0.05;

        forward_stance_final_angle = standJointPos[leg];    // swing的final角度
        forward_stance_final_angle[1] += 0.1;
        forward_stance_final_angle[2] -= 0.05;

        forward_swing_mid = standJointPos[leg];     // swing的中间抬起的角度
        forward_swing_mid[1] -= 0.12;
        forward_swing_mid[2] += 0.8;
    }
}


template<typename T>
void FSM_State_BalanceStand<T>::run() {
    float coe = 5;
    float phase = coe * _progress * this->_data->controlParameters->controller_dt;
    float max_progress = 500 / coe;

    for (int leg = 0; leg < 4; leg++) {
        float x_vel_cmd = 0.f;
        x_vel_cmd = this->_data->_desiredStateCommand->leftAnalogStick[1];
        if (swingState[leg]) { // leg is in swing

            if (x_vel_cmd > 0.05f) {
                printf("moving forward\n");
                swing_ini = forward_stance_ini_angle;
                swing_fin = forward_stance_final_angle;
                swing_mid = forward_swing_mid;
            }else if(x_vel_cmd < -0.05f){
                printf("moving backward\n");
                swing_ini = forward_stance_final_angle;
                swing_fin = forward_stance_ini_angle;
                swing_mid = forward_swing_mid;
            } else {
                printf("local pace\n");
                swing_ini = standJointPos[leg];
                swing_fin = standJointPos[leg];
                swing_mid = forward_swing_mid;
            }

            float a(0.f);
            float b(1.f);
            Vec3<T> inter_point;
            if (_progress <= max_progress / 2) {    // begin到mid
                b = 2. * (float) _progress / max_progress;
                a = 1.f - b;
                inter_point = a * swing_ini + b * swing_mid;
            } else {    // mid到end
                b = 2. * (float) _progress / max_progress - 1;
                a = 1.f - b;
                inter_point = a * swing_mid + b * swing_fin;
            }

            this->_data->_legController->commands[leg].qDes = inter_point;
            this->_data->_legController->commands[leg].qdDes = Vec3<T>(0.f, 0.f, 0.f);
            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(4.5, 3.2, 2.4).asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.043, 0.036, 0.021).asDiagonal();
        } else {
            if (x_vel_cmd > 0.05f) {
                stance_ini = forward_stance_final_angle;
                stance_fin = forward_stance_ini_angle;
            } else if(x_vel_cmd < -0.05f){
                stance_ini = forward_stance_ini_angle;
                stance_fin = forward_stance_final_angle;
            } else {
                stance_ini = standJointPos[leg];
                stance_fin = standJointPos[leg];
            }

            float a(0.f);
            float b(1.f);
            if (_progress <= max_progress) {
                b = (float) _progress / max_progress;
                a = 1.f - b;
            }
            Vec3<T> inter_point = a * stance_ini + b * stance_fin;
            this->_data->_legController->commands[leg].qDes = inter_point;
            this->_data->_legController->commands[leg].qdDes = Vec3<T>(0.f, 0.f, 0.f);
            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(4.5, 3.8, 2.4).asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.043, 0.036, 0.021).asDiagonal();
        }
    }

    printf("_progress: %d, phase: %f, max_phase: %f\n", _progress, phase, max_progress);
    if (_progress >= max_progress) {
        _progress = 0;
        for (int i = 0; i < 4; i++)
            swingState[i] = !swingState[i];

    } else {
        _progress++;
    }
}


//
//template<typename T>
//void FSM_State_BalanceStand<T>::onEnter() {
//    _progress = 0;
//    for (int leg(0); leg < 4; leg++) {
//        init_point = this->_data->_legController->datas[leg].p;
//        _jointSwingTrajectories[leg].setInitialJointPosition(init_point, leg);
//        final_ponit_forward = init_point;
//        final_ponit_forward[0] += 0.04;
//        final_ponit_backward = init_point;
//        final_ponit_backward[0] -= 0.04;
//
//        _jointSwingTrajectories[leg].setFinalJointPosition(init_point, leg);
//        _jointSwingTrajectories[leg].setHeight(-0.22, leg);
//        standJointPos[leg] = this->_data->_legController->datas[leg].q;
//    }
//}
//
//template<typename T>
//void FSM_State_BalanceStand<T>::run() {
//
//    float coe = 5;
//    float phase = coe * _progress * this->_data->controlParameters->controller_dt;
//    float max_phase = 500 / coe;
//
//    for (int leg = 0; leg < 4; leg++) {
//
//        float x_vel_cmd = 0.f;
//        x_vel_cmd = this->_data->_desiredStateCommand->leftAnalogStick[1];
//        if (swingState[leg]) { // leg is in swing
//            if (enterSwing[leg]) {
////                printf("phase: %f, leg %d enter swing\n", phase, leg);
//                enterSwing[leg] = false;
//                enterStand[leg] = true;
//
//                _jointSwingTrajectories[leg].setInitialJointPosition(init_point, leg);
//
//                if (x_vel_cmd > 0.05f) {
//                    printf("moving forward\n");
//                    _jointSwingTrajectories[leg].setFinalJointPosition(final_ponit_forward, leg);
//                } else if (x_vel_cmd < -0.05f) {
//                    printf("moving backward\n");
//                    _jointSwingTrajectories[leg].setFinalJointPosition(final_ponit_backward, leg);
//                } else {
//                    _jointSwingTrajectories[leg].setFinalJointPosition(init_point, leg);    // 加入速度后的新位置
//                }
//
//                _jointSwingTrajectories[leg].setHeight(-0.22, leg);     // 中间点不变，高度变为-0.22
//            }
//            _jointSwingTrajectories[leg].computeSwingTrajectoryBezier(phase, 2.f);      // 更新关节位置
//
//            this->_data->_legController->commands[leg].qDes = _jointSwingTrajectories[leg].getJointPosition();
//            this->_data->_legController->commands[leg].qdDes = _jointSwingTrajectories[leg].getJointVelocity();
//            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(2.2, 2.2, 2.2).asDiagonal();
//            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.022, 0.022, 0.022).asDiagonal();
//        } else {    // leg is in stance
//            if (enterStand[leg]) {
////                printf("phase: %f, leg %d enter stand\n", phase, leg);
//                enterStand[leg] = false;
//                enterSwing[leg] = true;
//                if (x_vel_cmd > 0.05f) {
//                    stance_ini = JointSwingTrajectory<T>::inverseKinematics(final_ponit_forward, leg);
//                } else if (x_vel_cmd < -0.05f) {
//                    stance_ini = JointSwingTrajectory<T>::inverseKinematics(final_ponit_backward, leg);
//                } else {
//                    stance_ini = standJointPos[leg];
//                }
//                stance_fin = standJointPos[leg];    // 站立的位置
//            }
//            float a(0.f);
//            float b(1.f);
//            if (_progress <= max_phase) {
//                b = (float) _progress / max_phase;
//                a = 1.f - b;
//            }
//            Vec3<T> inter_point = a * stance_ini + b * stance_fin;
//
//            this->_data->_legController->commands[leg].qDes = inter_point;
//            this->_data->_legController->commands[leg].qdDes = Vec3<T>(0.f, 0.f, 0.f);
//            this->_data->_legController->commands[leg].kpJoint = Vec3<T>(3.2, 3.2, 3.2).asDiagonal();
//            this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.03, 0.03, 0.03).asDiagonal();
//
//        }
//    }
//
//    int swingStateBackup[4];
//    for (int i = 0; i < 4; i++)
//        swingStateBackup[i] = swingState[i];
//
//    if (_progress >= max_phase / 2) {
//        for (int i = 0; i < 4; i++)
//            if (swingState[i] > 1)
//                swingState[i] = 0;
//
//        if (_progress >= max_phase) {
//            for (int i = 0; i < 4; i++)
//                swingState[i] = !swingStateBackup[i];
//            _progress = 0;
//        }
//        _progress++;
//    } else{
//        _progress++;
//    }
//
////    if (_progress >= 100) {
////        _progress = 0;
////        // 1 / 2
////        for (int i = 0; i < 4; i++)
////            swingState[i] = !swingState[i];
////
//////        // 3 / 4
//////        for (int i = 0; i < 4; i++) {
//////            swingState[i] = 0;
//////        }
//////        switch (count % 4) {
//////            case 0:
//////                swingState[0] = 1;
//////                break;
//////            case 1:
//////                swingState[2] = 1;
//////                break;
//////            case 2:
//////                swingState[1] = 1;
//////                break;
//////            case 3:
//////                swingState[3] = 1;
//////                break;
//////            default:
//////                break;
//////        }
//////        count++;
////
////    } else {
////        _progress++;
////    }
//}

template<typename T>
bool FSM_State_BalanceStand<T>::isBusy() {
    return false;
}

template
class FSM_State_BalanceStand<float>;