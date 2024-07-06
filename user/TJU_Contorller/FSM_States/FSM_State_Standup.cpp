/*============================= Stand Up ==============================*/
/** 执行之后，机器人保持站立状态。
 */
#include "FSM_State_StandUp.h"
#include "Math/Interpolation.h"

#define PI (3.141592654f)

/**
 * 构造函数
 *
 * @param _controlFSMData 这里面保存了相关参数
 */
template<typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
          _ini_foot_pos(4), _ini_joint_pos(4), _end_joint_pos(4) {

    this->turnOffAllSafetyChecks();
}

template<typename T>
void FSM_State_StandUp<T>::onEnter() {
    printf("[FSM_State_StandUp] onEnter...\n");
    // Reset iteration counter
    iter = 0;

    for (int leg(0); leg < 4; ++leg) {
        T hMax = 0.32;
        Vec3<T> pDes = this->_data->_legController->datas[leg].p;
        pDes[2] = -hMax;

        _ini_joint_pos[leg] = this->_data->_legController->datas[leg].q;
        _end_joint_pos[leg] = this->_data->_legController->inverseKinematics(pDes, leg);
    }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
//template<typename T>
//void FSM_State_StandUp<T>::run() {
//
//    float phase = iter * this->_data->controlParameters->controller_dt;
//
//    for (int leg(0); leg < 4; leg++) {
//        // 计算贝塞尔曲线（针对joint）
//        jointSwingTrajectories[leg].computeSwingTrajectoryBezier(phase, 2.f);
//
//        this->_data->_legController->commands[leg].qDes = jointSwingTrajectories[leg].getJointPosition();
//        this->_data->_legController->commands[leg].qdDes = jointSwingTrajectories[leg].getJointVelocity();
//        this->_data->_legController->commands[leg].kpJoint = Vec3<T>(1.8, 1.8, 1.8).asDiagonal();
//        this->_data->_legController->commands[leg].kdJoint = Vec3<T>(0.02, 0.02, 0.02).asDiagonal();
//    }
//
//    if (iter >= 500) {
//        iter = 500;
//    } else {
//        iter++;
//    }
//}
template<typename T>
void FSM_State_StandUp<T>::run() {
    iter++;
    T progress = 1 * iter * this->_data->controlParameters->controller_dt;

    if (progress > 1.0) { progress = 1.0; }

    for (int i = 0; i < 4; i++) {
        this->_data->_legController->commands[i].qDes = Interpolate::cubicBezier<Vec3<T>>(_ini_joint_pos[i],
                                                                                          _end_joint_pos[i], progress);
        this->_data->_legController->commands[i].qdDes
                = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_ini_joint_pos[i], _end_joint_pos[i], progress);
        this->_data->_legController->commands[i].kpJoint = Vec3<T>(1.8, 1.8, 1.8).asDiagonal();
        this->_data->_legController->commands[i].kdJoint = Vec3<T>(0.02, 0.02, 0.02).asDiagonal();
    }
}

/**
 * 判断动作是否完成,Busy状态下不允许切换状态
*/
template<typename T>
bool FSM_State_StandUp<T>::isBusy() {

//    T t = (iter * this->_data->controlParameters->controller_dt) /
//          (this->_data->userParameters->stand_up_time);
//
//    if (t < 1.0f)
//        return true;
//    else
    return false;
}

// template class FSM_State_StandUp<double>;
template
class FSM_State_StandUp<float>;
