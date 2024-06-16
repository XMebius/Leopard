/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "FSM_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE"),
          _ini_joint_pos(4), _end_joint_pos(4) {
    // Do nothing
    // Set the pre controls safety checks
//    this->turnOnAllSafetyChecks();
    this->turnOffAllSafetyChecks();
}

template<typename T>
void FSM_State_Passive<T>::onEnter() {

    printf("[FSM_State_Passive] onEnter...\n");
    // init_pos
//    for (size_t leg(0); leg < 4; ++leg) {
//        _ini_joint_pos[leg] = this->_data->_legController->datas[leg].q;
//        _end_joint_pos[leg] = this->_data->_legController->datas[leg].q;
//    }
}

/**
 * Busy状态下不允许切换状态
*/
template<typename T>
bool FSM_State_Passive<T>::isBusy() {
    // not busy, always return false
    printf("[FSM_State_Passive] isBusy...\n");
    return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Passive<T>::run() {
    iter++;
//    if(iter < 20) {
//        for(int leg=0; leg<4; leg++) {
//            printf("J matrix for leg %d\n", leg);
//            auto datas = this->_data->_legController->datas;
//            for (int i = 0; i < 3; i++) {
//                printf("%f %f %f\n", datas[leg].J.transpose()(i, 0), datas[leg].J.transpose()(i, 1),
//                       datas[leg].J.transpose()(i, 2));
//            }
//        }
//        printf("\n");
//    }
//    Vec3<T> kp(this->_data->userParameters->Kp_stand[0],
//               this->_data->userParameters->Kp_stand[1],
//               this->_data->userParameters->Kp_stand[2]);
//    Vec3<T> kd(this->_data->userParameters->Kd_stand[0],
//               this->_data->userParameters->Kd_stand[1],
//               this->_data->userParameters->Kd_stand[2]);
//
//    for (int leg = 0; leg < 4; leg++) {
//        this->_data->_legController->commands[leg].kpJoint = kp.asDiagonal();
//        this->_data->_legController->commands[leg].kdJoint = kd.asDiagonal();
//
//        this->_data->_legController->commands[leg].qDes =
//                Interpolate::cubicBezier<Vec3<T >>(_ini_joint_pos[leg], _end_joint_pos[leg], 0.1);
//        this->_data->_legController->commands[leg].qdDes =
//                Interpolate::cubicBezierFirstDerivative<Vec3<T >>(_ini_joint_pos[leg], _end_joint_pos[leg], 0.1) / 0.1;
//    }

}

// template class FSM_State_Passive<double>;
template
class FSM_State_Passive<float>;
