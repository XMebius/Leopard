//
// Created by mebius on 24-5-13.
//

#include "FSM_State_Locomotion.h"

template<typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION") {
    // safety check
    this->turnOnAllSafetyChecks();
    this->checkPDesFoot = false;
    // define a MPC controller
    cMPC = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
                                   27 / (1000. * _controlFSMData->controlParameters->controller_dt),
                                   _controlFSMData->userParameters);
    _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    _wbc_data = new LocomotionCtrlData<T>();
}

template<typename T>
void FSM_State_Locomotion<T>::onEnter() {
    printf("[FSM_State_Locomotion] onEnter...\n");
    cMPC->initialize();
    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
//    printf("[FSM_State_Locomotion] after onEnter...\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Locomotion<T>::run() {
//    printf("[FSM_State_Locomotion] run...\n");
    cMPC->run<T>(*this->_data);
    Vec3<T> pDes_backup[4];
    Vec3<T> vDes_backup[4];
    Mat3<T> Kp_backup[4];
    Mat3<T> Kd_backup[4];

    for(int leg(0); leg<4; ++leg){
        pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
        vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
        Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
        Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
    }

    if(this->_data->userParameters->use_wbc > 0.9){
        _wbc_data->pBody_des = cMPC->pBody_des;
        _wbc_data->vBody_des = cMPC->vBody_des;
        _wbc_data->aBody_des = cMPC->aBody_des;


        _wbc_data->pBody_RPY_des = cMPC->pBody_RPY_des;
        _wbc_data->vBody_Ori_des = cMPC->vBody_Ori_des;

        for(size_t i(0); i<4; ++i){
            _wbc_data->pFoot_des[i] = cMPC->pFoot_des[i];
            _wbc_data->vFoot_des[i] = cMPC->vFoot_des[i];
            _wbc_data->aFoot_des[i] = cMPC->aFoot_des[i];
            _wbc_data->Fr_des[i] = cMPC->Fr_des[i];
        }
        _wbc_data->contact_state = cMPC->contact_state;
        _wbc_ctrl->run(_wbc_data, *this->_data);
    }
    for(int leg(0); leg<4; ++leg){
        //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
        this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
        //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
        this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
    }
}

template<typename T>
bool FSM_State_Locomotion<T>::isBusy() {
    return false;
}

template
class FSM_State_Locomotion<float>;