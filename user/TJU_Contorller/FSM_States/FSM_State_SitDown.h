//
// Created by mebius on 24-5-12.
//

#ifndef LEOPARD_FSM_STATE_SITDOWN_H
#define LEOPARD_FSM_STATE_SITDOWN_H
#include "FSM_State.h"

template<typename T>
class FSM_State_SitDown: public FSM_State<T>{
public:
    FSM_State_SitDown(ControlFSMData<T>* _controlFSMData);

    void onEnter() override;

    void run() override;

    bool isBusy() override;
private:
    std::vector< Vec3<T> > _ini_joint_pos;
    std::vector< Vec3<T> > _end_joint_pos;
    int iter = 0;
};


#endif //LEOPARD_FSM_STATE_SITDOWN_H
