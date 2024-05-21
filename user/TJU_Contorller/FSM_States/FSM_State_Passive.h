#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"
#include "Math/Interpolation.h"

template<typename T>
class FSM_State_Passive : public FSM_State<T> {
public:
    FSM_State_Passive(ControlFSMData<T> *_controlFSMData);

    void onEnter();

    void run();

    bool isBusy();

private:
    std::vector<Vec3<T> > _ini_joint_pos;
    std::vector<Vec3<T> > _end_joint_pos;
};

#endif  // FSM_STATE_PASSIVE_H
