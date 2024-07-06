#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"
#include <Controllers/JointSwingTrajectory.h>


/**
 *
 */
template<typename T>
class FSM_State_StandUp : public FSM_State<T> {
public:
    FSM_State_StandUp(ControlFSMData<T> *_controlFSMData);

    // Behavior to be carried out when entering a state
    void onEnter();

    void _SetJPosInterPts(
            const size_t &curr_iter, size_t max_iter, int leg,
            const Vec3<T> &ini, const Vec3<T> &fin);
    // Run the normal behavior for the state
    void run();

    bool isBusy();

private:
    // 迭代器，进入状态后迭代了多少个dt周期
    int iter = 0;
    std::vector <Vec3<T>> _ini_foot_pos;
    std::vector <Vec3<T>> _ini_joint_pos;
    std::vector <Vec3<T>> _end_joint_pos;
    JointSwingTrajectory<T> jointSwingTrajectories[4];

};

#endif  // FSM_STATE_STANDUP_H
