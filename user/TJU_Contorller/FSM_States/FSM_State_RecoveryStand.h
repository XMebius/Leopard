/*
 * @file FSM_State_RecoveryStand.h
 * @brief 
 */

#ifndef LEOPARD_FSM_STATE_RECOVERYSTAND_H
#define LEOPARD_FSM_STATE_RECOVERYSTAND_H

#include "FSM_State.h"
#include "Math/Interpolation.h"

template<typename T>
class FSM_State_RecoveryStand : public FSM_State<T>  {
public:
    FSM_State_RecoveryStand(ControlFSMData<T> *_controlFSMData);

    void onEnter();

    void run();

    bool isBusy();

private:

    void _RollOver(const int & iter);
    void _StandUp(const int & iter);
    void _FoldLegs(const int & iter);

    bool _UpsideDown();
    void _SetJPosInterPts(
            const size_t & curr_iter, size_t max_iter, int leg,
            const Vec3<T> & ini, const Vec3<T> & fin);

    int iter = 0;
    int _motion_start_iter = 0;
    unsigned long long _state_iter;
    int _flag = FoldLegs;

    static constexpr int StandUp = 0;
    static constexpr int FoldLegs = 1;
    static constexpr int RollOver = 2;

    // JPos
    Vec3<T> fold_jpos[4];
    Vec3<T> stand_jpos[4];
    Vec3<T> rolling_jpos[4];
    Vec3<T> initial_jpos[4];
    Vec3<T> zero_vec3;

    // 0.5 kHz
    const int rollover_ramp_iter = 150;
    const int rollover_settle_iter = 150;

    const int fold_ramp_iter = 400;
    const int fold_settle_iter = 700;

    int standup_ramp_iter = 250;
    const int standup_settle_iter = 250;

    bool firstStandUp = true;

};


#endif //LEOPARD_FSM_STATE_RECOVERYSTAND_H
