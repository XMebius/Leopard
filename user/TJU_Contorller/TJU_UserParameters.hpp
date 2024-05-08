//
// Created by mebius on 24-4-22.
//

#ifndef LEOPARD_TJU_USERPARAMETERS_HPP
#define LEOPARD_TJU_USERPARAMETERS_HPP

#include "ControlParameters/ControlParameters.h"

class TJU_UserParameters: public ControlParameters {
public:
    TJU_UserParameters():
    ControlParameters("user-parameters"),
    INIT_PARAMETER(cmpc_gait),
    INIT_PARAMETER(cmpc_x_drag),
    INIT_PARAMETER(cmpc_use_sparse),
    INIT_PARAMETER(cmpc_bonus_swing),
    INIT_PARAMETER(use_wbc),
    INIT_PARAMETER(sit_down_height),
    INIT_PARAMETER(stand_up_height),
    INIT_PARAMETER(sit_down_time),
    INIT_PARAMETER(stand_up_time),
    INIT_PARAMETER(Kp_stand),
    INIT_PARAMETER(Kd_stand),
    INIT_PARAMETER(Kp_body),
    INIT_PARAMETER(Kd_body),
    INIT_PARAMETER(Kp_ori),
    INIT_PARAMETER(Kd_ori),
    INIT_PARAMETER(Kp_foot),
    INIT_PARAMETER(Kd_foot),
    INIT_PARAMETER(Kp_joint),
    INIT_PARAMETER(Kd_joint),
    INIT_PARAMETER(stance_legs),
    INIT_PARAMETER(use_jcqp),
    INIT_PARAMETER(jcqp_max_iter),
    INIT_PARAMETER(jcqp_rho),
    INIT_PARAMETER(jcqp_sigma),
    INIT_PARAMETER(jcqp_alpha),
    INIT_PARAMETER(jcqp_terminate),
    INIT_PARAMETER(gait_type),
    INIT_PARAMETER(gait_period_time),
    INIT_PARAMETER(gait_switching_phase),
    INIT_PARAMETER(gait_override)
    {}

    // MPC
    DECLARE_PARAMETER(double, cmpc_gait);
    DECLARE_PARAMETER(double, cmpc_x_drag);
    DECLARE_PARAMETER(double, cmpc_use_sparse);
    DECLARE_PARAMETER(double, cmpc_bonus_swing);
    DECLARE_PARAMETER(double, use_wbc);

    DECLARE_PARAMETER(double, sit_down_height);
    DECLARE_PARAMETER(double, stand_up_height);
    DECLARE_PARAMETER(double, sit_down_time);
    DECLARE_PARAMETER(double, stand_up_time);

    DECLARE_PARAMETER(Vec3<double>, Kp_stand);
    DECLARE_PARAMETER(Vec3<double>, Kd_stand);

    // PD
    DECLARE_PARAMETER(Vec3<double>, Kp_body);
    DECLARE_PARAMETER(Vec3<double>, Kd_body);

    DECLARE_PARAMETER(Vec3<double>, Kp_ori);
    DECLARE_PARAMETER(Vec3<double>, Kd_ori);

    DECLARE_PARAMETER(Vec3<double>, Kp_foot);
    DECLARE_PARAMETER(Vec3<double>, Kd_foot);

    DECLARE_PARAMETER(Vec3<double>, Kp_joint);
    DECLARE_PARAMETER(Vec3<double>, Kd_joint);

    DECLARE_PARAMETER(double, stance_legs);

    // QP
    DECLARE_PARAMETER(double, use_jcqp);
    DECLARE_PARAMETER(double, jcqp_max_iter);
    DECLARE_PARAMETER(double, jcqp_rho);
    DECLARE_PARAMETER(double, jcqp_sigma);
    DECLARE_PARAMETER(double, jcqp_alpha);
    DECLARE_PARAMETER(double, jcqp_terminate);

    // Gait Scheduler
    DECLARE_PARAMETER(double, gait_type);
    DECLARE_PARAMETER(double, gait_period_time);
    DECLARE_PARAMETER(double, gait_switching_phase);
    DECLARE_PARAMETER(double, gait_override);
};

#endif //LEOPARD_TJU_USERPARAMETERS_HPP
