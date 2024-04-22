//
// Created by mebius on 24-4-22.
//


#include "rt/rt_usb.h"
#define MOTOR_CHEATER

MotorCmd abad_cmd, hip_cmd, knee_cmd;
std::vector<MotorCmd> spine_leg_cmd{abad_cmd, hip_cmd, knee_cmd};

MotorData abad_data, hip_data, knee_data;
std::vector<MotorData> spine_leg_data{abad_data, hip_data, knee_data};

usb_command_t usb_command_drv;
usb_data_t usb_data_drv;
usb_torque_t usb_torque_drv;

SerialPort serial("/dev/ttyACM0");

pthread_mutex_t usb_mutex;

const float max_torque[3] = {17.f, 17.f, 26.f};// TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};  // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

// only used for actual robot
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};

// only used for actual robot
const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float hip_offset[4] = {M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
const float knee_offset[4] = {K_KNEE_OFFSET_POS, -K_KNEE_OFFSET_POS,
                              -K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};


void init_usb() {
    // check size
    size_t command_size = sizeof(usb_command_t);
    size_t data_size = sizeof(usb_data_t);

    memset(&usb_command_drv, 0, sizeof(usb_command_drv));
    memset(&usb_data_drv, 0, sizeof(usb_data_drv));

    if (pthread_mutex_init(&usb_mutex, NULL) != 0) {
        printf("Error initializing usb mutex\n");
    }

    if (command_size != K_EXPECTED_COMMAND_SIZE) {
        printf("usb_command_t size is %lu, expected %d\n", command_size, K_EXPECTED_COMMAND_SIZE);
    } else {
        printf("[RT USB] command size good\n");
    }

    if (data_size != K_EXPECTED_DATA_SIZE) {
        printf("usb_data_t size is %lu, expected %d\n", data_size, K_EXPECTED_DATA_SIZE);
    } else {
        printf("[RT USB] data size good\n");
    }

    printf("[RT USB] open\n");

    // open serial port
}

void fake_usb_control(usb_command_t *cmd, usb_data_t *data,
                      usb_torque_t *torque_out, int leg_num) {

    torque_out->tau_abad[leg_num] =
            cmd->kp_abad[leg_num] * (cmd->q_des_abad[leg_num] - data->q_abad[leg_num]) +
            cmd->kd_abad[leg_num] * (cmd->qd_des_abad[leg_num] - data->qd_abad[leg_num]) +
            cmd->tau_abad_ff[leg_num];

    torque_out->tau_hip[leg_num] =
            cmd->kp_hip[leg_num] * (cmd->q_des_hip[leg_num] - data->q_hip[leg_num]) +
            cmd->kd_hip[leg_num] * (cmd->qd_des_hip[leg_num] - data->qd_hip[leg_num]) +
            cmd->tau_hip_ff[leg_num];

    torque_out->tau_knee[leg_num] =
            cmd->kp_knee[leg_num] * (cmd->q_des_knee[leg_num] - data->q_knee[leg_num]) +
            cmd->kd_knee[leg_num] * (cmd->qd_des_knee[leg_num] - data->qd_knee[leg_num]) +
            cmd->tau_knee_ff[leg_num];

    const float *torque_limits = disabled_torque;

    if (cmd->flags[leg_num] & 0b1) {
        if (cmd->flags[leg_num] & 0b10)
            torque_limits = wimp_torque;
        else
            torque_limits = max_torque;
    }

    if (torque_out->tau_abad[leg_num] > torque_limits[0])
        torque_out->tau_abad[leg_num] = torque_limits[0];
    if (torque_out->tau_abad[leg_num] < -torque_limits[0])
        torque_out->tau_abad[leg_num] = -torque_limits[0];

    if (torque_out->tau_hip[leg_num] > torque_limits[1])
        torque_out->tau_hip[leg_num] = torque_limits[1];
    if (torque_out->tau_hip[leg_num] < -torque_limits[1])
        torque_out->tau_hip[leg_num] = -torque_limits[1];

    if (torque_out->tau_knee[leg_num] > torque_limits[2])
        torque_out->tau_knee[leg_num] = torque_limits[2];
    if (torque_out->tau_knee[leg_num] < -torque_limits[2])
        torque_out->tau_knee[leg_num] = -torque_limits[2];
}

void printCmd(usb_command_t* cmd, uint8_t leg) {
    printf("abad--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
           (cmd->q_des_abad[leg] * abad_side_sign[leg]) + abad_offset[leg],
           cmd->qd_des_abad[leg] * abad_side_sign[leg],
           cmd->kp_abad[leg],
           cmd->kd_abad[leg],
           cmd->tau_abad_ff[leg] * abad_side_sign[leg]);

    printf("hip--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
             (cmd->q_des_hip[leg] * hip_side_sign[leg]) + hip_offset[leg],
              cmd->qd_des_hip[leg] * hip_side_sign[leg],
              cmd->kp_hip[leg],
              cmd->kd_hip[leg],
              cmd->tau_hip_ff[leg] * hip_side_sign[leg]);

    printf("knee--> q_des: %f, qd_des: %f, kp:%f, kd:%f, tau:%f\n",
                (cmd->q_des_knee[leg] * knee_side_sign[leg]) + knee_offset[leg],
                cmd->qd_des_knee[leg] * knee_side_sign[leg],
                cmd->kp_knee[leg],
                cmd->kd_knee[leg],
                cmd->tau_knee_ff[leg] * knee_side_sign[leg]);
}

void usb_to_spine(usb_command_t *cmd, std::vector<MotorCmd> &SPine_leg_cmd, uint8_t leg) {

//    printCmd(cmd, leg);

    assert(leg >= 0 && leg <= 3);
    // todo: mode settings and mode
    SPine_leg_cmd[0].id = leg * 3 + 0;// abad
    SPine_leg_cmd[1].id = leg * 3 + 1;// hip
    SPine_leg_cmd[2].id = leg * 3 + 2;// knee

    for (auto &i: SPine_leg_cmd) {
        i.mode = 1;// foc control
    }

#ifndef MOTOR_CHEATER
    SPine_leg_cmd[0].Pos = (cmd->q_des_abad[leg] * abad_side_sign[leg]) + abad_offset[leg];// abad
    SPine_leg_cmd[1].Pos = (cmd->q_des_hip[leg] * hip_side_sign[leg]) + hip_offset[leg];   // hip
    SPine_leg_cmd[2].Pos = (cmd->q_des_knee[leg] * knee_side_sign[leg]) + knee_offset[leg];// knee

    SPine_leg_cmd[0].W = cmd->qd_des_abad[leg] * abad_side_sign[leg];// abad
    SPine_leg_cmd[1].W = cmd->qd_des_hip[leg] * hip_side_sign[leg];  // hip
    SPine_leg_cmd[2].W = cmd->qd_des_knee[leg] * knee_side_sign[leg];// knee

    SPine_leg_cmd[0].K_P = cmd->kp_abad[leg];// abad
    SPine_leg_cmd[1].K_P = cmd->kp_hip[leg]; // hip
    SPine_leg_cmd[2].K_P = cmd->kp_knee[leg];// knee

    SPine_leg_cmd[0].K_W = cmd->kd_abad[leg];// abad
    SPine_leg_cmd[1].K_W = cmd->kd_hip[leg]; // hip
    SPine_leg_cmd[2].K_W = cmd->kd_knee[leg];// knee

    SPine_leg_cmd[0].T = cmd->tau_abad_ff[leg] * abad_side_sign[leg];// abad
    SPine_leg_cmd[1].T = cmd->tau_hip_ff[leg] * hip_side_sign[leg];  // hip
    SPine_leg_cmd[2].T = cmd->tau_knee_ff[leg] * knee_side_sign[leg];// knee



#else
    (void) cmd;
//    printf("Using fake usb data\n");
    SPine_leg_cmd[0].Pos = 0.0;// abad
    SPine_leg_cmd[1].Pos = 0.0;// hip
    SPine_leg_cmd[2].Pos = 0.0;// knee

    SPine_leg_cmd[0].W = 3.14 / 4. * 6.33;// abad
    SPine_leg_cmd[1].W = 3.14 / 4. * 6.33;// hip
    SPine_leg_cmd[2].W = 3.14 / 4. * 6.33;// knee

    //    SPine_leg_cmd[0].W=0; // abad
    //    SPine_leg_cmd[1].W=0; // hip
    //    SPine_leg_cmd[2].W=0; // knee

    SPine_leg_cmd[0].K_P = 0.0;// abad
    SPine_leg_cmd[1].K_P = 0.0;// hip
    SPine_leg_cmd[2].K_P = 0.0;// knee

    SPine_leg_cmd[0].K_W = 0.05;// abad
    SPine_leg_cmd[1].K_W = 0.05;// hip
    SPine_leg_cmd[2].K_W = 0.05;// knee

    SPine_leg_cmd[0].T = 0.0;// abad
    SPine_leg_cmd[1].T = 0.0;// hip
    SPine_leg_cmd[2].T = 0.0;// knee

#endif

}

void spine_to_usb(usb_data_t *data, std::vector<MotorData> &SPine_leg_data, uint8_t leg) {

    data->q_abad[leg] = (SPine_leg_data[0].Pos - abad_offset[leg]) * abad_side_sign[leg];// abad
    data->q_hip[leg] = (SPine_leg_data[1].Pos - hip_offset[leg]) * hip_side_sign[leg];   // hip
    data->q_knee[leg] = (SPine_leg_data[2].Pos - knee_offset[leg]) * knee_side_sign[leg];// knee

    data->qd_abad[leg] = SPine_leg_data[0].W * abad_side_sign[leg];// abad
    data->qd_hip[leg] = SPine_leg_data[1].W * hip_side_sign[leg];  // hip
    data->qd_knee[leg] = SPine_leg_data[2].W * knee_side_sign[leg];// knee

//    printf("q_abad: %f, q_hip: %f, q_knee: %f\n", data->q_abad[leg], data->q_hip[leg], data->q_knee[leg]);
//    printf("qd_abad: %f, qd_hip: %f, qd_knee: %f\n", data->qd_abad[leg], data->qd_hip[leg], data->qd_knee[leg]);

    // no check
}


int usb_driver_iterations = 0;
void usb_send_receive(usb_command_t *command, usb_data_t *data) {
    usb_driver_iterations++;
    data->spi_driver_status = usb_driver_iterations << 16;

    // each leg TODO: send 3 motors at the same time???
    for (uint8_t leg = 0; leg < 4; leg++) {
        usb_to_spine(command, spine_leg_cmd, leg);
        serial.sendRecv(spine_leg_cmd, spine_leg_data);
        spine_to_usb(data, spine_leg_data, leg);
    }
//    leg=3;
}

void usb_driver_run() {
    for (int i = 3; i < 4; i++) {
        fake_usb_control(&usb_command_drv, &usb_data_drv, &usb_torque_drv, i);
    }

    // lock mutex
    pthread_mutex_lock(&usb_mutex);
    usb_send_receive(&usb_command_drv, &usb_data_drv);
    pthread_mutex_unlock(&usb_mutex);
}

usb_command_t *get_usb_command() {
    return &usb_command_drv;
}

usb_data_t *get_usb_data() {
    return &usb_data_drv;
}
