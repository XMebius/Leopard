//
// Created by mebius on 24-4-22.
//

#include <rt/rt_wheeltec.h>

bool init_port(void *pSerialPort) {
    CSerialPortInit(pSerialPort,
                    "/dev/ttyUSB0",
                    921600,
                    ParityNone,
                    DataBits8,
                    StopOne,
                    FlowNone,
                    4096);
    CSerialPortSetReadIntervalTimeout(pSerialPort, 20);
    int open_flag = CSerialPortOpen(pSerialPort);
    std::cout <<"open_flag: " << open_flag<<std::endl;
    return open_flag;
}

void read_vectorNav(void *pSerialPort, VectorNavData& vectorNavData) {
    while (true) {
        uint8_t check_head[1] = {0xff};
        CSerialPortReadData(pSerialPort, (void*)check_head, 1);
        if (check_head[0] != FRAME_HEAD)
            continue;

        uint8_t head_type[1] = {0xff};
        CSerialPortReadData(pSerialPort, (void*)head_type, 1);
        if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS &&
            head_type[0] != 0x50 && head_type[0] != TYPE_GROUND)
            continue;

        uint8_t check_len[1] = {0xff};
        CSerialPortReadData(pSerialPort, check_len, 1);
        if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
            continue;
        else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
            continue;

        uint8_t check_sn[1] = {0xff};
        CSerialPortReadData(pSerialPort, check_sn, 1);
        uint8_t head_crc8[1] = {0xff};
        CSerialPortReadData(pSerialPort, head_crc8, 1);
        uint8_t head_crc16_H[1] = {0xff};
        CSerialPortReadData(pSerialPort, head_crc16_H, 1);
        uint8_t head_crc16_L[1] = {0xff};
        CSerialPortReadData(pSerialPort, head_crc16_L, 1);

        if (head_type[0] == TYPE_IMU) {
            uint8_t read_msg[57];
            CSerialPortReadData(pSerialPort, read_msg, 57);
            // 解包IMU数据
            float imu_data[12];
            int timestamp;
            memcpy(imu_data, read_msg, sizeof(float) * 12);
            memcpy(&timestamp, read_msg + 48, sizeof(int));

            vectorNavData.gyro[0] = imu_data[0];
            vectorNavData.gyro[1] = imu_data[1];
            vectorNavData.gyro[2] = imu_data[2];
            vectorNavData.accelerometer[0] = imu_data[3];
            vectorNavData.accelerometer[1] = imu_data[4];
            vectorNavData.accelerometer[2] = imu_data[5];

//            printf("Gyroscope_X(rad/s): %f\n", imu_data[0]);
//            printf("Gyroscope_Y(rad/s): %f\n", imu_data[1]);
//            printf("Gyroscope_Z(rad/s): %f\n", imu_data[2]);
//            printf("Accelerometer_X(m/s^2): %f\n", imu_data[3]);
//            printf("Accelerometer_Y(m/s^2): %f\n", imu_data[4]);
//            printf("Accelerometer_Z(m/s^2): %f\n", imu_data[5]);
        }
        else if(head_type[0] == TYPE_AHRS) {
            uint8_t read_msg[49];
            CSerialPortReadData(pSerialPort, read_msg, 49);

            // 解包AHRS数据
            float ahrs_data[10];
            int timestamp;
            memcpy(ahrs_data, read_msg, sizeof(float) * 10);
            memcpy(&timestamp, read_msg + 40, sizeof(int));
            vectorNavData.quat[0] = ahrs_data[6];
            vectorNavData.quat[1] = ahrs_data[7];
            vectorNavData.quat[2] = ahrs_data[8];
            vectorNavData.quat[3] = ahrs_data[9];
//            printf("Q1: %f\n", ahrs_data[6]);
//            printf("Q2: %f\n", ahrs_data[7]);
//            printf("Q3: %f\n", ahrs_data[8]);
//            printf("Q4: %f\n", ahrs_data[9]);
        }
    }
}
