//
// Created by mebius on 24-5-9.
//

#include "rt/rt_beitong.h"

int gampad_fd;
const char* gamepad_path = "/dev/input/beitong";
struct input_event beiTong_ev;

bool init_gamepad() {
    gampad_fd = open(gamepad_path, O_RDONLY);
    if (gampad_fd < 0) {
        std::cout << "Failed to open gamepad" << std::endl;
        return false;
    }
    return true;
}

void read_gamepad(BeiTong &beiTong) {
    if (read(gampad_fd, &beiTong_ev, sizeof(beiTong_ev)) == sizeof(beiTong_ev)) {
        if (beiTong_ev.type == EV_KEY) {
            switch (beiTong_ev.code) {
                case BTN_A:
                    beiTong.a = beiTong_ev.value;
                    printf("a button pressed: %d\n", beiTong_ev.value);
                    break;
                case BTN_B:
                    beiTong.b = beiTong_ev.value;
                    printf("b button pressed: %d\n", beiTong_ev.value);
                    break;
                case BTN_X:
                    beiTong.x = beiTong_ev.value;
                    break;
                case BTN_Y:
                    beiTong.y = beiTong_ev.value;
                    printf("y button pressed: %d\n", beiTong_ev.value);
                    break;
                case BTN_START:
                    beiTong.start = beiTong_ev.value;
                    break;
                case BTN_BACK:
                    beiTong.back = beiTong_ev.value;
                    break;
                case BTN_TL:
                    beiTong.leftBumper = beiTong_ev.value;
                    printf("left bumper: %d\n", beiTong_ev.value);
                    break;
                case BTN_TR:
                    beiTong.rightBumper = beiTong_ev.value;
                    printf("right bumper: %d\n", beiTong_ev.value);
                    break;
                case BTN_TL2:
                    beiTong.leftTriggerButton = beiTong_ev.value;
                    break;
                case BTN_TR2:
                    beiTong.rightTriggerButton = beiTong_ev.value;
                    break;
                default:
                    break;
            }
        } else if (beiTong_ev.type == EV_ABS) {
            switch (beiTong_ev.code) {
                case ABS_X:
                    beiTong.leftStick[0] = (128.f - beiTong_ev.value) / 128.f;
                    beiTong.leftStick[0] = std::max(-1.f, std::min(1.f, beiTong.leftStick[0]));
                    break;
                case ABS_Y:
                    beiTong.leftStick[1] = (128.f - beiTong_ev.value) / 128.f;
                    beiTong.leftStick[1] = std::max(-1.f, std::min(1.f, beiTong.leftStick[1]));
                    break;
                case ABS_Z:
                    beiTong.rightStick[0] = (128.f - beiTong_ev.value) / 128.f;
                    beiTong.rightStick[0] = std::max(-1.f, std::min(1.f, beiTong.rightStick[0]));
                    break;
                case ABS_RZ:
                    beiTong.rightStick[1] = (128.f - beiTong_ev.value) / 128.f;
                    beiTong.rightStick[1] = std::max(-1.f, std::min(1.f, beiTong.rightStick[1]));
                    break;
                default:
                    break;
            }
        }
    }
}