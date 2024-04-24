//
// Created by mebius on 24-4-23.
//

#include <main_helper.h>
#include "TJU_Controller.hpp"

int main(int argc, char **argv) {
    std::cout << "TJU Controller Starts..." << std::endl;

    try {
        main_helper(argc, argv, new TJU_Controller());
    } catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return 0;
}