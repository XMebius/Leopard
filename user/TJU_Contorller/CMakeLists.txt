include_directories("./")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../lcm-types/cpp")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
#include_directories("/usr/local/include/lcm/")   # lcm includes
#include_directories("/usr/local/include/eigen3")   # eigen includes

file(GLOB_RECURSE SRC_FILES
        "./*.cpp"
)

add_executable(tj_ctrl ${SRC_FILES} TJU_Controller.cpp main.cpp)
target_link_libraries(tj_ctrl robot biomimetics)
target_link_libraries(tj_ctrl dynacore_param_handler qpOASES)


