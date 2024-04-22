set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error")
include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_library(cserialport SHARED ${SOURCES})
target_link_libraries(cserialport pthread)