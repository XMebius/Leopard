add_library(UnitreeMotorSDK SHARED IMPORTED)
set_target_properties(UnitreeMotorSDK PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libUnitreeMotorSDK_M80106_Linux64.so"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
add_library(UniTree INTERFACE)
target_link_libraries(UniTree INTERFACE UnitreeMotorSDK)