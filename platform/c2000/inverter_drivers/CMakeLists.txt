add_library(inverter_drivers 
    adcdriver.cpp
    breakswitchdriver.cpp
    eeprom.cpp 
    current.cpp 
    currentvoltagedriver.cpp 
    encoder.cpp 
    encoderdriver.cpp
    pmicspidriver.cpp 
    pwmdriver.cpp 
    gatedriver.cpp 
    gatedriverinterface.cpp
    scheduler.cpp)
target_link_libraries(inverter_drivers global_options libopeninv device_support)
target_include_directories(inverter_drivers
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
