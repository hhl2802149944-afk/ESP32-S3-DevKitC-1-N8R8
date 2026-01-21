#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include "hello_world_main.cpp" // To get MotorState and other shared globals if needed, 
                                // but better to just share the state pointers.

#ifdef __cplusplus
extern "C" {
#endif

void start_ble_server(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVER_H
