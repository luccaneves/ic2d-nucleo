#ifndef CONFIG_ANALOG_TORQUE_SENSOR_H
#define CONFIG_ANALOG_TORQUE_SENSOR_H

#define TORQUE_SENSOR_PIN PC_0
#define TORQUE_SENSOR_2_PIN PC_1
#define PRESSURE_SENSOR_A_PIN PC_2
#define PRESSURE_SENSOR_B_PIN PC_3
#define PRESSURE_SENSOR_S_PIN PC_4
#define PRESSURE_SENSOR_T_PIN PC_5
#define TORQUE_SENSOR_OFFSET 1.65f
#define TORQUE_SENSOR_BUFFER_SIZE 128
#define TORQUE_SENSOR_2_BUFFER_SIZE 128
#define PRESSURE_RANGE 250.0        // [Bar]
#ifdef IC2D_SETUP
#define LOADCELL_250_RANGE  250.0f       // [N] SMT1-250N Load Cell capacity 
#define LOADCELL_250_OFFSET 1.667f       // [V] SMT1-250N Load Cell center voltage
#define LOADCELL_5K_RANGE  5000.0f       // [N] Buster Load Cell capacity 
#define LOADCELL_5K_OFFSET  1.677f       // [V] Buster Load Cell center voltage
#endif

#endif // CONFIG_ANALOG_TORQUE_SENSOR_H