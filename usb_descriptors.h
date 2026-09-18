#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

// One CDC interface: zenoh in the robot, text output in imu_diagnostic.
#define ITF_NUM_CDC0_COMM  0
#define ITF_NUM_CDC0_DATA  1
#define ITF_NUM_TOTAL      2
#define EP_CDC0_NOTIF      0x81
#define EP_CDC0_OUT        0x02
#define EP_CDC0_IN         0x82
#define CDC_IDX_ZENOH      0
#if defined(IMU_DIAGNOSTIC) || defined(MOTOR_TERMINAL)
#define CDC_IDX_DEBUG      0
#endif
#define STR_IDX_LANG       0
#define STR_IDX_MANUF      1
#define STR_IDX_PRODUCT    2
#define STR_IDX_SERIAL     3
#define STR_IDX_CDC0       4

#endif // USB_DESCRIPTORS_H
