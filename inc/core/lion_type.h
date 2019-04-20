#ifndef _LION_TYPE_H_
#define _LION_TYPE_H_

#include <cstdint>

namespace lion
{

#define SERIAL_MSG_SOF                      0xFE
#define SERIAL_MSG_MAX_LENGTH               50

#define SERIAL_MSG_OFFSET_SOF               0
#define SERIAL_MSG_OFFSET_DATA_LENGTH       1
#define SERIAL_MSG_OFFSET_SEQ               3
#define SERIAL_MSG_OFFSET_CRC8              4
#define SERIAL_MSG_OFFSET_CMD_ID            5
#define SERIAL_MSG_OFFSET_DATA              7

#define SERIAL_ROBOT_STATUS_CMD_ID          0x0001
#define SERIAL_POSE_UPLOAD_CMD_ID           0x0002
#define SERIAL_CONTROL_CMD_ID               0x0003
#define SERIAL_GIMBAL_DOWNLOAD_CMD_ID       0x0005
#define SERIAL_RC_UPLOAD_CMD_ID             0x0006
#define SERIAL_RC_DOWNLOAD_CMD_ID           0x0007
#define SERIAL_IMU_UPLOAD_CMD_ID            0x0008
#define SERIAL_IMU_DOWNLOAD_CMD_ID          0x0009
#define SERIAL_CUSTOM_DATA_CMD_ID           0x0020


#define MAX_THREAD_TYPE_NUM 2

#define SERIAL_READ_THREAD  0
#define VIRTUAL_RC_THREAD   1

#define VIRTUAL_RC_COMMAND_LENGTH 18

#define INPUT_RC         0
#define INPUT_PC         1
#define INPUT_NONE       2

#define ASSIST_SEMI_AUTO 0
#define ASSIST_FULL_AUTO 1
#define ASSIST_MANUAL    2

typedef enum {
    RECEIVED_EMPTY_MSG = 0,
    RECEIVED_POSE_MSG,
    RECEIVED_SHOOTER_MSG,
    RECEIVED_STATUS_MSG,
    RECEIVED_RC_MSG,
    RECEIVED_IMU_MSG,
    RECEIVED_UWB_MSG
} receive_msg_type_e;


typedef struct
{
    float x;
    float y;
    float z;
} vector3f_t;

typedef struct
{
    // error_list
    uint16_t error_code;
} robot_status_t;

typedef struct
{
    struct {
        vector3f_t speed;
    } chassis;

    struct {
        struct {
            float angle;
            float speed;
        } yaw;

        struct {
            float angle;
            float speed;
        } pitch;
    } gimbal;

    uint8_t control_mode;
} robot_pose_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

    int16_t yaw;
    int16_t pitch;

    uint8_t input_mode;
    uint8_t assist_mode;
} robot_rc_t;

typedef struct
{
	vector3f_t acc;
	vector3f_t gyro;
	vector3f_t mag;
} robot_imu_t;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;

typedef struct
{
    quaternion_t q;
    vector3f_t   acc;
    vector3f_t   gyro;
} imu_msg_t;

typedef struct
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    /* left and right lever information */
    uint8_t last_sw1;
    uint8_t last_sw2;
    uint8_t sw1;
    uint8_t sw2;

    /* mouse movement and button information */
     struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;

    /* keyboard key information */
     union
    {
        uint16_t code;
         struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } keyboard;
} rc_msg_t;

}

#endif
