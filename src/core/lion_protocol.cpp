#include <core/lion_protocol.h>
#include <ros/ros.h>
#include <lion_brain.h>
#include <lion_type.h>

using namespace lion;

void Protocol::init_protocol()
{
    // memset((robot_status_t *)&status, 0, sizeof(robot_status_t));
    memset((robot_pose_t *)&pose,     0, sizeof(robot_pose_t));
    // memset((robot_imu_t *)&imu,       0, sizeof(robot_imu_t));
    memset((robot_rc_t *)&rc,         0, sizeof(robot_rc_t));
}

bool Protocol::unpack()
{
    std::lock_guard<std::mutex> lock(rx_frame_mutex);

    switch (serial_rx_buffer[SERIAL_MSG_OFFSET_CMD_ID])
    {
        case SERIAL_CONTROL_CMD_ID:
            memcpy((robot_pose_t *)&pose,
                   serial_rx_buffer + SERIAL_MSG_OFFSET_DATA,
                   data_segment_length);
            ROS_WARN("yaw = %f, pitch = %f", pose.gimbal.yaw.angle, pose.gimbal.pitch.angle);
            return true;

        case SERIAL_IMU_UPLOAD_CMD_ID:
            // memcpy((robot_imu_t *)&imu,
            //        serial_rx_buffer + SERIAL_MSG_OFFSET_DATA,
            //        data_segment_length);
            return true;

        case SERIAL_RC_UPLOAD_CMD_ID:
            memcpy((robot_rc_t *)&rc,
                   serial_rx_buffer + SERIAL_MSG_OFFSET_DATA,
                   data_segment_length);

            robot->getNode()->publish_rc_msg(rc);
            return true;

        default:
            return false;
    }
}

Protocol::Protocol(Robot* robot)
    : robot(robot)
{
    serial_tx_buffer = new uint8_t[SERIAL_MSG_MAX_LENGTH];
    serial_rx_buffer = new uint8_t[SERIAL_MSG_MAX_LENGTH];
    serial_rx_byte   = new uint8_t[1];

    // transmit
    serial_tx_msg_length = SERIAL_MSG_MAX_LENGTH;
    serial_tx_seq = 0;

    // receive
    // clear serial receiving pool
    while(robot->serial->available())
        robot->serial->read(serial_rx_byte, 1);

    init_receive_buffer();

    memset(serial_tx_buffer,  0, SERIAL_MSG_MAX_LENGTH);
    memset(serial_rx_byte,    0, 1);

    init_protocol();
}

Protocol::~Protocol()
{
    if (serial_tx_buffer)
    {
        delete serial_tx_buffer;
        serial_tx_buffer = NULL;
    }

    if (serial_rx_buffer)
    {
        delete serial_rx_buffer;
        serial_rx_buffer = NULL;
    }

    if (serial_rx_byte)
    {
        delete serial_rx_byte;
        serial_rx_byte = NULL;
    }

    robot = NULL;
}

void Protocol::init_receive_buffer()
{
    memset(serial_rx_buffer, 0, SERIAL_MSG_MAX_LENGTH);

    serial_rx_buffer_index = 0;
    serial_rx_msg_length = SERIAL_MSG_MAX_LENGTH;

    frame_header_available = false;
    frame_total_available = false;
}

void Protocol::read()
{
    // calling by a serial_read_thread
    while(robot->serial->available())
    {
        // read bytes and construct and unpack frame
        robot->serial->read(serial_rx_byte, 1);
        serial_msg_unpack_handler(serial_rx_byte[0]);
    }
}

//data 是要存的数据的起始地址，data_length是要传的数据的字节长度
bool Protocol::send(const void* data, uint16_t cmd_id, int data_length)
{
    // pack frame
    int frame_length = serial_msg_pack_handler(data_length, serial_tx_seq++,
                                                cmd_id, data);

    // send bytes
    robot->serial->write(serial_tx_buffer, frame_length);
}

/**
 * Descriptions: CRC8 checksum function
 * Input: Data to check, Stream length, initialized checksum
 * Output: CRC checksum
 */
uint8_t Protocol::get_crc8_check_sum(uint8_t *pchMessage,uint16_t dwLength,uint8_t ucCRC8)
{
    uint8_t ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = crc8_tab[ucIndex];
    }
    return(ucCRC8);
}

/**
 * Descriptions: CRC8 Verify function
 * Input: Data to Verify, Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
 */
bool Protocol::verify_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = get_crc8_check_sum(pchMessage, dwLength - 1, crc8_init);
    return ( ucExpected == pchMessage[dwLength - 1] );
}


/**
 * Descriptions: append CRC8 to the end of data
 * Input: Data to CRC and append, Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
 */
void Protocol::append_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = get_crc8_check_sum ( (uint8_t *)pchMessage, dwLength - 1, crc8_init);
    pchMessage[dwLength - 1] = ucCRC;
}


/**
 * Descriptions: CRC16 checksum function
 * Input: Data to check,Stream length, initialized checksum
 * Output: CRC checksum
 */
uint16_t Protocol::get_crc16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
        return 0xFFFF;
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ crc16_table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/**
 * Descriptions: CRC16 Verify function
 * Input: Data to Verify,Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
 */
bool Protocol::verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
        return 0;
    wExpected = get_crc16_check_sum ( pchMessage, dwLength - 2, crc16_init);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/**
 * Descriptions: append CRC16 to the end of data
 * Input: Data to CRC and append,Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
 */
void Protocol::append_crc16_check_sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
        return;
    wCRC = get_crc16_check_sum ( (uint8_t *)pchMessage, dwLength - 2, crc16_init);
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

/**
  * @brief     串口通信解包函数
  * @param[in] msg: 一个byte数据
  * @retval    none
  */
void Protocol::serial_msg_unpack_handler(const uint8_t msg)
{
    // 遇到帧头SERIAL_MSG_SOF，初始化buffer
    if (msg == SERIAL_MSG_SOF){
        init_receive_buffer();
    }

    // 接受信息直到一帧结束，包括SERIAL_MSG_SOF
    if (serial_rx_buffer_index < SERIAL_MSG_MAX_LENGTH && serial_rx_buffer_index < serial_rx_msg_length)
        serial_rx_buffer[serial_rx_buffer_index++] = msg;

    // 接收完帧头信息，校验帧头，校验通过时获取帧长信息
    if (serial_rx_buffer_index == SERIAL_MSG_OFFSET_CMD_ID)
    {
        frame_header_available = verify_crc8_check_sum(serial_rx_buffer, SERIAL_MSG_OFFSET_CMD_ID);

        if (frame_header_available)
        {
            data_segment_length = (serial_rx_buffer[2] << 8) | (serial_rx_buffer[1] & 0xff);
            serial_rx_msg_length = data_segment_length + SERIAL_MSG_OFFSET_DATA + 2;
        }
    }

    // 接收完全帧信息，校验全帧，只有帧头和全帧校验都通过才对数据帧进行解包
    if (frame_header_available && serial_rx_buffer_index >= serial_rx_msg_length)
    {
        frame_header_available = false;
        frame_total_available = verify_crc16_check_sum(serial_rx_buffer, serial_rx_msg_length);

        if (frame_total_available){
            unpack();
        }
    }

    return;
}

/**
  * @brief     串口通信封包函数
  * @param[in] buffer: 完整的一帧信息
  * @retval    封装好的数据包总长度
  */
int Protocol::serial_msg_pack_handler(uint16_t data_length,
                                      uint8_t  seq,
                                      uint16_t cmd_id,
                                      const void * data)
{
    std::lock_guard<std::mutex> lock(tx_frame_mutex);

    // frame head
    serial_tx_buffer[SERIAL_MSG_OFFSET_SOF]             =  SERIAL_MSG_SOF;
    serial_tx_buffer[SERIAL_MSG_OFFSET_DATA_LENGTH]     =  data_length & 0xff;
    serial_tx_buffer[SERIAL_MSG_OFFSET_DATA_LENGTH + 1] =  data_length >> 8;
    serial_tx_buffer[SERIAL_MSG_OFFSET_SEQ]             =  seq;
    append_crc8_check_sum(serial_tx_buffer, SERIAL_MSG_OFFSET_CRC8 + 1);
    // cmd id
    serial_tx_buffer[SERIAL_MSG_OFFSET_CMD_ID]          =  cmd_id & 0xff;
    serial_tx_buffer[SERIAL_MSG_OFFSET_CMD_ID + 1]      =  cmd_id >> 8;
    // data
    memcpy(serial_tx_buffer + SERIAL_MSG_OFFSET_DATA, data, data_length);
    // frame tail
    append_crc16_check_sum(serial_tx_buffer, SERIAL_MSG_OFFSET_DATA + data_length + 2);
    return (SERIAL_MSG_OFFSET_DATA + data_length + 2);
}


