//
// Created by root on 2021/6/19.
//

#ifndef GARBAGESORTINGTROLLEY_SERIAL_H
#define GARBAGESORTINGTROLLEY_SERIAL_H


#include <iostream>
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <cerrno>      // ERROR Number Definitions
#include <termios.h>
#include <cstring>
#include <cstdio>

#include "log.h"

#define PC2STM32 "/dev/ttyUSB0"//串口位置

/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        20
//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)
//end字节,协议固定为0xA5
#define    VISION_TOF         (0xA6)

/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/
/**    ----------------------------------------------------------------------------------------------------
FIELD  |  A5  |  index  |  distance  |  angle  |  is_get_clamp_position  |  is_get_putback_position  |  mission_state  |  blank  |  A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1  |    1    |     4      |    4    |           1             |             1             |        1        |     6   |   1  |
       ----------------------------------------------------------------------------------------------------
**/
/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/


/**---------------------------------------RECEIVE DATA PROTOCOL----------------------------------------**/
/**    -----------------------------------------------------------------------------------------------------------------------------------------
FIELD  |  head  |  index  |  x  |  y  |  angle  |  is_clamped  |  is_turning  |  blank  |  A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1    |    1    |  4  |  4  |    4    |      1       |      1       |   3    |  1   |
---------------------------------------------------------------------------------------------------------
**/

using namespace std;

/**
 * @brief receive data structure
 */
struct ReceiveData
{
    uint8_t head{};

    uint8_t index{};

    int x = 0;
    int y = 0;
    float angle = 0;

    uint8_t is_clamped = false;

    uint8_t is_turning = false;

    uint8_t tail{};
};

/**
 * @brief SerialPort
 * @param filename 串口名字
 * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
 */
class Serial
{
private:

    int fd;
    int speed;
    char event;
    int bits;
    int stop;
    uint8_t buff[VISION_LENGTH];
    uint8_t  buff_read[100];
    int read_count;
    static int set_opt(int fd, int speed, char event, int bits, int stop);

public:
    explicit Serial(int speed = 115200, char event = 'N', int bits = 8, int stop = 1);
    ~Serial();
    void pack(const uint8_t index, const float distance, const float angle, const uint8_t is_get_clamp_position,
              const uint8_t is_get_putback_position, const uint8_t mission_state);
    bool init_port(int speed = 115200, char  event = 'N', int bits = 8, int stop = 1);
    bool write_data();
    bool read_data(struct ReceiveData& buffer);
};

#endif //GARBAGESORTINGTROLLEY_SERIAL_H
