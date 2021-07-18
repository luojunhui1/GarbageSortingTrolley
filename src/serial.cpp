//
// Created by root on 2021/6/19.
//
#include "serial.h"

bool wait_uart = false;

/**
 * @brief list all the available uart resources
 * @param none
 * @return uart's name that user appointed
 */
static string get_uart_dev_name() {
    FILE *ls = popen("ls /dev/ttyTHS* --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    pclose(ls);
    return name;
}

/**
 * @brief constructor of Serial class
 * @param nSpeed baud rate
 * @param nEvent verification mode, odd parity, even parity or none
 * @param nBits the length of data
 * @param nStop length of stop bits
 * @return none
 */
Serial::Serial(int speed, char event, int bits, int stop) :
        speed(speed), event(event), bits(bits), stop(stop) {
        if (wait_uart) {
            LOGA("[PORT] : Wait For Serial Be Ready");
            init_port(speed, event, bits, stop);
            LOGA("[PORT] : Port Setup Successfully");
        } else {
            if (init_port(speed, event, bits, stop)) {
                LOGA("[PORT] : Port Setup Successfully");
            } else {
                LOGE("[ERROR] : Port Setup Failed");
            }
        }
}

/**
 * @brief destructor of Serial class
 * @param none
 * @return none
 */
Serial::~Serial() {
    close(fd);
    fd = -1;
}

/**
 * @brief initialize port
 * @param nSpeed_ baud rate
 * @param nEvent_ verification mode, odd parity, even parity or none
 * @param nBits_ the length of data
 * @param nStop_ length of stop bits
 * @return none
 */
bool Serial::init_port(int speed, char event, int bits, int stop){
    string name = PC2STM32;
    if (name.empty()) {
        return false;
    }
    if ((fd = open(name.data(), O_RDWR|O_APPEND|O_SYNC)) < 0) {
        LOGE("[ERROR] : fd Open failed");
        return false;
    }
    return set_opt(fd, speed,event, bits, stop) >= 0;
}

/**
 * @brief package the data needed by lower computer
 * @return none
 */
void Serial::pack(const float distance, const float angle, const uint8_t mission,
                  const uint8_t is_target_found, const uint8_t is_target_close, const uint8_t is_target_in_center,
                  const uint8_t is_get_clamp_position, const uint8_t is_clamp_success, const uint8_t is_get_putback_position,
                  const uint8_t target_type, const uint8_t direction)
{
    unsigned char *p;
    memset(buff, 0, VISION_LENGTH);

    buff[0] = VISION_SOF;
    memcpy(buff + 1, &distance, 4);
    memcpy(buff + 5, &angle, 4);
    memcpy(buff + 9, &mission, 1);
    memcpy(buff + 10, &is_target_found, 1);
    memcpy(buff + 11, &is_target_close, 1);
    memcpy(buff + 12, &is_target_in_center, 1);
    memcpy(buff + 13, &is_get_clamp_position, 1);
    memcpy(buff + 14, &is_clamp_success, 1);
    memcpy(buff + 15, &is_get_putback_position, 1);
    memcpy(buff + 16, &target_type, 1);
    memcpy(buff + 17, &direction, 1);

    buff[VISION_LENGTH - 1] = VISION_TOF;
}

void Serial::pack(State state)
{
    unsigned char *p;
    memset(buff, 0, VISION_LENGTH);

    buff[0] = VISION_SOF;
    memcpy(buff + 1, &state.distance, 4);
    memcpy(buff + 5, &state.angle, 4);
    memcpy(buff + 9, &state.mission_state, 1);
    memcpy(buff + 10, &state.is_target_found, 1);
    memcpy(buff + 11, &state.is_target_close, 1);
    memcpy(buff + 12, &state.is_target_in_center, 1);
    memcpy(buff + 13, &state.is_get_clamp_position, 1);
    memcpy(buff + 14, &state.is_clamp_success, 1);
    memcpy(buff + 15, &state.is_get_putback_position, 1);
    memcpy(buff + 16, &state.target_type, 1);
    memcpy(buff + 17, &state.direction, 1);

    buff[VISION_LENGTH - 1] = VISION_TOF;
}
/**
 * @brief write data to port
 * @return always should be true
 */
bool Serial::write_data(){
    if (fd <= 0){
        if(wait_uart){
            init_port(speed, event, bits, stop);
        }
        return false;
    }
    tcflush(fd, TCOFLUSH);

    if (write(fd, buff, VISION_LENGTH) < 0) {
        close(fd);
        if (wait_uart) {
            init_port(speed, event, bits, stop);
        }
        return false;
    }

    return true;
}

/**
 * @brief read data sent by lower computer
 * @param buffer_ instance should be updated by data sent from lower computer
 * @return on finding the right data in a limited length of received data, return true, if not, return false
 */
bool Serial::read_data(struct ReceiveData &buffer){
    int onceReadCount = 0;

    memset(buff_read,0,VISION_LENGTH);
    read_count = VISION_LENGTH;
    tcflush(fd, TCIFLUSH);

    while(read_count--)
    {
        read(fd, &buff_read[0], 1);
        if(buff_read[0] == 0xA5)break;
    }

    if(read_count == 0)return false;

    read_count = 1;
    while (read_count < VISION_LENGTH - 1)
    {
        try
        {
            onceReadCount = read(fd, (buff_read + read_count), VISION_LENGTH - read_count);
        }
        catch(exception e)
        {
            LOGE("[ERROR] : Data Read Error");
            return false;
        }

        if (onceReadCount < 1)
        {
            return false;
        }

        read_count += onceReadCount;
    }

    if (buff_read[0] != VISION_SOF || buff_read[VISION_LENGTH - 1] != VISION_TOF)
    {
        return false;
    }
    else
    {
        memcpy(&buffer.is_clamp_complete,buff_read + 1,1);
        memcpy(&buffer.is_front_area,buff_read + 2,1);
        memcpy(&buffer.is_putback_complete,buff_read + 3,1);
        return true;
    }

}

/**
 * @brief set port
 * @param fd file or port descriptor
 * @param nSpeed baud rate
 * @param nEvent verification mode, odd parity, even parity or none
 * @param nBits length of data
 * @param nStop length of stop bits
 * @return
 */
int Serial::set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop) {
    termios newtio{}, oldtio{};

    if (tcgetattr(fd, &oldtio) != 0) {
        LOGE("[ERROR] : Serial Setup Failed");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            break;
    }

    switch (nEvent) {
        case 'O':  //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':  //无校验
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }

    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag &= ~OPOST;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 1;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        LOGE("[ERROR] : Serial Setup Error");
        return -1;
    }

    LOGM("[PORT] : Serial Setup Done");

    return 0;
}
