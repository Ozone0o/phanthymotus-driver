#include "hal_uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <limits.h>

/* DJI USB Vendor ID */
#define DJI_USB_VID 0x2CA3
/* FTDI Vendor ID (used on E-Port dev board) */
#define FTDI_USB_VID 0x0403

static int s_uart_fd = -1;
static uint16_t s_vid = 0;
static uint16_t s_pid = 0;

/* ── Baud rate conversion ────────────────────────────────────────────── */

static speed_t _baud_to_speed(uint32_t baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:      return B921600;
    }
}

/* ── Auto-detect USB VID/PID from sysfs ─────────────────────────────── */

static int _detect_vid_pid(const char *device) {
    char path[PATH_MAX], sysfs_path[PATH_MAX], sysfs_device[PATH_MAX];
    char resolved_device[PATH_MAX], buf[16];
    const char *dev_name;
    int got_vid = 0;
    int got_pid = 0;

    if (!device) return -1;

    if (realpath(device, resolved_device) != NULL) {
        dev_name = strrchr(resolved_device, '/');
        dev_name = dev_name ? dev_name + 1 : resolved_device;
    } else {
        dev_name = strrchr(device, '/');
        dev_name = dev_name ? dev_name + 1 : device;
    }

    if (snprintf(sysfs_path, sizeof(sysfs_path),
                 "/sys/class/tty/%s/device", dev_name) < 0 ||
        realpath(sysfs_path, sysfs_device) == NULL) {
        return -1;
    }

    for (int depth = 0; depth < 8; ++depth) {
        int n = snprintf(path, sizeof(path), "%s/idVendor", sysfs_device);
        if (n >= 0 && (size_t)n < sizeof(path)) {
            FILE *f = fopen(path, "r");
            if (f) {
                if (fgets(buf, sizeof(buf), f)) {
                    s_vid = (uint16_t)strtoul(buf, NULL, 16);
                    got_vid = 1;
                }
                fclose(f);
            }
        }

        n = snprintf(path, sizeof(path), "%s/idProduct", sysfs_device);
        if (n >= 0 && (size_t)n < sizeof(path)) {
            FILE *f = fopen(path, "r");
            if (f) {
                if (fgets(buf, sizeof(buf), f)) {
                    s_pid = (uint16_t)strtoul(buf, NULL, 16);
                    got_pid = 1;
                }
                fclose(f);
            }
        }

        if (got_vid && got_pid) break;
        char *slash = strrchr(sysfs_device, '/');
        if (!slash || slash == sysfs_device) break;
        *slash = '\0';
    }

    printf("[hal_uart] detected VID=0x%04X PID=0x%04X for %s\n", s_vid, s_pid, device);
    return (got_vid && got_pid) ? 0 : -1;
}

/* ── Public API ────────────────────────────────────────────────────── */

int HalUart_Init(const char *device, uint32_t baudRate) {
    struct termios tty;

    printf("[hal_uart] opening %s @ %u baud\n", device, baudRate);

    s_uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (s_uart_fd < 0) {
        printf("[hal_uart] open failed: %s (errno=%d)\n", strerror(errno), errno);
        return -1;
    }

    /* Clear O_NONBLOCK after open (we handle timeout via select) */
    int flags = fcntl(s_uart_fd, F_GETFL, 0);
    fcntl(s_uart_fd, F_SETFL, flags & ~O_NONBLOCK);

    /* Configure serial port */
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(s_uart_fd, &tty) != 0) {
        printf("[hal_uart] tcgetattr failed: %s\n", strerror(errno));
        close(s_uart_fd);
        s_uart_fd = -1;
        return -1;
    }

    speed_t speed = _baud_to_speed(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    /* 8N1, no flow control */
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;

    /* Raw mode — no echo, no signals, no canonical */
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                      ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    /* Minimum 1 byte, 1 decisecond timeout */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(s_uart_fd, TCSANOW, &tty) != 0) {
        printf("[hal_uart] tcsetattr failed: %s\n", strerror(errno));
        close(s_uart_fd);
        s_uart_fd = -1;
        return -1;
    }

    /* Flush buffers */
    tcflush(s_uart_fd, TCIOFLUSH);

    /* Detect USB VID/PID from sysfs */
    _detect_vid_pid(device);

    printf("[hal_uart] initialized: fd=%d, baud=%u\n", s_uart_fd, baudRate);
    return 0;
}

int HalUart_Write(const uint8_t *data, uint32_t len) {
    if (s_uart_fd < 0) return -1;

    ssize_t written = 0;
    while ((uint32_t)written < len) {
        ssize_t n = write(s_uart_fd, data + written, len - written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            printf("[hal_uart] write error: %s\n", strerror(errno));
            return -1;
        }
        written += n;
    }
    return (int)written;
}

int HalUart_Read(uint8_t *buf, uint32_t len, uint32_t timeout_ms) {
    if (s_uart_fd < 0) return -1;

    fd_set fds;
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&fds);
    FD_SET(s_uart_fd, &fds);

    int ret = select(s_uart_fd + 1, &fds, NULL, NULL, timeout_ms > 0 ? &tv : NULL);
    if (ret <= 0) {
        return (ret == 0) ? 0 : -1;  /* 0 = timeout, -1 = error */
    }

    ssize_t n = read(s_uart_fd, buf, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) return 0;
        return -1;
    }
    return (int)n;
}

int HalUart_GetDeviceInfo(T_HalUartDeviceInfo *info) {
    if (!info) return -1;
    info->vid = s_vid;
    info->pid = s_pid;
    return 0;
}

void HalUart_Close(void) {
    if (s_uart_fd >= 0) {
        close(s_uart_fd);
        s_uart_fd = -1;
        printf("[hal_uart] closed\n");
    }
}
