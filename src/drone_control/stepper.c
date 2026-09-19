#define _POSIX_C_SOURCE 200809L

#include "stepper.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#define ACE_GPIO_CHIP "/dev/gpiochip0"
#define ACE_LINES_PER_MOTOR 4

static const unsigned char HALFSTEP_PHASES[8] = {
    0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9
};

struct Stepper {
    int  id;
    int  line_fd;
    int  uses_v2_api;
    int  direction;
    int  phase;
    long position;
};

static int g_dry_run  = 0;
static int g_chip_fd  = -1;
static int g_chip_ref = 0;

void stepper_set_dry_run(int enabled) {
    g_dry_run = enabled ? 1 : 0;
}

static int chip_open(void) {
    if (g_dry_run) {
        g_chip_ref++;
        return 0;
    }

    if (g_chip_fd < 0) {
        g_chip_fd = open(ACE_GPIO_CHIP, O_RDWR | O_CLOEXEC);
        if (g_chip_fd < 0) {
            fprintf(stderr, "Konnte %s nicht oeffnen: %s\n",
                    ACE_GPIO_CHIP, strerror(errno));
            if (errno == EACCES) {
                fprintf(stderr, "Tipp: Benutzer in die Gruppe 'gpio' aufnehmen "
                                "(sudo usermod -aG gpio $USER), dann neu anmelden.\n");
            }
            return -1;
        }
    }
    g_chip_ref++;
    return 0;
}

static void chip_close(void) {
    if (--g_chip_ref <= 0) {
        if (g_chip_fd >= 0) close(g_chip_fd);
        g_chip_fd  = -1;
        g_chip_ref = 0;
    }
}

static int request_lines_v2(const unsigned int pins[4], int *out_fd) {
#ifdef GPIO_V2_GET_LINE_IOCTL
    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof(req));
    for (int i = 0; i < ACE_LINES_PER_MOTOR; i++) req.offsets[i] = pins[i];
    req.num_lines    = ACE_LINES_PER_MOTOR;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    snprintf(req.consumer, sizeof(req.consumer), "ace_stepper");

    if (ioctl(g_chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) return -1;
    *out_fd = req.fd;
    return 0;
#else
    (void)pins;
    (void)out_fd;
    errno = ENOTTY;
    return -1;
#endif
}

static int request_lines_v1(const unsigned int pins[4], int *out_fd) {
    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    for (int i = 0; i < ACE_LINES_PER_MOTOR; i++) req.lineoffsets[i] = pins[i];
    req.lines = ACE_LINES_PER_MOTOR;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    snprintf(req.consumer_label, sizeof(req.consumer_label), "ace_stepper");

    if (ioctl(g_chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) return -1;
    *out_fd = req.fd;
    return 0;
}

static int write_phase(Stepper *s, unsigned char bits) {
    if (g_dry_run) {
        printf("M%d  IN1=%d IN2=%d IN3=%d IN4=%d\n", s->id,
               (bits >> 0) & 1, (bits >> 1) & 1,
               (bits >> 2) & 1, (bits >> 3) & 1);
        return 0;
    }

#ifdef GPIO_V2_GET_LINE_IOCTL
    if (s->uses_v2_api) {
        struct gpio_v2_line_values vals;
        memset(&vals, 0, sizeof(vals));
        vals.mask = 0xF;
        vals.bits = bits;
        return ioctl(s->line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals) < 0 ? -1 : 0;
    }
#endif

    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));
    for (int i = 0; i < ACE_LINES_PER_MOTOR; i++) {
        data.values[i] = (unsigned char)((bits >> i) & 1);
    }
    return ioctl(s->line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0 ? -1 : 0;
}

Stepper *stepper_create(int id, const unsigned int pins[4], int direction) {
    if (chip_open() != 0) return NULL;

    Stepper *s = calloc(1, sizeof(*s));
    if (!s) {
        chip_close();
        return NULL;
    }

    s->id        = id;
    s->line_fd   = -1;
    s->direction = (direction < 0) ? -1 : 1;
    s->phase     = 0;
    s->position  = 0;

    if (!g_dry_run) {
        if (request_lines_v2(pins, &s->line_fd) == 0) {
            s->uses_v2_api = 1;
        } else if (request_lines_v1(pins, &s->line_fd) == 0) {
            s->uses_v2_api = 0;
        } else {
            fprintf(stderr, "Motor %d: GPIO %u,%u,%u,%u nicht anforderbar: %s\n",
                    id, pins[0], pins[1], pins[2], pins[3], strerror(errno));
            if (errno == EBUSY) {
                fprintf(stderr, "Ein anderer Prozess oder ein Overlay belegt den Pin "
                                "(pruefen mit: gpioinfo).\n");
            }
            free(s);
            chip_close();
            return NULL;
        }
    }

    if (write_phase(s, 0x0) != 0) {
        stepper_destroy(s);
        return NULL;
    }
    return s;
}

int stepper_advance(Stepper *s, int dir) {
    if (!s || dir == 0) return -1;

    int turn = (dir > 0 ? 1 : -1) * s->direction;

    s->phase = (s->phase + (turn > 0 ? 1 : 7)) & 7;
    if (write_phase(s, HALFSTEP_PHASES[s->phase]) != 0) return -1;

    s->position += (dir > 0 ? 1 : -1);
    return 0;
}

int stepper_hold(Stepper *s) {
    return s ? write_phase(s, HALFSTEP_PHASES[s->phase]) : -1;
}

int stepper_release(Stepper *s) {
    return s ? write_phase(s, 0x0) : -1;
}

long stepper_position(const Stepper *s) {
    return s ? s->position : 0;
}

void stepper_destroy(Stepper *s) {
    if (!s) return;
    stepper_release(s);
    if (s->line_fd >= 0) close(s->line_fd);
    free(s);
    chip_close();
}
