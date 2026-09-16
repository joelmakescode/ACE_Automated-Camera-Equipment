#define _POSIX_C_SOURCE 200809L

#include "motion.h"
#include "stepper.h"

#include <stdio.h>
#include <time.h>

static Stepper *g_motors[ACE_MOTOR_COUNT];
static int      g_initialized = 0;

static long g_steps_total[ACE_MOTOR_COUNT];
static long g_steps_left[ACE_MOTOR_COUNT];
static int  g_direction[ACE_MOTOR_COUNT];
static long g_bresenham_error[ACE_MOTOR_COUNT];

static long g_lead_steps = 0;
static long g_lead_done  = 0;

static unsigned int    g_step_delay_us = ACE_DEFAULT_STEP_DELAY_US;
static struct timespec g_next_step_at;
static int             g_running = 0;

static void deadline_add_us(struct timespec *ts, unsigned int us) {
    ts->tv_nsec += (long)us * 1000L;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec  += 1;
    }
}

static long elapsed_us(const struct timespec *a, const struct timespec *b) {
    return (a->tv_sec - b->tv_sec) * 1000000L + (a->tv_nsec - b->tv_nsec) / 1000L;
}

static void reschedule_if_far_behind(const struct timespec *now) {
    if (elapsed_us(now, &g_next_step_at) > (long)g_step_delay_us) {
        g_next_step_at = *now;
        deadline_add_us(&g_next_step_at, g_step_delay_us);
    }
}

int motion_init(void) {
    if (g_initialized) return 0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        g_motors[i] = stepper_create(i, ACE_MOTOR_PINS[i]);
        if (!g_motors[i]) {
            fprintf(stderr, "motion_init: Motor %d (%s) fehlgeschlagen.\n",
                    i, ACE_MOTOR_NAMES[i]);
            motion_shutdown();
            return -1;
        }
    }

    g_initialized = 1;
    return 0;
}

void motion_shutdown(void) {
    g_running = 0;
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        stepper_destroy(g_motors[i]);
        g_motors[i] = NULL;
    }
    g_initialized = 0;
}

int motion_start(const long steps[ACE_MOTOR_COUNT], unsigned int step_delay_us) {
    if (!g_initialized) return -1;

    if (step_delay_us < ACE_MIN_STEP_DELAY_US) {
        fprintf(stderr, "Schrittzeit %u us zu kurz, auf %u us begrenzt.\n",
                step_delay_us, ACE_MIN_STEP_DELAY_US);
        step_delay_us = ACE_MIN_STEP_DELAY_US;
    }

    g_lead_steps = 0;
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        long n = steps[i];
        g_direction[i]   = (n >= 0) ? 1 : -1;
        g_steps_total[i] = (n >= 0) ? n : -n;
        g_steps_left[i]  = g_steps_total[i];
        if (g_steps_total[i] > g_lead_steps) g_lead_steps = g_steps_total[i];
    }

    if (g_lead_steps == 0) {
        g_running = 0;
        return 0;
    }

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        g_bresenham_error[i] = g_lead_steps / 2;
    }

    g_lead_done     = 0;
    g_step_delay_us = step_delay_us;
    g_running       = 1;
    clock_gettime(CLOCK_MONOTONIC, &g_next_step_at);
    return 0;
}

int motion_tick(void) {
    if (!g_running) return 1;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (elapsed_us(&g_next_step_at, &now) > 0) return 0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        if (g_steps_left[i] <= 0) continue;

        g_bresenham_error[i] += g_steps_total[i];
        if (g_bresenham_error[i] >= g_lead_steps) {
            g_bresenham_error[i] -= g_lead_steps;
            if (stepper_advance(g_motors[i], g_direction[i]) != 0) {
                fprintf(stderr, "Motor %d (%s): Schritt fehlgeschlagen.\n",
                        i, ACE_MOTOR_NAMES[i]);
                g_running = 0;
                return -1;
            }
            g_steps_left[i]--;
        }
    }

    g_lead_done++;
    if (g_lead_done >= g_lead_steps) {
        g_running = 0;
        return 1;
    }

    deadline_add_us(&g_next_step_at, g_step_delay_us);
    reschedule_if_far_behind(&now);
    return 0;
}

void motion_wait_next(void) {
    if (!g_running) return;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_next_step_at, NULL);
}

int motion_run(const long steps[ACE_MOTOR_COUNT], unsigned int step_delay_us) {
    if (motion_start(steps, step_delay_us) != 0) return -1;

    for (;;) {
        int r = motion_tick();
        if (r < 0) return -1;
        if (r > 0) return 0;
        motion_wait_next();
    }
}

bool motion_busy(void) {
    return g_running != 0;
}

long motion_position(int motor) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0;
    return stepper_position(g_motors[motor]);
}

void motion_hold(void) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) stepper_hold(g_motors[i]);
}

void motion_release(void) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) stepper_release(g_motors[i]);
}
