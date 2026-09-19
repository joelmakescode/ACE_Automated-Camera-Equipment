#define _POSIX_C_SOURCE 200809L

#include "motion.h"
#include "stepper.h"

#include <pthread.h>
#include <stddef.h>
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

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_worker;
static volatile int    g_worker_active = 0;

static MotionIdleHook volatile g_idle_hook = NULL;

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

static void sleep_ms(long ms) {
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

int motion_init(void) {
    pthread_mutex_lock(&g_lock);
    if (g_initialized) {
        pthread_mutex_unlock(&g_lock);
        return 0;
    }

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        g_motors[i] = stepper_create(i, ACE_MOTOR_PINS[i], ACE_MOTOR_DIRECTION[i]);
        if (!g_motors[i]) {
            fprintf(stderr, "motion_init: Motor %d (%s) fehlgeschlagen.\n",
                    i, ACE_MOTOR_NAMES[i]);
            for (int j = 0; j < ACE_MOTOR_COUNT; j++) {
                stepper_destroy(g_motors[j]);
                g_motors[j] = NULL;
            }
            pthread_mutex_unlock(&g_lock);
            return -1;
        }
    }

    g_initialized = 1;
    pthread_mutex_unlock(&g_lock);
    return 0;
}

void motion_shutdown(void) {
    motion_thread_stop();

    pthread_mutex_lock(&g_lock);
    g_running = 0;
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        stepper_destroy(g_motors[i]);
        g_motors[i] = NULL;
    }
    g_initialized = 0;
    pthread_mutex_unlock(&g_lock);
}

int motion_start(const long steps[ACE_MOTOR_COUNT], unsigned int step_delay_us) {
    pthread_mutex_lock(&g_lock);

    if (!g_initialized) {
        pthread_mutex_unlock(&g_lock);
        return -1;
    }

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
        pthread_mutex_unlock(&g_lock);
        return 0;
    }

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        g_bresenham_error[i] = g_lead_steps / 2;
    }

    g_lead_done     = 0;
    g_step_delay_us = step_delay_us;
    g_running       = 1;
    clock_gettime(CLOCK_MONOTONIC, &g_next_step_at);

    pthread_mutex_unlock(&g_lock);
    return 0;
}

int motion_tick(void) {
    pthread_mutex_lock(&g_lock);

    if (!g_running) {
        pthread_mutex_unlock(&g_lock);
        return 1;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (elapsed_us(&g_next_step_at, &now) > 0) {
        pthread_mutex_unlock(&g_lock);
        return 0;
    }

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        if (g_steps_left[i] <= 0) continue;

        g_bresenham_error[i] += g_steps_total[i];
        if (g_bresenham_error[i] >= g_lead_steps) {
            g_bresenham_error[i] -= g_lead_steps;
            if (stepper_advance(g_motors[i], g_direction[i]) != 0) {
                fprintf(stderr, "Motor %d (%s): Schritt fehlgeschlagen.\n",
                        i, ACE_MOTOR_NAMES[i]);
                g_running = 0;
                pthread_mutex_unlock(&g_lock);
                return -1;
            }
            g_steps_left[i]--;
        }
    }

    g_lead_done++;
    if (g_lead_done >= g_lead_steps) {
        g_running = 0;
        pthread_mutex_unlock(&g_lock);
        return 1;
    }

    deadline_add_us(&g_next_step_at, g_step_delay_us);
    reschedule_if_far_behind(&now);

    pthread_mutex_unlock(&g_lock);
    return 0;
}

void motion_wait_next(void) {
    struct timespec deadline;
    int running;

    pthread_mutex_lock(&g_lock);
    running  = g_running;
    deadline = g_next_step_at;
    pthread_mutex_unlock(&g_lock);

    if (!running) return;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
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

void motion_abort(void) {
    pthread_mutex_lock(&g_lock);
    g_running = 0;
    pthread_mutex_unlock(&g_lock);
}

static void *worker_main(void *arg) {
    (void)arg;

    while (g_worker_active) {
        if (motion_busy()) {
            motion_tick();
            motion_wait_next();
            continue;
        }

        MotionIdleHook hook = g_idle_hook;
        if (hook) hook();

        if (!motion_busy()) sleep_ms(2);
    }
    return NULL;
}

void motion_set_idle_hook(MotionIdleHook hook) {
    g_idle_hook = hook;
}

int motion_thread_start(void) {
    if (g_worker_active) return 0;

    g_worker_active = 1;
    if (pthread_create(&g_worker, NULL, worker_main, NULL) != 0) {
        g_worker_active = 0;
        fprintf(stderr, "motion_thread_start: pthread_create fehlgeschlagen.\n");
        return -1;
    }
    return 0;
}

void motion_thread_stop(void) {
    if (!g_worker_active) return;

    g_worker_active = 0;
    pthread_join(g_worker, NULL);
}

bool motion_busy(void) {
    pthread_mutex_lock(&g_lock);
    int running = g_running;
    pthread_mutex_unlock(&g_lock);
    return running != 0;
}

long motion_position(int motor) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0;

    pthread_mutex_lock(&g_lock);
    long pos = stepper_position(g_motors[motor]);
    pthread_mutex_unlock(&g_lock);
    return pos;
}

void motion_positions(long out[ACE_MOTOR_COUNT]) {
    if (!out) return;

    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        out[i] = stepper_position(g_motors[i]);
    }
    pthread_mutex_unlock(&g_lock);
}

void motion_hold(void) {
    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) stepper_hold(g_motors[i]);
    pthread_mutex_unlock(&g_lock);
}

void motion_release(void) {
    pthread_mutex_lock(&g_lock);
    g_running = 0;
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) stepper_release(g_motors[i]);
    pthread_mutex_unlock(&g_lock);
}
