/*
 * Single Door Control System (ESP-IDF with Timer ISR)
 *
 * A finite state machine that manages a single-motor door system.
 * Uses ESP Timer ISR to set timeout flags; FSM checks flags without polling.
 * Cleaner separation: timer logic in ISR, state logic in FSM.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

/* Motor control values */
#define MOTOR_FORWARD  1
#define MOTOR_OFF      0
#define MOTOR_REVERSE (-1)

#define TIMEOUT 5000  /* 5 seconds for open/close */

/* Door states */
typedef enum {
    DOOR_CLOSED,
    DOOR_OPENING,
    DOOR_OPENED,
    DOOR_CLOSING
} door_state_t;

/* Command types */
typedef enum {
    CMD_STOP,
    CMD_OPEN,
    CMD_CLOSE
} command_t;

/* Global state */
static door_state_t door_state = DOOR_CLOSED;
static command_t command = CMD_STOP;
static volatile bool timeout_flag = false;  /* Set by ISR, checked by FSM */
static int32_t motor = MOTOR_OFF;

/* Timer handle */
static esp_timer_handle_t timer_handle = NULL;

/**
 * Timer callback (runs in dedicated ISR context)
 * Sets the timeout flag to signal FSM
 */
static void timer_callback(void *arg)
{
    timeout_flag = true;
}

/**
 * Initialize the timer
 * Must be called once during startup
 */
static void timer_init(void)
{
    if (timer_handle != NULL) {
        return;  /* Already initialized */
    }

    esp_timer_create_args_t timer_args = {
        .callback = timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  /* Runs in dedicated task, not ISR */
        .name = "door_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
}

/**
 * Set a timeout (non-blocking, ISR-driven)
 * @param milliseconds Duration in milliseconds
 */
static void set_timer(int32_t milliseconds)
{
    if (timer_handle == NULL) {
        timer_init();
    }

    /* Stop any running timer */
    esp_timer_stop(timer_handle);

    /* Reset flag before starting new timer */
    timeout_flag = false;

    /* Start one-shot timer */
    ESP_ERROR_CHECK(esp_timer_start_once(timer_handle, (uint64_t)milliseconds * 1000));
}

/**
 * Stop the timer and clear flag
 */
static void stop_timer(void)
{
    if (timer_handle != NULL) {
        esp_timer_stop(timer_handle);
    }
    timeout_flag = false;
}

/**
 * Log state and motor changes
 */
static void log_state_changes(void)
{
    /* Log motor state changes */
    static int32_t last_motor = MOTOR_OFF;
    if (motor != last_motor) {
        const char *motor_str = (motor > 0) ? "FORWARD" : (motor < 0) ? "REVERSE" : "OFF";
        printf("[%lld ms] Motor: %s\n", esp_timer_get_time() / 1000, motor_str);
        last_motor = motor;
    }

    /* Log state changes */
    static door_state_t last_state = DOOR_CLOSED;
    if (door_state != last_state) {
        const char *state_str[] = {"CLOSED", "OPENING", "OPENED", "CLOSING"};
        printf("[%lld ms] Door State: %s\n", esp_timer_get_time() / 1000, state_str[door_state]);
        last_state = door_state;
    }
}

/**
 * Main control loop for single door state machine.
 * Checks timeout_flag set by ISR instead of polling.
 */
static void door_state_machine(void)
{
    switch (door_state) {
        case DOOR_CLOSED:
            if (command == CMD_OPEN) {
                set_timer(TIMEOUT);  
                motor = MOTOR_FORWARD;
                door_state = DOOR_OPENING;
            }
            break;

        case DOOR_OPENING:
            if (command == CMD_CLOSE) {
                stop_timer();
                motor = MOTOR_REVERSE;
                door_state = DOOR_CLOSING;
            }
            if (timeout_flag) {
                motor = MOTOR_OFF;
                door_state = DOOR_OPENED;
                timeout_flag = false;
            }
            break;

        case DOOR_OPENED:
            if (command == CMD_CLOSE) {
                set_timer(TIMEOUT); 
                motor = MOTOR_REVERSE;
                door_state = DOOR_CLOSING;
            }
            break;

        case DOOR_CLOSING:
            if (command == CMD_OPEN) { 
                stop_timer();
                motor = MOTOR_FORWARD;
                door_state = DOOR_OPENING;
            }
            if (timeout_flag) {
                motor = MOTOR_OFF;
                door_state = DOOR_CLOSED;
                timeout_flag = false;
            }
            break;

        default: 
            door_state = DOOR_CLOSED;
            motor = MOTOR_OFF;
            stop_timer();
            break;
    }

    /* Log any state or motor changes */
    log_state_changes();
}

/**
 * Get the current motor command value
 * @return MOTOR_FORWARD, MOTOR_OFF, or MOTOR_REVERSE
 */
int32_t get_motor_command(void)
{
    return motor;
}

/**
 * Get the current door state
 * @return Current door state enum
 */
door_state_t get_door_state(void)
{
    return door_state;
}

/**
 * Set the door command (open/close/stop)
 * @param cmd Command to execute
 */
void set_door_command(command_t cmd)
{
    command = cmd;
}

/**
 * Main application entry point
 */
void app_main(void)
{
    printf("\n=== Single Door Control System ===\n");
    printf("System initialized\n\n");

    /* Initialize timer */
    timer_init();

    while (1) {
        /* Process state machine (checks timeout_flag set by ISR) */
        door_state_machine();

        /* TODO: Add your command input logic here */
        /* Example: Read button/sensor inputs and call set_door_command() */

        /* Non-blocking small delay to prevent CPU lockup */
        esp_rom_delay_us(100);
    }
}
