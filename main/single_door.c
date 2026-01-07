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
#include "driver/gpio.h"

/* Motor control values */
#define MOTOR_FORWARD  1
#define MOTOR_OFF      0
#define MOTOR_REVERSE (-1)

#define TIMEOUT 5000  /* 5 seconds for open/close */

/* GPIO pin assignments (ESP32-C3 safe defaults; adjust to your board) */
#define MOTOR_CW_GPIO     2
#define MOTOR_CCW_GPIO    3
#define MOTOR_STOP_GPIO   4

#define BTN_OPEN_GPIO     5
#define BTN_CLOSE_GPIO    6
#define BTN_STOP_GPIO     7

#define DEBOUNCE_US       50000  /* 50 ms debounce */

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

/* Forward declarations */
static void apply_motor_outputs(void);
static void read_buttons(void);
static void gpio_init_io(void);

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

/* Initialize motor outputs and button inputs */
static void gpio_init_io(void)
{
    gpio_config_t io_conf = {0};

    /* Motor output pins */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_CW_GPIO) |
                           (1ULL << MOTOR_CCW_GPIO) |
                           (1ULL << MOTOR_STOP_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    /* Initialize to STOP */
    gpio_set_level(MOTOR_CW_GPIO, 0);
    gpio_set_level(MOTOR_CCW_GPIO, 0);
    gpio_set_level(MOTOR_STOP_GPIO, 1);

    /* Button input pins (active-low with pull-ups) */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BTN_OPEN_GPIO) |
                           (1ULL << BTN_CLOSE_GPIO) |
                           (1ULL << BTN_STOP_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

/* Drive motor GPIOs based on current motor command */
static void apply_motor_outputs(void)
{
    if (motor > 0) {
        /* FORWARD (CW) */
        gpio_set_level(MOTOR_CW_GPIO, 1);
        gpio_set_level(MOTOR_CCW_GPIO, 0);
        gpio_set_level(MOTOR_STOP_GPIO, 0);
    } else if (motor < 0) {
        /* REVERSE (CCW) */
        gpio_set_level(MOTOR_CW_GPIO, 0);
        gpio_set_level(MOTOR_CCW_GPIO, 1);
        gpio_set_level(MOTOR_STOP_GPIO, 0);
    } else {
        /* STOP */
        gpio_set_level(MOTOR_CW_GPIO, 0);
        gpio_set_level(MOTOR_CCW_GPIO, 0);
        gpio_set_level(MOTOR_STOP_GPIO, 1);
    }
}

/* Poll buttons with debounce and update command */
static void read_buttons(void)
{
    uint64_t now = (uint64_t)esp_timer_get_time();

    /* Per-button debounce state */
    typedef struct {
        int gpio;
        bool last_raw;
        bool latched;      /* true while button is held to avoid repeats */
        uint64_t t_change; /* last time raw state changed */
        command_t cmd;
    } btn_t;

    static btn_t btns[3] = {
        { BTN_OPEN_GPIO,  true, false, 0, CMD_OPEN },
        { BTN_CLOSE_GPIO, true, false, 0, CMD_CLOSE },
        { BTN_STOP_GPIO,  true, false, 0, CMD_STOP }
    };

    /* Read each button (active-low) */
    for (int i = 0; i < 3; ++i) {
        bool raw = gpio_get_level(btns[i].gpio) ? true : false; /* true = not pressed */
        if (raw != btns[i].last_raw) {
            btns[i].last_raw = raw;
            btns[i].t_change = now;
        } else {
            if ((now - btns[i].t_change) >= DEBOUNCE_US) {
                bool pressed = (raw == false);
                if (pressed && !btns[i].latched) {
                    /* Priority: STOP overrides others; process after loop */
                    btns[i].latched = true;
                } else if (!pressed && btns[i].latched) {
                    btns[i].latched = false;
                }
            }
        }
    }

    /* Apply command priority: STOP > OPEN > CLOSE */
    if (btns[2].latched) {
        command = CMD_STOP;
    } else if (btns[0].latched) {
        command = CMD_OPEN;
    } else if (btns[1].latched) {
        command = CMD_CLOSE;
    }
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

    /* Reflect motor command on GPIOs */
    apply_motor_outputs();
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
    gpio_init_io();
    timer_init();

    while (1) {
        door_state_machine();
        read_buttons();
    }
}
