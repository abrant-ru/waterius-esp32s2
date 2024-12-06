#pragma once

#include "soc/sens_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_periph.h"
#include "soc/rtc_io_struct.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

#define LED_S2			(gpio_num_t)15
#define LED_STATE		GPIO_NUM_35
#define BATT_VOL		GPIO_NUM_9
#define BATT_VOL_ADC	ADC_CHANNEL_8
#define BATT_EN			GPIO_NUM_12

enum class power_t {
    Battery         = 0,
    USB             = 1,
};

enum class ulp_event_t {
    NONE            = 0,
    TIME            = 1,
    BUTTON          = 2,
    USB             = 3,
};

struct ulp_config_t {
    bool            use_led;
    bool            use_out;
    uint            debounce_max_count;
};

enum class ulp_channel_type_t {
    Disabled        = 0,
    Discrete        = 1,
    Analog          = 2,
};

struct ulp_channel_t {
    uint16_t        type;
    uint16_t        pulse_count;
    uint            adc_value;
};

struct board_data_t {
    ulp_config_t    config;
    ulp_channel_t   ch0;
    ulp_channel_t   ch1;
	power_t			power;
	bool			usb_connected;
    uint            battery_voltage;
    uint            wake_up_counter;
    uint            wake_up_period;
};

ulp_event_t get_wakeup_event(void);
void deep_sleep(void);
void initialize_pins(void);
void initialize_rtc_pins(void);
void init_ulp_program(void);
void board_read(board_data_t &data);
size_t autoprint(const char *format, ...);

