#include <stdio.h>
#include "LEDIndicatorPP.h"

#include "driver/gpio.h"

using enum BlinkStepType;
using enum LedState;

constexpr gpio_num_t LED_0_PIN = GPIO_NUM_12;
constexpr gpio_num_t LED_1_PIN = GPIO_NUM_13;

class LEDController {
public:
    enum class BlinkType {
        BLINK_PRIORITY_HIGHEST,
        BLINK_OFF,
        BLINK_FAST,
        BLINK_NORMAL,
        BLINK_IDLE,
        BLINK_PRIORITY_LOWEST
    };

    static LEDController *getInstance() {
        if (instance == nullptr) {
            instance = new LEDController();
            instance->init();
            instance->start_tmr();
        }
        return instance;
    }

    void init() {
        led_pin = LED_0_PIN;
        gpio_config_t gpio_conf = {0};
        gpio_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_conf.pin_bit_mask = (1ULL << led_pin);
        gpio_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&gpio_conf);
        gpio_set_level(led_pin, 1);

        auto led_mutex = xSemaphoreCreateMutex();
        if (led_mutex == nullptr) {
            ESP_LOGE("LED", "led_mutex create failed");
        }
        const auto lock_func = [](void *user_data) {
            auto led_mutex = static_cast<SemaphoreHandle_t>(user_data);
            if (pdTRUE != xSemaphoreTake(led_mutex, pdMS_TO_TICKS(100))) {
                ESP_LOGW("LED", "led_mutex take timeout");
            }
        };
        const auto unlock_func = [](void *user_data) {
            auto led_mutex = static_cast<SemaphoreHandle_t>(user_data);
            xSemaphoreGive(led_mutex);
        };

        const auto write_pin = [](uint8_t level, void *user_data) {
            auto pin = *reinterpret_cast<gpio_num_t *>(user_data);
            if (pin == GPIO_NUM_NC) { return; }
            gpio_set_level(pin, level);
        };

        led_indicator = new LEDIndicator<BlinkType>(std::make_unique<LEDDriver_GPIO>(
            write_pin,
            1,
            &led_pin
        ));

        led_indicator->addLockFunc(std::move(lock_func), std::move(unlock_func), led_mutex);

        BlinkPattern blink_pattern_idle{
            {
                {HOLD, ON, 100000},
                {LOOP, 0},
            }
        };
        led_indicator->addPattern(BlinkType::BLINK_IDLE, blink_pattern_idle);

        BlinkPattern blink_pattern_off{
            {
                {HOLD, OFF, 2000},
                {STOP, 0},
            }
        };
        led_indicator->addPattern(BlinkType::BLINK_OFF, blink_pattern_off);

        BlinkPattern blink_pattern_normal{
            {
                {HOLD, OFF, 500},
                {HOLD, ON, 500},
                {LOOP, 0},
            }
        };
        led_indicator->addPattern(BlinkType::BLINK_NORMAL, blink_pattern_normal);

        BlinkPattern blink_pattern_fast{
            {
                {HOLD, OFF, 250},
                {HOLD, ON, 250},
                {HOLD, OFF, 250},
                {HOLD, ON, 1000},
                {STOP, 0},
            }
        };
        led_indicator->addPattern(BlinkType::BLINK_FAST, blink_pattern_fast);

        led_indicator->start(BlinkType::BLINK_IDLE);
    }

    void start_tmr() {
        tmr = xTimerCreate(
            "led_i",
            pdMS_TO_TICKS(20),
            true,
            nullptr,
            [](TimerHandle_t timer) {
                instance->update();
            }
        );
        if (tmr) {
            xTimerStart(tmr, portMAX_DELAY);
        }
    }

    void update() {
        led_indicator->update();
    }

    auto getIndicator() { return led_indicator; }

private:
    gpio_num_t led_pin;
    LEDIndicator<BlinkType> *led_indicator = nullptr;
    TimerHandle_t tmr;
    static inline LEDController *instance = nullptr;
};

static const char *const TAG = "main";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "LED Indicator Example");
    LEDController::getInstance();

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "start led off pattern");
    LEDController::getInstance()->getIndicator()->start(LEDController::BlinkType::BLINK_OFF);

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "start normal blink");
    LEDController::getInstance()->getIndicator()->start(LEDController::BlinkType::BLINK_NORMAL);

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "manually stop normal blink");
    LEDController::getInstance()->getIndicator()->stop(LEDController::BlinkType::BLINK_NORMAL);

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "start normal blink and fast blink, it should behave fast blink and rollback to normal");
    LEDController::getInstance()->getIndicator()->start(LEDController::BlinkType::BLINK_FAST);
    LEDController::getInstance()->getIndicator()->start(LEDController::BlinkType::BLINK_NORMAL);
}
