/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 0

#define EXAMPLE_LED_NUMBERS 1
#define EXAMPLE_CHASE_SPEED_MS 1
#define BRIGHTNESS 0.05
// #define MODE_RAINBOW
#define MODE_LIGHT
#define DEFAULT_BLUE 255
#define DEFAULT_GREEN 255
#define DEFAULT_RED 255
// #define PowerRune_Armour
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

static QueueHandle_t gpio_evt_queue = NULL;

// 输入IO： GPIO 1/2/4/5/6/7/10/12/8/9
uint32_t gpio_input[10] = {1, 2, 4, 5, 6, 7, 10, 12, 8, 9};

static void gpio_task_example(void *arg)
{
    static uint32_t io_num, switch_state = 0;

    while (1)
    {
        // 轮询读取GPIO 1/2/4/5/6/7/10/12/8/9，如果有输入，则将输入的GPIO号放入队列
        for (int i = 0; i < 10; i++)
        {
            if (gpio_get_level(gpio_input[i]) == 0 && switch_state == 0)
            {
                io_num = gpio_input[i];
                switch_state = 1;
                vTaskDelay(2 / portTICK_PERIOD_MS); // 延时20ms，防止抖动
                ESP_LOGI(TAG, "RING NO[%" PRIu32 "] intr, val: %d", io_num, 0);
                break;
            }
            else if (gpio_get_level(io_num) == 1 && switch_state == 1)
            {
                switch_state = 0;
                ESP_LOGI(TAG, "RING NO[%" PRIu32 "] intr, val: %d", io_num, 1);
            }
        }
    }
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

void app_main(void)
{

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that is GPIO3
    io_conf.pin_bit_mask = (1ULL << 3) | (1ULL << 1);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(3, 1);
    gpio_set_level(1, 1);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    gpio_set_level(3, 0);

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

#ifdef PowerRune_Armour
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that is GPIO14/21/38/13
    io_conf.pin_bit_mask = (1ULL << 14) | (1ULL << 21) | (1ULL << 38) | (1ULL << 13);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // Write 000 to GPIO14/21/38, 0 to GPIO13
    gpio_set_level(14, 0);
    gpio_set_level(21, 0);
    gpio_set_level(38, 0);
    gpio_set_level(13, 0);
    // INPUT GPIO Setup & ISR Registration
    // INPUT: KEY[1:10] -> GPIO1/2/4/5/6/7/10/12/8/9
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    // GPIO 1/2/4/5/6/7/10/12/8/9
    io_conf.pin_bit_mask = (1ULL << 1) | (1ULL << 2) | (1ULL << 4) | (1ULL << 5) | (1ULL << 6) | (1ULL << 7) | (1ULL << 10) | (1ULL << 12) | (1ULL << 8) | (1ULL << 9);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    // create a queue to handle gpio event from isr
    // gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // // start gpio task
    // ESP_LOGI(TAG, "Starting GPIO_TASK...");
    // // 绑定到第二个核心
    // xTaskCreatePinnedToCore(gpio_task_example, "gpio_task_example", 2048, NULL, tskIDLE_PRIORITY, NULL, 1);

    // // install gpio isr service
    // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // // hook isr handler for specific gpio pin GPIO 1/2/4/5/6/7/10/12/8/9
    // gpio_isr_handler_add(1, gpio_isr_handler, (void *)1);
    // gpio_isr_handler_add(2, gpio_isr_handler, (void *)2);
    // gpio_isr_handler_add(4, gpio_isr_handler, (void *)3);
    // gpio_isr_handler_add(5, gpio_isr_handler, (void *)4);
    // gpio_isr_handler_add(6, gpio_isr_handler, (void *)5);
    // gpio_isr_handler_add(7, gpio_isr_handler, (void *)6);
    // gpio_isr_handler_add(10, gpio_isr_handler, (void *)7);
    // gpio_isr_handler_add(12, gpio_isr_handler, (void *)8);
    // gpio_isr_handler_add(8, gpio_isr_handler, (void *)9);
    // gpio_isr_handler_add(9, gpio_isr_handler, (void *)10);

#endif

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
#ifndef PowerRune_Armour
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 192, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        // .flags = {.with_dma = 1},
    };
#endif

#ifdef PowerRune_Armour
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 1024, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
        .flags = {.with_dma = 1},
    };
#endif
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
#ifdef MODE_RAINBOW
    while (1)
    {
        // GPIO 14/21/38/13 = 000, 001, 010, 011, 100
        for (int j = 0; j < EXAMPLE_LED_NUMBERS; j += 1)
        {
            // Build RGB pixels
            hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
            led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
            led_strip_pixels[j * 3 + 0] = green * BRIGHTNESS;
            led_strip_pixels[j * 3 + 1] = blue * BRIGHTNESS;
            led_strip_pixels[j * 3 + 2] = red * BRIGHTNESS;
        }
        for (int i = 0; i < 5; i++)
        {
            gpio_set_level(14, i & 0x01);
            gpio_set_level(21, (i >> 1) & 0x01);
            gpio_set_level(38, (i >> 2) & 0x01);

            // Flush RGB values to LEDs
            // i=0, size of pixels=301; i=1, pixels=86; i=2, pixels=86; i=3, pixels=30; i=4, pixels=165
            switch (i)
            {
            case 0:
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 903, &tx_config)); // 301*3
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                break;
            case 1:
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 258, &tx_config)); // 86*3
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                break;
            case 2:
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 276, &tx_config)); // 86*3
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                break;
            case 3:
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 180, &tx_config)); // 60*3
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                break;
            case 4:
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 495, &tx_config)); // 165*3
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                break;
            default:
                break;
            }
            // memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            // ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            // ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            // vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 1;
        // vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    }
#endif
#ifdef MODE_LIGHT
    while (1)
    {

        for (int i = 0; i < 5; i++)
        {
            // gpio_set_level(14, i & 0x01);
            // gpio_set_level(21, (i >> 1) & 0x01);
            // gpio_set_level(38, (i >> 2) & 0x01);
            for (int j = 0; j < EXAMPLE_LED_NUMBERS; j++)
            {
                hue = start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = red * BRIGHTNESS;
                led_strip_pixels[j * 3 + 1] = green * BRIGHTNESS;
                led_strip_pixels[j * 3 + 2] = blue * BRIGHTNESS;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            start_rgb += 1;
        }
        vTaskDelay(10);
    }
// break;
#endif
}