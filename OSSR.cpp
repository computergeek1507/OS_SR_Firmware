#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

uint status[4]{0,0,0,0};

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    status[gpio -2] = 1;
}

#include "pico/stdlib.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "hardware/i2c.h"

int main()
{
    stdio_init_all();

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);

    // Init i2c1 controller
    i2c_init(i2c1, 1000000);
    // Set up pins 26 and 27
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    // If you don't do anything before initializing a display pi pico is too fast and starts sending
    // commands before the screen controller had time to set itself up, so we add an artificial delay for
    // ssd1306 to set itself up
    sleep_ms(250);

    // Create a new display object at address 0x3D and size of 128x64
    pico_ssd1306::SSD1306 display = pico_ssd1306::SSD1306(i2c1, 0x3C, pico_ssd1306::Size::W128xH64);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    display.setOrientation(0);


   // puts("Hello, world!");


    //const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    const uint LED_PIN2 = 6U;

    const uint DATA1 = 10U;
    const uint DATA2 = 11U;
    const uint DATA3 = 12U;
    const uint DATA4 = 13U;

    pico_ssd1306::drawText(&display, font_8x8, "OS SM REC", 25 ,15);
    pico_ssd1306::drawText(&display, font_8x8, "Normal Mode", 25 ,25);

    // Send buffer to the display
    display.sendBuffer();


    gpio_set_irq_enabled_with_callback(DATA1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    uint offTime{0U};
    //gpio_init(LED_PIN);
    //gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(LED_PIN2);
    gpio_init(DATA1);
    gpio_init(DATA2);
    gpio_init(DATA3);
    gpio_init(DATA4);
    gpio_set_dir(LED_PIN2, GPIO_OUT);
    gpio_set_dir(DATA1, GPIO_OUT);
    gpio_set_dir(DATA2, GPIO_OUT);
    gpio_set_dir(DATA3, GPIO_OUT);
    gpio_set_dir(DATA4, GPIO_OUT);

    gpio_put(LED_PIN2, true);
    gpio_put(DATA1, 0);
    gpio_put(DATA1, 0);
    gpio_put(DATA1, 0);
    gpio_put(DATA1, 0);

    while (true) {
        //gpio_put(LED_PIN, 1);
        //gpio_put(LED_PIN2, 1);
        //sleep_ms(250);
        //gpio_put(LED_PIN, 0);
        //gpio_put(LED_PIN2, 0);
        //sleep_ms(250);
        //if(status[0]==1)
        //{
        //    gpio_put(LED_PIN2, 1);
        //    offTime++;
        //}
        //else{
        //    gpio_put(LED_PIN2, 0);
        //}
        //if(offTime>20){

        //    status[0]=0;
        //    offTime=0;
        //}
        //status[0]=0;
        gpio_put(LED_PIN2, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN2, 0);
        sleep_ms(1000);
    }

    return 0;
}
