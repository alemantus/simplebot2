#include <stdio.h>
#include "pico/stdlib.h"
 
#define LED_PIN 20
int LED_STATE = 0;
 
int main()
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    stdio_init_all();
 
    while (1)
    {
        printf("Enter 1 or 0 To Turn LED ON or OFF:\n");
        scanf("%d", &LED_STATE);
        gpio_put(LED_PIN, LED_STATE);
        sleep_ms(100);
    }
}