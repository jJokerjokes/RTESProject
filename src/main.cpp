#include <mbed.h>

DigitalOut led(LED1);

int main() {
    printf("MbedOS demo starting...\n");

    float value = 3.14159f;

    while (1) {

        // Toggle LED
        led = !led;

        printf("LED state: %d\n", led.read());

        // Print a float with 2 decimal places
        printf("The value of pi is approximately: %f\n", value);

        // 1 second delay
        thread_sleep_for(1000);
    }
}