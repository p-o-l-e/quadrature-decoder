
#include <stdio.h>
#include "quadrature_decoder.h"

#define NCODER_A 9  // Pin Base
#define NCODER_B 10 // Pin Base + 1

quadrature_decoder ncoder;
int32_t ncoder_index;

int32_t quad_encoder_init()
{
    gpio_init(NCODER_A);
    gpio_init(NCODER_B);

    gpio_pull_down(NCODER_A);
    gpio_pull_down(NCODER_B);
    quadrature_decoder_init(&ncoder, pio0);
    return add_quadrature_decoder(&ncoder, NCODER_A);
}

int main(void)
{
    stdio_init_all();
    ncoder_index = quad_encoder_init();
    while (1)
    {
        printf("%ld\n", get_count(&ncoder, ncoder_index));
        sleep_ms(200);
    }
}
