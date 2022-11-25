/* 
   This library is a C translation of the [QuadratureDecoder]
   (https://github.com/adamgreen/QuadratureDecoder), 
   all functions are a 1:1 port of the original.
   2022 Unmanned

    Copyright 2021 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#pragma once

#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>


#define DMA_MAX_TRANSFER_COUNT 0xFFFFFFFF
#define DMA_REFRESH_THRESHOLD  0x80000000

typedef struct 
{
    PIO                 m_pio;
    volatile uint32_t   m_counters[NUM_PIO_STATE_MACHINES];
    uint32_t            m_dmaChannels[NUM_PIO_STATE_MACHINES];

} quadrature_decoder;

bool quadrature_decoder_init(quadrature_decoder* qd, PIO pio);
int32_t add_quadrature_decoder(quadrature_decoder* qd, uint32_t pinBase);
void restart_dma_before_it_stops(quadrature_decoder* qd, int32_t index);
int32_t get_count(quadrature_decoder* qd, int32_t index);
