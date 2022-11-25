cmake_minimum_required(VERSION 3.12)

add_library(quadrature_decoder INTERFACE)

target_sources(quadrature_decoder INTERFACE

    ${CMAKE_CURRENT_LIST_DIR}/quadrature_decoder.c

)

pico_generate_pio_header(quadrature_decoder ${CMAKE_CURRENT_LIST_DIR}/quadrature_decoder.pio)
