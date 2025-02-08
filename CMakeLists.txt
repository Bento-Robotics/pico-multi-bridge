cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(rp2040-can-mcp2515 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

message("CAN-2040 support added..")

FILE(GLOB_RECURSE can2040_sources ${CMAKE_CURRENT_LIST_DIR}/can2040/src/*.c)
add_library(can2040 INTERFACE)
target_sources(can2040 INTERFACE ${can2040_sources})

target_include_directories(can2040 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/can2040/src)
target_link_libraries(can2040 INTERFACE pico_base_headers hardware_pio hardware_dma cmsis_core)

pico_generate_pio_header(can2040 ${CMAKE_CURRENT_LIST_DIR}/can2040/pio/can2040.pio)

function(add_board BOARD)
  add_executable(gs_usb_${BOARD}
    main.c
    usb_descriptors.c
  )
  target_include_directories(gs_usb_${BOARD} PRIVATE .)
  target_link_libraries(gs_usb_${BOARD} pico_stdlib tinyusb_device tinyusb_board can2040)
  target_compile_definitions(gs_usb_${BOARD} PRIVATE -DBOARD_${BOARD})
  pico_add_extra_outputs(gs_usb_${BOARD})
endfunction()

add_board(PICO_CAN2040)
