# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/devcontainers/esp/v5.2/esp-idf/components/bootloader/subproject"
  "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader"
  "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix"
  "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix/tmp"
  "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix/src/bootloader-stamp"
  "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix/src"
  "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/devcontainers/IDF/Examples/rmt_led_strip/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
