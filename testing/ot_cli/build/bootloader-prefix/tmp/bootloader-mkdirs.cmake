# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/espressif2/Espressif/frameworks/esp-idf-v5.1.2/components/bootloader/subproject"
  "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader"
  "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix"
  "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix/tmp"
  "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix/src"
  "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Niki/Desktop/sensorlelel/ot_cli/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()