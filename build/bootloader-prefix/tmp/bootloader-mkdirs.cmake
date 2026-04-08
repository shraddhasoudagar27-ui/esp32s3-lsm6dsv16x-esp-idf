# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/esp_idf/.espressif/v6.0/esp-idf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "C:/esp_idf/.espressif/v6.0/esp-idf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader"
  "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix"
  "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix/tmp"
  "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix/src/bootloader-stamp"
  "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix/src"
  "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/LSM6D_intern/imu_demo_lsm6d/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
