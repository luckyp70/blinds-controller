# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Software versioning
set(OTA_VERSION_MAJOR 1)
set(OTA_VERSION_MINOR 0)
set(OTA_VERSION_PATCH 0)
set(OTA_VERSION_BUILD 0)

# Hardware versioning
set(OTA_HW_VERSION_MAJOR 1)
set(OTA_HW_VERSION_MINOR 0)


# Compose the 32-bit software version [major.minor.patch.build]
math(EXPR OTA_VERSION_HEX "(${OTA_VERSION_MAJOR} << 24) | (${OTA_VERSION_MINOR} << 16) | (${OTA_VERSION_PATCH} << 8) | ${OTA_VERSION_BUILD}")

# Compose the 16-bit hardware version: [major.minor]
math(EXPR OTA_HW_VERSION_HEX "(${OTA_HW_VERSION_MAJOR} << 8) | ${OTA_HW_VERSION_MINOR}")

add_compile_definitions(
    OTA_UPGRADE_RUNNING_FILE_VERSION=${OTA_VERSION_HEX}
    OTA_UPGRADE_HW_VERSION=${OTA_HW_VERSION_HEX}
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(blinds-controller)
