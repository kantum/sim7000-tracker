cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(mqtt-simple)

# NORDIC SDK APP START
#target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

set(IOTC_PATH $ENV{HOME}/depots/iot-device-sdk-embedded-c)

target_include_directories(app PRIVATE ${IOTC_PATH}/examples/common/src)
target_include_directories(app PRIVATE ${IOTC_PATH}/include)

target_sources(app PRIVATE
        src/main.c
		${IOTC_PATH}/example/common/src/commandline.c
		${IOTC_PATH}/example/common/src/example_utils.c)

# libiotc
target_link_libraries(app PUBLIC ${IOTC_PATH}/bin/zephyr/libiotc.a)

# mbedTLS
set(IOTC_MBEDTLS_LIB_PATH ${IOTC_PATH}/third_party/tls/mbedtls/library)
target_link_libraries(app PUBLIC ${IOTC_MBEDTLS_LIB_PATH}/libmbedtls.a)
target_link_libraries(app PUBLIC ${IOTC_MBEDTLS_LIB_PATH}/libmbedx509.a)
target_link_libraries(app PUBLIC ${IOTC_MBEDTLS_LIB_PATH}/libmbedcrypto.a)

# wolfssl, disabled since wolfssl fails to build with -m32 flag (--enable-32bit)
#  target_link_libraries(app PUBLIC ${IOTC_PATH}/third_party/tls/wolfssl/src/.libs/libwolfssl.a)

