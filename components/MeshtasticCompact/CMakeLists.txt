cmake_minimum_required(VERSION 3.16)
file(GLOB_RECURSE MESHTASTIC_SRCS "meshtastic/*.cpp")
idf_component_register(SRCS "MeshtasticCompact.cpp"  ${MESHTASTIC_SRCS} "pb_common.c" "pb_decode.c" "pb_encode.c"
                    INCLUDE_DIRS "." "include" REQUIRES driver esp_timer mbedtls )

                    