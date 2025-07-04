cmake_minimum_required(VERSION 3.16)
file(GLOB_RECURSE MESHTASTIC_SRCS "unishox2.cpp" "meshtastic/*.cpp")
idf_component_register(SRCS "MeshtasticCompact.cpp" unishox2.cpp ${MESHTASTIC_SRCS} "pb_common.c" "pb_decode.c" "pb_encode.c"
                    INCLUDE_DIRS "." "include" REQUIRES driver esp_timer mbedtls )
