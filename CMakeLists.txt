cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE MESHTASTIC_SRCS "src/unishox2.cpp" "meshtastic/*.cpp")
idf_component_register(SRCS "src/MeshtasticCompact.cpp" "src/unishox2.cpp" ${MESHTASTIC_SRCS} "src/pb_common.c" "src/pb_decode.c" "src/pb_encode.c"
                    INCLUDE_DIRS "." "include" "meshtastic" REQUIRES driver esp_timer mbedtls )
