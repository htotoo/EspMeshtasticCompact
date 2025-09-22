
file(GLOB_RECURSE MESHTASTIC_SRCS "src/unishox2.cpp" "meshtastic/*.cpp")
idf_component_register(SRCS "src/main.php" "src/MeshtasticCompact.cpp" "src/MeshtasticCompactFileIO.cpp" "src/unishox2.cpp" ${MESHTASTIC_SRCS} "src/pb_common.c" "src/pb_decode.c" "src/pb_encode.c"
                    INCLUDE_DIRS "." "include" "meshtastic" REQUIRES driver esp_timer mbedtls radiolib nvs_flash )
