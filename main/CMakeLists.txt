
file(GLOB_RECURSE MESHTASTIC_SRCS "../src/unishox2.cpp" "meshtastic/*.cpp")
file(GLOB_RECURSE ED25519_SRCS "../lib/ed25519/*.c0")
idf_component_register(SRCS "main.cpp" "../src/MeshtasticCompact.cpp"  "../src/MeshcoreCompact.cpp" "../src/MeshtasticCompactFileIO.cpp" "../src/unishox2.cpp" ${ED25519_SRCS} ${MESHTASTIC_SRCS} "../src/pb_common.c" "../src/pb_decode.c" "../src/pb_encode.c"
                    INCLUDE_DIRS "." "../" "../lib/ed25519" "../include" "../meshtastic" REQUIRES driver esp_timer mbedtls radiolib nvs_flash )
