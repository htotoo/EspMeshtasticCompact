

file(GLOB_RECURSE MESHTASTIC_SRCS "meshtastic/*.cpp")

idf_component_register(SRCS "MeshtasticCompact.cpp"  ${MESHTASTIC_SRCS} "pb_common.c" "pb_decode.c" "pb_encode.c"
                    INCLUDE_DIRS "." "include")
