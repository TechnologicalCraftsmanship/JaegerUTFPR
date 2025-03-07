# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )
set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(SRCS "main.c" "motor.c" "udp_server.c" "tcp_server.c" "mic.c" "DataArray.c" "speaker.c" "speakerRMT.c" "BlockArray.c"
						INCLUDE_DIRS ".")