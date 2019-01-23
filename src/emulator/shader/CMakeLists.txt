add_library(
	shader
	STATIC
	include/shader/functions.h
	include/shader/profile.h
	include/shader/types.h
	include/shader/matcher.h
	include/shader/decoder_detail.h
	src/spirv_recompiler.cpp
	src/parse_gxp_usse.cpp
)

target_include_directories(shader PUBLIC include)
target_link_libraries(shader PUBLIC util spirv-cross-glsl vita-headers)
target_link_libraries(shader PRIVATE gxm SPIRV microprofile)
