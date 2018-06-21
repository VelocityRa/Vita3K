add_library(
cpu
STATIC
include/cpu/functions.h
include/cpu/cpu_dynarmic.h
src/cpu.cpp
src/cpu_dynarmic.cpp
)

target_include_directories(cpu PUBLIC include)
target_link_libraries(cpu PUBLIC dynarmic disasm mem)
target_link_libraries(cpu PRIVATE unicorn)
