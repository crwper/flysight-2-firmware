# Host-side simulator/characterization harness for FlySight audio control.
#
# Configure (from test/):
#   cmake -B build -G Ninja -DCMAKE_C_COMPILER=C:/msys64/mingw64/bin/gcc.exe
#   cmake --build build
#
# The firmware sources below are compiled UNMODIFIED; the fakes/ directory
# shadows the platform headers (main.h, app_common.h, stm32_seq.h, ff.h)
# because it comes first on the include path.

cmake_minimum_required(VERSION 3.16)
project(fs_audio_sim C)

set(CMAKE_C_STANDARD 11)

set(FS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../FlySight)

option(COVERAGE "Instrument firmware sources with gcov" ON)

set(FIRMWARE_SOURCES
	${FS_DIR}/audio_control.c
	${FS_DIR}/config.c
	${FS_DIR}/nav.c
	${FS_DIR}/common.c
)

add_executable(audio_sim
	runner/main.c
	runner/track.c
	sim/sim.c
	fakes/fake_audio.c
	fakes/fake_seq_ts.c
	fakes/fake_gnss.c
	fakes/fake_ff.c
	fakes/fake_hal.c
	${FIRMWARE_SOURCES}
)

target_include_directories(audio_sim PRIVATE
	fakes   # must precede FS_DIR: shadows platform headers
	sim
	runner
	${FS_DIR}
)

target_compile_options(audio_sim PRIVATE -Wall -O0 -g)

if(COVERAGE)
	set_source_files_properties(${FIRMWARE_SOURCES} PROPERTIES COMPILE_OPTIONS "--coverage")
	target_link_options(audio_sim PRIVATE --coverage)
endif()

target_link_libraries(audio_sim PRIVATE m)
