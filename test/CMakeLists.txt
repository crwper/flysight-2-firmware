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
	${FS_DIR}/flight_params.c
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
	# Source-file properties are directory-scoped and therefore shared by every
	# target that compiles the file (config.c/nav.c/common.c are also linked
	# into audio_sim_ref). Gate --coverage on a per-target property so only
	# audio_sim is instrumented and gcov flags never leak into the reference.
	set_source_files_properties(${FIRMWARE_SOURCES} PROPERTIES
		COMPILE_OPTIONS "$<$<BOOL:$<TARGET_PROPERTY:FS_COVERAGE>>:--coverage>")
	set_target_properties(audio_sim PROPERTIES FS_COVERAGE ON)
	target_link_options(audio_sim PRIVATE --coverage)
endif()

target_link_libraries(audio_sim PRIVATE m)

# Frozen-oracle build for differential testing (card A0). Identical to
# audio_sim but compiling the frozen reference/audio_control_orig.c in place
# of the live FlySight/audio_control.c. No coverage instrumentation: the
# reference must reflect pure firmware behaviour, and gcov flags must never
# leak into it.
add_executable(audio_sim_ref
	runner/main.c
	runner/track.c
	sim/sim.c
	fakes/fake_audio.c
	fakes/fake_seq_ts.c
	fakes/fake_gnss.c
	fakes/fake_ff.c
	fakes/fake_hal.c
	reference/audio_control_orig.c
	${FS_DIR}/flight_params.c
	${FS_DIR}/config.c
	${FS_DIR}/nav.c
	${FS_DIR}/common.c
)

target_include_directories(audio_sim_ref PRIVATE
	fakes   # must precede FS_DIR: shadows platform headers
	sim
	runner
	${FS_DIR}
)

target_compile_options(audio_sim_ref PRIVATE -Wall -O0 -g)

target_link_libraries(audio_sim_ref PRIVATE m)
