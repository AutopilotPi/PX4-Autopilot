############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

foreach(REQUIRED_VARIABLE CMAKE_RUNTIME_OUTPUT_DIRECTORY PX4_SOURCE_DIR PX4_BINARY_DIR)
	if(NOT DEFINED ${REQUIRED_VARIABLE} OR "${${REQUIRED_VARIABLE}}" STREQUAL "")
		message(FATAL_ERROR "${REQUIRED_VARIABLE} is required")
	endif()
endforeach()

find_program(GIT_EXECUTABLE git)

if(NOT GIT_EXECUTABLE)
	message(FATAL_ERROR "git is required to create a PX4 release")
endif()

execute_process(
	COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_COMMIT
	OUTPUT_STRIP_TRAILING_WHITESPACE
	RESULT_VARIABLE GIT_RESULT
)

if(NOT GIT_RESULT EQUAL 0 OR GIT_COMMIT STREQUAL "")
	message(FATAL_ERROR "Failed to determine the PX4 git commit")
endif()

set(RELEASE_DIR "${PX4_BINARY_DIR}/release")
set(RELEASE_PX4_DIR "${RELEASE_DIR}/px4")
set(RELEASE_ARCHIVE "${PX4_BINARY_DIR}/px4-${GIT_COMMIT}.zip")

file(REMOVE_RECURSE "${RELEASE_DIR}")
file(MAKE_DIRECTORY "${RELEASE_PX4_DIR}")

foreach(SOURCE_DIR
		"${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
		"${PX4_SOURCE_DIR}/posix-configs"
		"${PX4_BINARY_DIR}/etc")
	if(NOT IS_DIRECTORY "${SOURCE_DIR}")
		message(FATAL_ERROR "Release source directory does not exist: ${SOURCE_DIR}")
	endif()

	file(COPY "${SOURCE_DIR}" DESTINATION "${RELEASE_PX4_DIR}" USE_SOURCE_PERMISSIONS)
endforeach()

execute_process(
	COMMAND ${CMAKE_COMMAND} -E tar cf "${RELEASE_ARCHIVE}" --format=zip px4
	WORKING_DIRECTORY "${RELEASE_DIR}"
	RESULT_VARIABLE ARCHIVE_RESULT
)

if(NOT ARCHIVE_RESULT EQUAL 0)
	message(FATAL_ERROR "Failed to create ${RELEASE_ARCHIVE}")
endif()

file(REMOVE_RECURSE "${RELEASE_DIR}")

message(STATUS "PX4 release created: ${RELEASE_ARCHIVE}")
