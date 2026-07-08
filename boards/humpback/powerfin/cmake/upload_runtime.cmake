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

if(DEFINED ENV{UPLOAD_DEST} AND NOT "$ENV{UPLOAD_DEST}" STREQUAL "")
	set(UPLOAD_DEST "$ENV{UPLOAD_DEST}")
else()
	if(NOT DEFINED ENV{UPLOAD_HOST} OR "$ENV{UPLOAD_HOST}" STREQUAL "")
		message(FATAL_ERROR "Upload destination not set. Use UPLOAD_DEST=user@host:/path or UPLOAD_HOST=host [UPLOAD_USER=user] [UPLOAD_DIR=/path].")
	endif()

	if(DEFINED ENV{UPLOAD_USER} AND NOT "$ENV{UPLOAD_USER}" STREQUAL "")
		set(UPLOAD_USER "$ENV{UPLOAD_USER}")
	else()
		set(UPLOAD_USER "root")
	endif()

	if(DEFINED ENV{UPLOAD_DIR} AND NOT "$ENV{UPLOAD_DIR}" STREQUAL "")
		set(UPLOAD_DIR "$ENV{UPLOAD_DIR}")
	else()
		set(UPLOAD_DIR "/root/px4")
	endif()

	set(UPLOAD_DEST "${UPLOAD_USER}@$ENV{UPLOAD_HOST}:${UPLOAD_DIR}")
endif()

if(NOT UPLOAD_DEST MATCHES "^([^:]+):(.+)$")
	message(FATAL_ERROR "Invalid UPLOAD_DEST '${UPLOAD_DEST}'. Expected user@host:/absolute/path.")
endif()

string(REGEX REPLACE "^([^:]+):(.+)$" "\\1" UPLOAD_REMOTE "${UPLOAD_DEST}")
string(REGEX REPLACE "^([^:]+):(.+)$" "\\2" UPLOAD_REMOTE_DIR "${UPLOAD_DEST}")

if(DEFINED ENV{UPLOAD_SSH_OPTS} AND NOT "$ENV{UPLOAD_SSH_OPTS}" STREQUAL "")
	separate_arguments(UPLOAD_SSH_OPTIONS UNIX_COMMAND "$ENV{UPLOAD_SSH_OPTS}")
else()
	set(UPLOAD_SSH_OPTIONS
		-o StrictHostKeyChecking=no
	)
endif()

if(DEFINED ENV{UPLOAD_PASSWORD} AND NOT "$ENV{UPLOAD_PASSWORD}" STREQUAL "")
	set(UPLOAD_SSH_PREFIX sshpass -e)
	string(REPLACE ";" " " UPLOAD_SSH_COMMAND "sshpass;-e;ssh;${UPLOAD_SSH_OPTIONS}")
else()
	set(UPLOAD_SSH_PREFIX)
	string(REPLACE ";" " " UPLOAD_SSH_COMMAND "ssh;${UPLOAD_SSH_OPTIONS}")
endif()

set(UPLOAD_RSYNC_OPTIONS
	--archive
	--human-readable
	--info=progress2
)

if(DEFINED ENV{UPLOAD_DELETE} AND "$ENV{UPLOAD_DELETE}" STREQUAL "1")
	list(APPEND UPLOAD_RSYNC_OPTIONS --delete)
endif()

message(STATUS "Uploading PX4 to ${UPLOAD_DEST}")

execute_process(
	COMMAND ${CMAKE_COMMAND} -E env "SSHPASS=$ENV{UPLOAD_PASSWORD}"
		${UPLOAD_SSH_PREFIX} ssh ${UPLOAD_SSH_OPTIONS} ${UPLOAD_REMOTE} "mkdir -p '${UPLOAD_REMOTE_DIR}'"
	RESULT_VARIABLE MKDIR_RESULT
)

if(NOT MKDIR_RESULT EQUAL 0)
	message(FATAL_ERROR "Failed to create remote directory '${UPLOAD_REMOTE_DIR}' on ${UPLOAD_REMOTE}")
endif()

execute_process(
	COMMAND ${CMAKE_COMMAND} -E env "SSHPASS=$ENV{UPLOAD_PASSWORD}"
		rsync ${UPLOAD_RSYNC_OPTIONS}
		-e "${UPLOAD_SSH_COMMAND}"
		${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
		${PX4_SOURCE_DIR}/posix-configs
		${PX4_BINARY_DIR}/etc
		${UPLOAD_DEST}
	RESULT_VARIABLE RSYNC_RESULT
)

if(NOT RSYNC_RESULT EQUAL 0)
	get_filename_component(PX4_BIN_PARENT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" DIRECTORY)
	set(TAR_UPLOAD_COMMAND
		"tar -C '${PX4_BIN_PARENT_DIR}' -cf - bin -C '${PX4_SOURCE_DIR}' posix-configs -C '${PX4_BINARY_DIR}' etc | ${UPLOAD_SSH_COMMAND} ${UPLOAD_REMOTE} \"tar -C '${UPLOAD_REMOTE_DIR}' -xf -\""
	)

	message(STATUS "rsync upload failed, falling back to tar over ssh")

	execute_process(
		COMMAND ${CMAKE_COMMAND} -E env "SSHPASS=$ENV{UPLOAD_PASSWORD}"
			sh -c "${TAR_UPLOAD_COMMAND}"
		RESULT_VARIABLE TAR_RESULT
	)

	if(NOT TAR_RESULT EQUAL 0)
		message(FATAL_ERROR "tar upload failed")
	endif()
endif()
