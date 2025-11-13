function(list_sources_recursive output_var search_dir)
	set(options "")
	set(oneValueArgs "")
	set(multiValueArgs EXCLUDE_DIRS)

	cmake_parse_arguments(PARSE_ARGV 2
			"ARG"
			"${options}"
			"${oneValueArgs}"
			"${multiValueArgs}"
	)

	file( GLOB_RECURSE files "${search_dir}/*.cpp" "${search_dir}/*.h")

	foreach (exclude_dir ${ARG_EXCLUDE_DIRS})
			list(FILTER files EXCLUDE REGEX ".*/${search_dir}/${exclude_dir}/.*")
	endforeach ()

	set(${output_var} ${files}  PARENT_SCOPE)
endfunction()

function(append_sources_to_file file_path)
	list_sources_recursive(source_list ${ARGN})
	string(REPLACE ";" "\n" source_list "${source_list}")
	file(APPEND "${SOURCES_LIST_FILE}" "${source_list}\n")
endfunction()

# The file path where we dump the list of source files to be formatted
set(SOURCES_LIST_FILE "${CMAKE_BINARY_DIR}/sources_to_format.txt")
file(REMOVE "${SOURCES_LIST_FILE}")

append_sources_to_file(source_list qCC EXCLUDE_DIRS extern)
append_sources_to_file(source_list ccViewer EXCLUDE_DIRS extern)
append_sources_to_file(source_list libs EXCLUDE_DIRS CCAppCommon/QDarkStyleSheet qCC_db/extern qCC_io/extern)
append_sources_to_file(source_list plugins/core/GL EXCLUDE_DIRS qSSAO/extern)
append_sources_to_file(source_list plugins/core/IO EXCLUDE_DIRS qE57IO/extern qPhotoscanIO/extern)

find_program(CLANG_FORMAT NAMES clang-format)

add_custom_target(
	format COMMAND ${CMAKE_COMMAND} -E echo "Format with: $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format>"
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> --version
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> -i "--files=${SOURCES_LIST_FILE}"
)

add_custom_target(
	check-format
	COMMAND ${CMAKE_COMMAND} -E echo "Checking format with: $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format>"
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> --version
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> --Werror --dry-run "--files=${SOURCES_LIST_FILE}"
)
