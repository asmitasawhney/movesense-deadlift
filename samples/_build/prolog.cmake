# Settings that are present in every Movesense build go here.

cmake_policy(SET CMP0005 NEW) # Preprocessor definition values are now escaped automatically.
cmake_policy(SET CMP0011 NEW) # Included scripts do automatic cmake_policy PUSH and POP.

# TODO: This is from when you used to have several applications you could make
# from one build. The define is used in some places in code. Might just get rid
# of this.
if(NOT DEFINED APPLICATION)
    set(APPLICATION "MovesenseApplication")
endif()
add_definitions(-DCMAKE_APPLICATION="${APPLICATION}")

# Horrific hack until we get cstdint properly in
# (UINT32_MAX etc won't work without this)
add_definitions(-D__STDC_LIMIT_MACROS)

set(TOOLS_PATH ${MOVESENSE_CORE_LIBRARY}/tools)

if (APPLE OR UNIX OR FORCE_LINUX)
    message(STATUS "PATH_TOOL_PYTHON will use python from shell PATH")
    set(PATH_TOOL_PYTHON python)
else()
    message(STATUS "PATH_TOOL_PYTHON set to portable-python (win32 only)")
    set(PATH_TOOL_PYTHON ${TOOLS_PATH}/portable-python/App/python.exe)
endif()

# helper functions
include(${CMAKE_SOURCE_DIR}/../_build/functions.cmake)
