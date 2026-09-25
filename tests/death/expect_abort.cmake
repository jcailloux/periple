# Runs PROGRAM, echoes its output, and fails if it exits normally.
execute_process(COMMAND ${PROGRAM} RESULT_VARIABLE result OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${out}${err}")
if(result EQUAL 0)
    message(FATAL_ERROR "expected ${PROGRAM} to abort")
endif()
