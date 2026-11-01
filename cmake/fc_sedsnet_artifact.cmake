# FC consumes only the C static archive. Asking Cargo for an rlib as well
# suppresses its declared release-embedded LTO and exhausts the MAIN slot.
# Keep the SDK's generic/Rust-consumer defaults unchanged; opt in here.
function(fc_select_sedsnet_staticlib)
    set(SEDSNET_CRATE_TYPE "staticlib" CACHE STRING
        "FC C-only embedded SDK artifact; requires SEDSnet 4.3.6 selector" FORCE)
endfunction()

function(fc_verify_sedsnet_staticlib sdk_directory)
    # This is the actual list passed verbatim to the SDK custom command, not
    # a cache label, help string or guessed SDK version. Public 4.3.5 has no
    # selector, so reject it before attempting an oversized firmware build.
    get_directory_property(_fc_sdk_args DIRECTORY "${sdk_directory}"
                           DEFINITION RS_ARGS_BOOTSTRAP)
    list(FIND _fc_sdk_args "--crate-type" _fc_type_index)
    list(LENGTH _fc_sdk_args _fc_arg_count)
    math(EXPR _fc_value_index "${_fc_type_index} + 1")
    set(_fc_artifact_supported FALSE)
    if(_fc_type_index GREATER_EQUAL 0 AND _fc_value_index LESS _fc_arg_count)
        list(GET _fc_sdk_args ${_fc_value_index} _fc_crate_type)
        if(_fc_crate_type STREQUAL "staticlib")
            set(_fc_artifact_supported TRUE)
        endif()
    endif()
    if(NOT _fc_artifact_supported)
        message(FATAL_ERROR
            "FlightComputer26 requires SEDSnet 4.3.6 or a reviewed compatible source "
            "whose CMake build forwards --crate-type staticlib. Public 4.3.5 is "
            "unsupported for this C-only recipe. Refresh the selected SDK source "
            "or set FETCHCONTENT_SOURCE_DIR_SEDSNET to a compatible checkout. "
            "Do not patch a shared/cached SDK or force LTO on dual rlib/staticlib outputs. "
            "Selected source: ${sdk_directory}")
    endif()
    message(STATUS "FC SEDSnet artifact: staticlib (SDK command verified)")
endfunction()
