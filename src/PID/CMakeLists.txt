add_library(
    PID
    include/PID.hpp
    src/PID.cpp
)

target_include_directories(PID PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
