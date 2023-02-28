link_directories("${FUZZYLITE_DIR}/release/bin")

add_library(
	FuzzyPID
	include/FuzzyPID.hpp
	src/FuzzyPID.cpp
)

target_include_directories(FuzzyPID PUBLIC ${FUZZYLITE_DIR})

target_link_libraries(FuzzyPID PUBLIC fuzzylite-static)


target_include_directories(FuzzyPID PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
