cmake_minimum_required(VERSION 3.20)
project(fifi)

set(CMAKE_CXX_STANDARD 20)

# glfw
add_subdirectory(third_party/glfw)

# glad
add_library(glad STATIC third_party/glad/src/glad.c)
target_include_directories(glad PUBLIC third_party/glad/include)

# glm
add_subdirectory(third_party/glm)

# collect sources
file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(fifi ${SOURCES})

target_include_directories(fifi PRIVATE
    src/
    third_party/glad/include
    third_party/glfw/include
    third_party/glm
)

target_link_libraries(fifi PRIVATE glfw glad glm)

add_custom_command(TARGET fifi POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/shaders
    $<TARGET_FILE_DIR:fifi>/shaders
)
