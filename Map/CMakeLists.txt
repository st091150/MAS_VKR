cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(QGeoView LANGUAGES C CXX)

# Если не найден mingw_64 cmake
# set(CMAKE_PREFIX_PATH "C:/Qt/6.8.2/mingw_64/lib/cmake")

add_subdirectory(lib)
add_subdirectory(shared)
add_subdirectory(Map)
