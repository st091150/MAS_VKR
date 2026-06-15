set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake >= 3.30: FindBoost removed. We only need headers (Boost.Geometry),
# so use CONFIG mode to avoid the removed module warning.
cmake_policy(SET CMP0167 NEW)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# MinGW/Windows: very long include lists can break moc_predefs generation
# (command line length limit). Disabling it keeps AUTOGEN stable.
set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)
set(BOOST_ROOT "C:/local/boost_1_87_0")

# Set the QT version
find_package(Qt6 COMPONENTS Core QUIET)
if (NOT Qt6_FOUND)
    set(QT_VERSION 5 CACHE STRING "Qt version for QGeoView")
else()
    set(QT_VERSION 6 CACHE STRING "Qt version for QGeoView")
endif()

find_package(Qt${QT_VERSION} REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    Network
)
# Boost: используем заголовки (Boost.Geometry). Для новых CMake FindBoost удалён,
# поэтому просто добавляем include-дир (header-only).
set(BOOST_INCLUDEDIR "${BOOST_ROOT}")

add_executable(Map
    main.cpp
    GeoViewMapLayers.h
    GeoViewWidget.h
    GeoViewWidget.cpp
    resources.qrc
)

add_subdirectory(state)
add_subdirectory(ui)
add_subdirectory(features)
add_subdirectory(utils)
add_subdirectory(geometry)
add_subdirectory(services)
add_subdirectory(dialogs)
add_subdirectory(clipper)

target_link_libraries(Map
    PRIVATE
    Qt${QT_VERSION}::Core
    Qt${QT_VERSION}::Network
    Qt${QT_VERSION}::Gui
    Qt${QT_VERSION}::Widgets
    QGeoView
    qgeoview-samples-shared
)

target_include_directories(Map PRIVATE ${BOOST_INCLUDEDIR})
