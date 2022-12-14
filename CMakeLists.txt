cmake_minimum_required( VERSION 3.7 )
project(async_detour_crowd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest")
set(PKG_CONFIG_EXECUTABLE "$ENV{usr}/bin/pkg-config.exe")
add_library(DetourCrowd STATIC IMPORTED)
add_library(Detour STATIC IMPORTED)
set_target_properties(DetourCrowd PROPERTIES
    IMPORTED_CONFIGURATIONS "Debug;Release"
    IMPORTED_LOCATION "$ENV{usr}/lib/DetourCrowd.lib"
    IMPORTED_LOCATION_DEBUG "$ENV{usr}/lib/DetourCrowd-d.lib")
set_target_properties(Detour PROPERTIES
    IMPORTED_CONFIGURATIONS "Debug;Release"
    IMPORTED_LOCATION "$ENV{usr}/lib/Detour.lib"
    IMPORTED_LOCATION_DEBUG "$ENV{usr}/lib/Detour-d.lib")


endif(MSVC)
include(FindPkgConfig)
if(MSVC)
set(ENV{PKG_CONFIG_PATH} "$ENV{usr}/lib/pkgconfig")
endif(MSVC)
pkg_search_module(RecastNavigation REQUIRED recastnavigation)


INCLUDE_DIRECTORIES(${RecastNavigation_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

FILE(GLOB_RECURSE SRC_LISTS ${CMAKE_SOURCE_DIR}/src/*.cpp)
FILE(GLOB_RECURSE HEADER_LISTS ${CMAKE_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} ${SRC_LISTS} ${HEADER_LISTS})
target_link_libraries(${PROJECT_NAME} DetourCrowd Detour)
add_subdirectory(./test)

foreach(p LIB BIN INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		string(TOLOWER ${p} lower_p)
        set(${var} "${CMAKE_INSTALL_PREFIX}/${lower_p}")
	endif()
endforeach()

export(TARGETS ${CMAKE_PROJECT_NAME}
     FILE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)

file(WRITE
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
    "include(\${CMAKE_CURRENT_LIST_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake)\n")

install(FILES
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
   ARCHIVE  DESTINATION lib
   LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include/${PROJECT_NAME})

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME})
