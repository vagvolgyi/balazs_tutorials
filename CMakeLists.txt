if(MSVC)
    cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
else()
    cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Macro to determine if project is compiled inside a ROS/catkin workspace
function (is_catkin_build RESULT)
  set (${RESULT} FALSE PARENT_SCOPE)
  if (DEFINED ENV{ROS_ROOT} OR DEFINED ENV{ROS_VERSION})
    if (DEFINED CATKIN_DEVEL_PREFIX)
      set (${RESULT} TRUE PARENT_SCOPE)
    endif ()
  endif ()
endfunction (is_catkin_build)


project(balazs_tutorials)

# ROS/catkin paths
is_catkin_build (PROJECT_IS_CATKIN_BUILT)
if (PROJECT_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}")
    set (LIBRARY_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib")
endif ()

if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MT>
        $<$<CONFIG:Debug>:/MTd>
        $<$<CONFIG:Release>:/MT>
    )
endif()

find_package(OpenCV REQUIRED)

find_package(Qt5 COMPONENTS Widgets)
if(NOT Qt5_FOUND)
    message(FATAL_ERROR "Qt5 was not found (ROS Melodic or newer required with rqt installed)")
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(COMMON_HEADER_FILES
    common/qt/image_frame/image_frame.h
    common/qt/image_frame/if_overlay.h
    )
set(COMMON_SOURCE_FILES
    common/qt/image_frame/image_frame.cpp
    common/qt/image_frame/if_overlay.cpp
    )

include_directories(${OpenCV_INCLUDE_DIRS} common .)

install(DIRECTORY icons
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
        )

###############
# qt_tutorial #
###############

add_executable(qt_tutorial
               ${COMMON_HEADER_FILES}
               ${COMMON_SOURCE_FILES}
               qt_tutorial/my_window.h
               qt_tutorial/overlays_dialog.h
               qt_tutorial/main.cpp
               qt_tutorial/my_window.cpp
               qt_tutorial/my_window.ui
               qt_tutorial/overlays_dialog.cpp
               qt_tutorial/overlays_dialog.ui
               )
set(qt_tutorial_LIBRARIES Qt5::Widgets ${OpenCV_LIBRARIES})
if(NOT MSVC)
    set(qt_tutorial_LIBRARIES ${qt_tutorial_LIBRARIES} pthread)
endif()
target_link_libraries(qt_tutorial ${qt_tutorial_LIBRARIES})

if (PROJECT_IS_CATKIN_BUILT)
    set(CATKIN_COMPONENTS_REQUIRED
        rqt_gui
        rqt_gui_cpp
        roslib
        )

    find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS_REQUIRED})

    include_directories(${catkin_INCLUDE_DIRS})

    ################
    # rqt_tutorial #
    ################

    set(rqt_tutorial_SRCS
        ${COMMON_HEADER_FILES}
        ${COMMON_SOURCE_FILES}
        rqt_tutorial/src/rqt_tutorial.cpp
        )
    set(rqt_tutorial_UIS
        rqt_tutorial/src/rqt_tutorial.ui
        )
    set(rqt_tutorial_HDRS
        rqt_tutorial/include/rqt_tutorial.h
        )

    if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
        file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
    endif()

    set(rqt_tutorial_INCLUDE_DIRECTORIES
        rqt_tutorial/include
        "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
        )
    include_directories(${rqt_tutorial_INCLUDE_DIRECTORIES})

    catkin_package(INCLUDE_DIRS ${rqt_tutorial_INCLUDE_DIRECTORIES}
                   LIBRARIES rqt_tutorial
                   CATKIN_DEPENDS ${CATKIN_COMPONENTS_REQUIRED}
                   )
    catkin_python_setup()

    qt5_wrap_cpp(rqt_tutorial_MOCS ${rqt_tutorial_HDRS})

    set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
    set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
    qt5_wrap_ui(rqt_tutorial_UIS_H ${rqt_tutorial_UIS})
    set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

    add_library(rqt_tutorial ${rqt_tutorial_SRCS} ${rqt_tutorial_MOCS} ${rqt_tutorial_UIS_H})
    target_link_libraries(rqt_tutorial ${catkin_LIBRARIES} Qt5::Widgets)

    find_package(class_loader)
    class_loader_hide_library_symbols(rqt_tutorial)

    catkin_install_python(PROGRAMS scripts/rqt_tutorial
                          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif ()
