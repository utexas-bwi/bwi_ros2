# Find Qt5 or Qt6 instead of deprecated Qt4
find_package(Qt5 QUIET COMPONENTS Widgets)

if(NOT Qt5_FOUND)
  message("-- ")
  message("-- SegwayRMP GUI will not be built: Qt5 not found")
  message("-- ")
endif()

# Find SDL2 instead of older SDL
find_package(SDL2 QUIET)

if(NOT SDL2_FOUND)
  message("-- ")
  message("-- SegwayRMP GUI will not be built: SDL2 not found")
  message("-- ")
endif()

if(Qt5_FOUND AND SDL2_FOUND)
  message("-- Building SegwayRMP GUI")

  # Include Qt5 components
  set(CMAKE_AUTOMOC ON)  # Enable automoc for Qt5
  set(CMAKE_AUTOUIC ON)  # Enable autouic for .ui forms
  set(CMAKE_AUTORCC ON)  # Enable autorcc for resource files (if needed)

  # Include directories
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  include_directories(${SDL2_INCLUDE_DIRS})
  include_directories(${Qt5Widgets_INCLUDE_DIRS})

  # Define source and header files
  set(libsegwayrmp_gui_SRCS
    src/gui/main.cc
    src/gui/segwayrmp_gui.cc
  )
  set(libsegwayrmp_gui_HDRS
    include/libsegwayrmp/gui/segwayrmp_gui.h
  )
  set(libsegwayrmp_gui_FORMS include/libsegwayrmp/gui/segwayrmp_gui.ui)

  # Automatically handle Qt's meta-object compiler (MOC) and user interface compiler (UIC)
  qt5_wrap_cpp(libsegwayrmp_gui_HDRS_MOC ${libsegwayrmp_gui_HDRS})
  qt5_wrap_ui(libsegwayrmp_gui_FORMS_HDRS ${libsegwayrmp_gui_FORMS})

  # Build the executable
  add_executable(libsegwayrmp_gui
    ${libsegwayrmp_gui_SRCS}
    ${libsegwayrmp_gui_HDRS_MOC}
    ${libsegwayrmp_gui_FORMS_HDRS}
  )

  # Link against necessary libraries (Qt5, SDL2, libsegwayrmp)
  target_link_libraries(libsegwayrmp_gui Qt5::Widgets ${SDL2_LIBRARIES} libsegwayrmp)
endif()

