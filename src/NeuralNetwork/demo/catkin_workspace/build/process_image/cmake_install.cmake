# Install script for directory: /home/triton_01/Documents/projet_iD/S1_G1_Saunier_Sough_Abouchi/src/NeuralNetwork/demo/catkin_workspace/src/process_image

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/triton_01/Documents/projet_iD/S1_G1_Saunier_Sough_Abouchi/src/NeuralNetwork/demo/catkin_workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/triton_01/Documents/projet_iD/S1_G1_Saunier_Sough_Abouchi/src/NeuralNetwork/demo/catkin_workspace/build/process_image/catkin_generated/installspace/process_image.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/process_image/cmake" TYPE FILE FILES
    "/home/triton_01/Documents/projet_iD/S1_G1_Saunier_Sough_Abouchi/src/NeuralNetwork/demo/catkin_workspace/build/process_image/catkin_generated/installspace/process_imageConfig.cmake"
    "/home/triton_01/Documents/projet_iD/S1_G1_Saunier_Sough_Abouchi/src/NeuralNetwork/demo/catkin_workspace/build/process_image/catkin_generated/installspace/process_imageConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/process_image" TYPE FILE FILES "/home/triton_01/Documents/projet_iD/S1_G1_Saunier_Sough_Abouchi/src/NeuralNetwork/demo/catkin_workspace/src/process_image/package.xml")
endif()

