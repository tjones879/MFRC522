# MFRC522

This is a modern C++ library for the contactless reader/writer MFRC522 chips.
This library is written to use libopencm3 and FreeRTOS and is agnostic with
respect to the host microcontroller being ran on. However, it is fairly easy
to port to different a different HAL or RTOS if desired. Instructions for
doing so can be found below.


### Usage
---------

There are two options for including this library with your code:

1. You can simply drop MFRC522.cpp and MFRC522.hpp directly with the rest of
   your source files and add them to your build system. Please see the examples
   folder to see how to do this.

2. You may also use this as a git submodule and integrate it as a CMake project.
   This method allows you to upgrade this library simply by calling git pull
   within the submodule folder.


### Documentation
-----------------


### Porting
-----------


### Contributions
-----------------
