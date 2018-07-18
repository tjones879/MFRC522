# MFRC522

The MFRC522 is a 13.56 MHz contactless reader/writer IC. It supports
working with ISO/IEC 14443 A/MIFARE and NTAG. Officially, this chip
supports SPI/UART/I2C connectivity to a host microcontroller. However,
most cheap MFRC522 modules found online for hobbiests are limited to
a single SPI connection.

The official MFRC522 [datasheet](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf)
can be found on NXP's website.

This is a modern C++ library for the contactless reader/writer MFRC522 chips.
This library is agnostic with respect to both the host microcontroller running
the code and the library used to drive your SPI peripherals. There are only
a few methods needed from your SPI class. Please see below for example SPI
implementations.


Usage
---------

There are two options for including this library with your code:

1. You can simply drop MFRC522.cpp and MFRC522.hpp directly with the rest of
   your source files and add them to your build system. Please see the examples
   folder to see how to do this.

2. You may also use this as a git submodule and integrate it as a CMake project.
   This method allows you to upgrade this library simply by calling git pull
   within the submodule folder.


Documentation
-----------------

This library only implements communication with the MFRC522 via a SPI.
As stated before, the code does not care how the SPI is implemented as
long as it has the following accessible methods:

<Class>::write();
<Class>::read();
<Class>::transfer();
<Class>::nssLow();
<Class>::nssHigh();
<Class>::~Class();

These should be pretty simple, but more explanation can be found in the examples
directory. It should be noted that the MFRC522 class expresses strong ownership
semantics of the SPI peripheral, so your implementation must allow for that.


Contributions
-----------------

If you wish to contribute to this project please follow these steps:

1. Identify an existing issue or create a new issue before beginning any work.
   Please state that you'd like to work on the given issue and ask any necessary
   questions.

2. When you have finished making your changes, please open a new PR through github,
   explain the changes and mention the issue number that will be fixed. This code
   will be reviewed and further changes may be suggested.

If you have any questions for any reason, feel free to send me an email to the address
found on my Github profile.
