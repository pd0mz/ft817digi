=============
 FT-817 Plus
=============

This is the software for the FT-817 plus project, which will add the
following functionality to your Yaesu FT-817 using an Atmega168 powered
Arduino Duemilanove (or compatible).

Features
========

  * Menu with five hardware buttons (up, down, left, right and select)

  * VSWR meter (1:1 .. 9:1) suitable for QRP (up to 5 Watts)

  * 1200 baud TNC compatible modem for AX.25

  * 9600 baud CAT passthrough mode

  * 9600 baud CAT interface, for displaying device information

Requirements
============

We are using Debian 6 for development, but what you basically need to
compile the project is:

  * GNU `Make <http://www.gnu.org/s/make/>`_

  * AVRGCC (`AVR32 GNU Toolchain <http://avrfreaks.net/AVRGCC/>`_)

  * AVRDUDE (`AVR Downloder/UploaDEr <http://www.nongnu.org/avrdude/>`_)

