Welcome to EVN!
===================================

**EVN** is a Robotics System that we view as the natural evolution of LEGO® MINDSTORMS®.

It is built around the EVN Alpha - a Technic-compatible robot controller with 26 ports for I2C, UART, Servos, EV3 and NXT Motors, etc.

On this page, you will find user guides for EVN Alpha, hardware information and documentation for the EVN Arduino library, 
which uses the Arduino-Pico core by Earle Philhower for RP2040-based boards.

To find out more about purchasing, visit our `product website`_.

.. _product website: https://coresg.tech/evn/

.. image:: images/brick_side.JPG
   :width: 70%
   :align: center

.. toctree::
   :maxdepth: 1
   :caption: Getting Started

   getting-started/first-boot
   getting-started/installation
   getting-started/programming
   getting-started/hardware-overview
   getting-started/standard-peripherals
   getting-started/troubleshooting

.. toctree::
   :maxdepth: 2
   :caption: Library Reference

   board/index
   actuators/index
   sensors/index
   displays/index
   others/index

.. toctree::
   :maxdepth: 1
   :caption: Guides

   guides/compass-cal
   guides/third-party-i2c
   guides/platformio
..
   guides/soldering
   guides/third-party-motors
   guides/motor-pid-cal
   guides/drivebase-pid-cal

Licenses
--------

The EVN libraries are licensed under GPL.

In addition, it contains code from additional open source projects:

* Arduino-Pico is licensed under the LGPL license.
* The Arduino IDE and ArduinoCore-API are developed and maintained by the Arduino team. The IDE is licensed under GPL.
* The RP2040 GCC-based toolchain is licensed under under the GPL.
* The Pico-SDK and Pico-Extras are by Raspberry Pi (Trading) Ltd. and licensed under the BSD 3-Clause license.
* The U8g2 Arduino library by olikraus is licensed under the BSD 2-Clause license.
* The U8g2 font used is part of The Ultimate Oldschool PC Font Pack, which is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
* Pololu's VL53L0X Arduino library is largely based on STMicroelectronics' VL53L0X API (licensed under the BSD 3-Clause license). The rest is licensed under the MIT license.

Acknowledgements
----------------

We'd like to thank the following people for their contributions to the Kickstarter:

.. list-table::
   :widths: 33 33 33
   :header-rows: 0

   * - Robots-Blog.com
     - Arthur Tan
     - Robert Lau
   * - Tan Chien Hao
     - shrmnk
     - John Seet
   * - Heng Jia Min
     - Jim Anderson
     - Steven Lim
   * - robotins
     - Alice Teoh
     - Justin H
   * - Colin "TinMan" Kulsa
     - Don Meyer
     - Lars Shakya Buch-Jepsen
   * - Roel Thijs
     - bechuts
     - Andrea Anderlini
   * - WhizNut
     - Heng Meng Lek
     - shenc
   * - Jonathan Carpenter
     - Volker Schmidt
     - davalillo
   * - Sam "Pasamio" Moffatt
     - Ashlee 🐈
     - mrshu
   * - Stéphane Hulard
     - Shannon Prescott
     - 

Without them (and many more unnamed backers), EVN would not exist today.