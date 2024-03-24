Installation
============

Arduino IDE Installation
------------------------

**Both** versions of Arduino IDE will work with this library:

* `Arduino IDE 2.2.1`_
* `Arduino IDE 1.8.19`_

.. _Arduino IDE 2.2.1: https://www.arduino.cc/en/software
.. _Arduino IDE 1.8.19: https://www.arduino.cc/en/software

Arduino-Pico Installation
--------------------------

After installing Arduino IDE, install Earle F Philhower's Arduino-Pico core (V3.6.3+). This is done in 4 steps:

1. Open Preferences in Arduino IDE (``File`` -- ``Preferences``)

2. Enter the following URL in the ``Additional Boards Manager URLs`` field: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

3. Open Boards Manager in Arduino IDE (``Tools`` -- ``Board`` -- ``Boards Manager...``)

4. Search and install ``"Raspberry Pi Pico/RP2040"`` by Earle F Philhower III. Install version 3.6.3 and above.

Updating Arduino-Pico Core
""""""""""""""""""""""""""

If you have an existing installation of the Arduino-Pico core, and want to update it, you may be unable to successfully compile after updating. This is an ``issue``_ with the Arduino IDE failing to extract the files correctly. 

.. _issue: https://github.com/arduino/Arduino/issues/11842

Currently, the best solution is to do a clean removal of the core before reinstalling.

1. Uninstall Arduino-Pico in Boards Manager. Close Arduino IDE.

2. Delete the following folder: ``C:\Users\%USERNAME%\AppData\Local\Arduino15\packages\rp2040``

3. Reopen Arduino IDE and do a fresh install as described above.

Library Installation
--------------------

Download the `GitHub repository`_ as a .zip file and install in Arduino IDE (
``Sketch`` -- ``Include Library`` -- ``Add .ZIP Library`` -- Select downloaded .zip file).

.. _GitHub repository: https://www.github.com/HTY2003/EVN-arduino

.. note::

    We plan to publish the EVN library to the Arduino IDE Library Manager by official release. Once that happens, the installation process will be simpler!