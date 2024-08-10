Programming with PlatformIO on VSCode
=====================================

.. image:: ../images/platformio/platformio.png

As the Arduino-Pico core `docs`_ state:

    "PlatformIO is a free, open-source build-tool written in Python, which also integrates into VSCode code as an extension.

    PlatformIO significantly simplifies writing embedded software by offering a unified build system, yet being able to create project files for many different IDEs, 
    including VSCode, Eclipse, CLion, etc. Through this, PlatformIO can offer extensive features such as IntelliSense (autocomplete), debugging, unit testing etc., 
    which are not available in the standard Arduino IDE."

Despite the added features, all your programs can still be written in Arduino code, and basic functions like compile, upload and serial monitor are present.

If you're wondering why we don't promote PlatformIO as the default way to program EVN, 
it doesn't officially support the Arduino-Pico core that our libraries build upon, 
so it can be a little more troublesome to setup. But if you're up for it, here we go!

Steps
-------

1. Install the latest version of `Git`_ for your OS. 

2. Install the latest version of `Visual Studio Code`_ for your OS.

3. **(For Windows Users only)** Enabling Long Paths in Windows

    Taken from the Arduino-Pico core `docpage`_ on using PlatformIO:

    * Open a Command Prompt window and execute the following command. Depending on your Git install, you may need to run Command Prompt to run as administrator (Instead of clicking Command Prompt to open, right-click and select "Run as administrator".

    .. code-block::

        git config --system core.longpaths true

    .. image:: ../images/platformio/platformio2.png

    * Enable long paths in the OS

        1. Click Windows key+R and type gpedit.msc, then press the Enter key. This launches the Local Group Policy Editor.

        .. image:: ../images/platformio/platformio3.png

        2. Navigate to Local Computer Policy > Computer Configuration > Administrative Templates > System > Filesystem.
        
        .. image:: ../images/platformio/platformio4.png

        3. Double click Enable NTFS/Win32 long paths. This should popup a window.

        .. image:: ../images/platformio/platformio5.png

        4. Select Enabled, then click Apply, followed by OK.
    
    * Reboot your computer for the changes to take effect

4. Install the PlatformIO IDE Extension on Visual Studio Code

    Open Visual Studio Code.

    Next, open the Extensions Panel by clicking the Extension Button on the left side of Visual Studio Code.

    Search for "PlatformIO IDE" in the search bar and install the first result.

    .. image:: ../images/platformio/platformio6.png

5. Download (or Git Clone) our Example PlatformIO Project

    In `this repository`_ you'll be able to download our example project to use as a template for other projects.

    After downloading as a ZIP file, unzip it to obtain the project folder.

    Alternatively, execute the command below in Command Prompt (Windows) or Terminal (Mac/Linux) to clone the repository.

    .. code-block::

        git clone https://github.com/HTY2003/EVN-PlatformIO-Example.git

6. Open the project in Visual Studio Code (File > Open Folder > Select your project folder).

    Once opened, PlatformIO should recognise this folder as a PlatformIO project and begin installing the necessary frameworks and toolchains for compilation.

    The initial setup can take some time, so you might want to grab a coffee.

    Meanwhile, here is the folder structure for our example project:

    .. code-block::

        |__ include
        |   |__ README
        |__ lib
        |   |__ README
        |__ test
        |   |__ README
        |__ src
        |   |__ main.cpp
        |__ platformio.ini
        |__ .gitignore

    On the bottom, you'll find the PIO toolbar, which contains the familiar buttons for Compile, Upload and Serial Port Monitor.

    .. image:: ../images/platformio/platformio7.png

    ``platformio.ini`` contains many important project settings, that we have configured to work with the Arduino-Pico Core and EVN libraries.

    ``src/main.cpp`` is where your program will be. Note that this is not an ``.ino`` file, and you need to add ``#include <Arduino.h>`` to the start of the file.

    ``lib/`` are where your libraries go. This is unlike Arduino IDE, where all installed libraries can be used by any sketches on the same computer.

.. note:: Creating a "New Project" without using our example template is possible, but you will need to swap out the contents of ``platformio.ini`` with the contents from our example.

Additional Resources
--------------------

Here are some additional resources you may want to explore:

* PlatformIO Documentation: https://docs.platformio.org/en/latest/index.html
* PlatformIO CLI reference: https://docs.platformio.org/en/latest/core/index.html
* PlatformIO ``platformio.ini`` Reference: https://docs.platformio.org/en/latest/projectconf/index.html
* Arduino-Pico docs regarding PlatformIO: https://arduino-pico.readthedocs.io/en/latest/platformio.html

.. _Git: https://git-scm.com/downloads
.. _Visual Studio Code: https://code.visualstudio.com/download
.. _docpage: https://arduino-pico.readthedocs.io/en/latest/platformio.html
.. _docs: https://arduino-pico.readthedocs.io/en/latest/platformio.html
.. _this repository: https://github.com/HTY2003/EVN-PlatformIO-Example
