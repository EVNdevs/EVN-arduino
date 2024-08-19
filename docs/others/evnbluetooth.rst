``EVNBluetooth``
================

The Bluetooth Module Standard Peripheral uses the HC-05 Bluetooth Module. 

This module operates over the Serial UART Communication Protocol, and can be set into 2 modes: Host & Remote.

Running as a **Remote** device means that a Bluetooth host (such as your laptop or phone) can connect to it and transmit commands or receive data.

Running as a **Host** device allows you to connect to other Remote devices, which can be used for communication between 2 Bluetooth Modules (1 Host & 1 Remote).

The module's settings are stored on the module. To access and edit those settings, users must hold a button to start it in Program Mode (AT mode) and send serial commands.

This library is intended to simplify the process of confuguring settings like baud rate and host/remote mode, such that manual text commands are not needed.

If the module is started in Program Mode, this library can configure settings and retrieve some useful information. Otherwise, this library does nothing.

Normal usage of the module (in both modes) is via ``Serial1`` or ``Serial2`` (depending on which Serial port is used), which have the same functions as ``Serial``.

Wiring (Serial)
---------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   VCC         3.3V Power
GND   GND         Ground (0V)
RX    TX          Host Receive
TX    RX          Host Transmit
====  ==========  ===========

Starting Module in Program Mode
--------------------------------

This only needs to be done once to set the proper settings.

1. Set EVN into Off mode using the On/Off button.
2. Press and hold the Bluetooth Module's onboard button.
3. Set EVN into On mode using the On/Off button.
4. Hold the button for 2s before releasing it. If the Bluetooth Module's red LED starts blinking once every 2 seconds (rate of 0.5Hz), it has successfully entered program mode.
5. Now that our module has been set into Program Mode, you most likely need to restart your program, so that our library can configure the module. Do this by pressing the Reset button.

Constructor
-----------

.. class:: EVNBluetooth(uint8_t serial_port, uint32_t baud_rate = 9600, char* name = (char*)"EVN Bluetooth", uint8_t mode = BT_REMOTE, char* addr = NULL);

    :param serial_port: Serial port the sensor is connected to (1 or 2)

    :param baud_rate: Baud rate used when not in Program Mode. Defaults to 9600. Supported baud rates: 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1382400
    
    :param name: Name of Bluetooth device. Defaults to ``(char*)"EVN Bluetooth"``. Store in a char array, or cast to ``char*`` before passing into this constructor.
    
    :param mode: Mode to run module in. Defaults to ``BT_REMOTE``

        * ``BT_HOST``: Host Mode
        * ``BT_REMOTE``: Remote Mode

    :param addr: Bluetooth Address of the device to connect to. This is only used in Host Mode. Store in a char array, or cast to ``char*`` before passing into this constructor.

    .. code-block:: cpp
        
        EVNBluetooth host(2, 9600, (char*)"EVN Bluetooth Host", BT_HOST, (char*)"19:8:34FE");

Functions
---------

All of these functions only do something when the module is in Program Mode. In normal operation, use ``Serial1`` or ``Serial2`` to transmit and receive data, like a normal Serial connection.

.. function:: bool begin(bool exit_program_mode = true)

    If the module is set to Program Mode, this function writes all settings to the module, before exiting program mode depending on user input.

    :param exit_program_mode: Whether to exit Program Mode after updating settings. Defaults to ``true``

    .. code-block:: cpp
        
        bt.begin();

.. function:: bool inProgramMode()
    
    :returns: ``true`` the module is in Program Mode, ``false`` otherwise

    .. code-block:: cpp
        
        bt.inProgramMode();

.. function:: bool exitProgramMode()

    Exits Program Mode for normal operation

    :returns: ``true`` if module is not in Program Mode, ``false`` otherwise

    .. code-block:: cpp
        
        bt.exitProgramMode();

.. function:: bool factoryReset()

    Resets all settings stored in the module to their factory defaults. This function is useful for resetting more advanced settings which are not exposed by this library.

    :returns: ``true`` if module was in Program Mode & successfully reset, ``false`` otherwise

    .. code-block:: cpp
        
        bt.factoryReset();

Retrieving Module Info
----------------------

These functions only work when the module is in Program Mode.

.. function:: bool getAddress(char* array)

    Writes address of Bluetooth Module (just the address, no ``Address: `` prefix) to given array.

    :param array: memory address of character array to write address to

    :returns: ``true`` if module was in Program Mode and address was successfully retrieved, ``false`` otherwise

    .. code-block:: cpp

        char address[32] = {0};
        bt.getAddress(address);

.. function:: void printAddress()

    Prints address of Bluetooth Module using ``Serial``. Write this address to the constructor of another EVNBluetooth object in Host Mode to connect to it.

    :returns: ``true`` if module was in Program Mode and address was successfully retrieved, ``false`` otherwise

    .. code-block:: cpp

        bt.printAddress();

    Example Output on Serial Monitor:

    .. code-block::

        Version: 19:8:34FE

        //If you are connecting to this Module with another Module in Host Mode, place (char*) "19:8:34FE" in the constructor for the Host Module.

.. function:: void getVersion(char* array)

    Writes firmware version of Bluetooth Module (just the version, no ``Version: `` prefix) to given array.

    :param array: memory address of character array to write address to

    :returns: ``true`` if module was in Program Mode and version was successfully retrieved, ``false`` otherwise

    .. code-block:: cpp

        char version[32] = {0};
        bt.getVersion(version);

.. function:: void printVersion()

    Prints firmware version of Bluetooth Module using ``Serial``.

    :returns: ``true`` if module was in Program Mode and version was successfully retrieved, ``false`` otherwise

    .. code-block:: cpp

        bt.printVersion();

    Example Output on Serial Monitor:

    .. code-block::

        Version: 3.0-20170601


Directly Programming the Module in Program Mode
-----------------------------------------------

A example sketch for using the Serial Monitor to program the Bluetooth Module using AT commands can be found in Examples
(Examples > EVN > Others > Debug > bluetoothAT).