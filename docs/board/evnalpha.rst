``EVNAlpha``
========================================

EVNAlpha is a class used to interface with the onboard hardware on EVN Alpha:

* Button
* LED
* I2C Multiplexers
* Battery ADC

Some Technical Details
----------------------

EVNAlpha uses **hardware interrupts** for the onboard button to act as an toggle switch (default) or pushbutton. This approach allows other libraries to use the values read by EVNButton, without the user having to poll and work around blocking functions.

By default, the button output is also linked to 2 other functions. Both can disabled by the user if needed:

* Motor Operation: the button can act as a motor disable switch, so that users can pause their robot before uploading
* LED: the LED acts as an indicator for the button output

EVN Alpha has 2 TCA9548A I2C multiplexers, 1 on each I2C bus. This allows users to connect multiple I2C devices with the same I2C address without worrying about address clashing. However, users must set the port (1-16) for a given peripheral before communicating with it. EVNAlpha includes functions for port selection and de-selection to ease this process.

.. note::
    Our EVN-specific peripheral libaries (e.g. EVNColourSensor) do the port selection automatically, so port selection functions are not needed for them.

.. note::
    For Ports 9-16, 3rd party libraries and end-user code have to use Wire1 to interface with their sensors, as these ports are connected to the I2C1 bus.

.. class:: EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = false, bool button_invert = false, uint32_t i2c_freq = 100000)
    
    :param mode: Determines behaviour of ``buttonRead()``. Defaults to ``BUTTON_TOGGLE``

        * ``BUTTON_TOGGLE`` -- Returns false on startup and toggles between true and false with each button press
        * ``BUTTON_PUSHBUTTON`` -- Returns true only when button is pressed
        * ``BUTTON_DISABLE`` -- Always returns true

    :param link_led: Links LED to display ``buttonRead()`` output. Defaults to ``true``.

    :param link_movement: Links all motor and servo operation to ``buttonRead()`` output. Defaults to ``false``.

    :param button_invert: Inverts output of ``buttonRead()``. Defaults to ``false``.

    :param i2c_freq: I2C frequency in Hz. Defaults to 400000 (400kHz).

    .. code-block:: cpp

        //constructor with default options
        EVNAlpha board;

        //constructor if you wish to set non-default options
        EVNAlpha board(BUTTON_PUSHBUTTON, false, false, false, 400000);

Functions
---------
.. function:: void begin()

    Initializes on-board hardware. This function should be called at the start of ``void setup()``, before anything else.

    .. code-block:: cpp

        EVNAlpha board;

        void setup()
        {
            board.begin();
        }

LED / Button
""""""""""""

.. function::   bool read()
                bool buttonRead()

    Get button output (varies depending on mode parameter in class constructor)

    :returns: boolean signifying button output

    .. code-block:: cpp

        bool button_output = board.read();

.. function::   void write(bool state)
                void ledWrite(bool state)

    Set LED to turn on (``true``) or off (``false``). However, the LED state can be overridden by the battery reading functions (see below).

    :param state: state to write to LED

    .. code-block:: cpp

        board.write(true);  //LED on
        board.write(false); //LED off

I2C Port Control
""""""""""""""""

These functions will be used mainly if you are trying to operate third-party I2C devices, that aren't Standard Peripherals.

.. function:: void setPort(uint8_t port)

    :param port: I2C port to be enabled (1-16)

    .. code-block:: cpp
        
        //set I2C port 16 to be active
        board.setPort(16);

.. function:: uint8_t getPort()

    :returns: last I2C port called using ``setPort()`` (1-16)

    .. code-block:: cpp

        int port = board.getPort(); //returns 1 on startup
    
.. function:: uint8_t getWirePort()

    :returns: last Wire I2C port called using ``setPort()`` (1-8)

    .. code-block:: cpp
        
        int wport = board.getWirePort();    //returns 1 on startup

.. function:: uint8_t getWire1Port()

    :returns: last Wire1 I2C port called using ``setPort()`` (9-16)

    .. code-block:: cpp
        
        int w1port = board.getWire1Port();  //returns 9 on startup

.. function:: void printPorts()

    This is an I2C port scanner function which prints all I2C devices on every port using ``Serial``

    .. code-block:: cpp
        
        board.printPorts();

    Example Serial Monitor Output:

    .. code-block::

        EVN Alpha I2C Port Scanner
        Battery: 8.183V | Cell 1: 4.096V | Cell 2: 4.087
        Port 16: 0x6A

    Even though no peripherals are connected to the board, port 16 has one I2C device under address 0x6A, which is our onboard battery charger and voltage measurement device.

Battery Voltage Reading
""""""""""""""""""""""""
All battery voltage reading functions have a ``flash_when_low`` input. 
This is a low battery alert function, which flashes the LED at a rate of 5Hz (5 blinks per second) when the battery voltage is too low.

When the alert is on, the LED's previous output (whether linked to button or controlled by the user) will be overridden.
To add the alert to your code, add ``getBatteryVoltage()`` (or ``getCell1Voltage()`` **and** ``getCell2Voltage()``) to ``void loop()`` and they will check the voltage each loop.

.. code-block:: c++

    void loop()
    {
      //main code here
      
      board.getBatteryVoltage(); //battery alert!
    }


.. function:: int16_t getBatteryVoltage(bool flash_when_low = true, uint16_t low_threshold_mv = 6900)

    :param flash_when_low: Sets LED to flash when battery voltage falls below ``low_threshold_mv``. Defaults to ``true``
    :param low_threshold_mv: Battery voltage threshold (in millivolts). When battery voltage falls below this voltage and ``flash_when_low`` is ``true``, low voltage alert is triggered. Defaults to 6900.

    :returns: combined voltage of both battery cells in millivolts
    
    .. code-block:: c++

        int battery = board.getBatteryVoltage();
        
.. function:: int16_t getCell1Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3450)

    Cell 1 refers to the cell nearer to the edge of the board.
    
    :param flash_when_low: Sets LED to flash when battery voltage falls below ``low_threshold_mv``. Defaults to ``true``
    :param low_threshold_mv: Cell voltage threshold (in millivolts). When this cell's voltage falls below this threshold and ``flash_when_low`` is ``true``, low battery alert is triggered. Defaults to 3450.

    :returns: voltage of first cell in millivolts

    .. code-block:: c++

        int cell1 = board.getCell1Voltage();

.. function:: int16_t getCell2Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3450)

    Cell 2 refers to the cell nearer to the centre of the board.

    :param flash_when_low: Sets LED to flash when battery voltage falls below ``low_threshold_mv``. Defaults to ``true``
    :param low_threshold_mv: Cell voltage threshold (in millivolts). When this cell's voltage falls below this threshold and ``flash_when_low`` is ``true``, the low battery alert is triggered. Defaults to 3450.

    :returns: voltage of second cell in millivolts

    .. code-block:: c++

        int cell2 = board.getCell2Voltage();

Set Functions
"""""""""""""
.. function:: void setMode(uint8_t mode)

    :param mode: Determines behaviour of ``buttonRead()`` (options shown below)
    
    * ``BUTTON_TOGGLE``
    * ``BUTTON_PUSHBUTTON``
    * ``BUTTON_DISABLE``

    .. code-block:: c++

        board.setMode(BUTTON_TOGGLE);

.. function:: void setLinkLED(bool enable)

    :param enable: Links LED to display ``buttonRead()`` output

    .. code-block:: c++

        board.setLinkLED(true);

.. function:: void setLinkMovement(bool enable)

    :param enable: Links all motor and servo operation to ``buttonRead()`` output

    .. code-block:: c++

        board.setLinkMovement(true);

.. function:: void setButtonInvert(bool enable)

    :param enable: Inverts output of ``buttonRead()``

    .. code-block:: c++

        board.setButtonInvert(true);

Get Functions
""""""""""""""

.. function:: uint8_t getMode()

    This function returns the button mode in numbers, as shown below.

    The written button modes (e.g. ``BUTTON_TOGGLE``, ``BUTTON_PUSHBUTTON``) are converted to these numbers when compiled, 
    so statements like ``if (board.getMode() == BUTTON_TOGGLE) {}`` are valid.

    :returns: Mode of button in numerical form
    
    * 0 (``BUTTON_DISABLE``)
    * 1 (``BUTTON_TOGGLE``)
    * 2 (``BUTTON_PUSHBUTTON``)

    .. code-block:: c++

        if (board.getMode() == BUTTON_TOGGLE)
        {

        }

.. function:: bool getLinkLED()

    :returns: Whether LED is linked to ``buttonRead()`` output

    .. code-block:: c++

        bool link_led = board.getLinkLED();

.. function:: bool getLinkMovement()

    :returns: Whether motor and servo operation is linked to ``buttonRead()`` output

    .. code-block:: c++

        bool link_movement = board.getLinkLED();

.. function:: bool getButtonInvert()

    :returns: Whether output of ``buttonRead()`` is inverted

    .. code-block:: c++

        bool button_invert = board.getButtonInvert();