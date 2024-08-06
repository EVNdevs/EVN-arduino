Troubleshooting
===============

.. note:: This page is a work-in-progress! Expect it to be filled up over time.

**My Serial prints at the start of the program are not appearing in Serial Monitor.**

* This is an small issue with all RP2040-based boards. By the time the COM Port appears and Serial Monitor can be opened, the RP2040 may have already begun running its program for a few seconds.
* 2 solutions for this:
    * Wait for Serial Monitor to be opened before printing

    .. code-block:: c++

        void setup()
        {
            while (!Serial);

            //normal void setup() code can go here
        }

    * If you need to run your program with the robot being, you can delay void setup() for a few seconds.

    .. code-block:: c++

        void setup()
        {
            delay(5000);
            
            //normal void setup() code can go here
        }

