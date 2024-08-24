DIY Guide for EVN Alpha V1.4
============================

.. note:: This guide is incomplete (mainly missing pictures). Coming Soon!

What You Need
-------------

Components
""""""""""

* 1 EVN Alpha PCB

* 4 RJ11 Right Buckle Connectors (i.e. "the EV3 Port")

* Male 2.54mm Pin Headers of varying lengths and colours

    * 16-Pin: 1 Red, 1 Black, 1 Yellow, 1 Blue
    * 6-pin: 4 White 
    * 4-pin: 2 White, 1 Red, 1 Black, 1 Yellow

* 4 Omron B3F-1070 Tactile Pushbuttons (with clipped pins)

* 1 2-Cell 18650 Battery Holder

Components come with our Hacker Kit.
These are the quantities needed to assemble one EVN Alpha, but you will find that our kits come with a little extra in case something is damaged before or during assembly.

Equipment
"""""""""

* Soldering Iron
* Solder Wire

* Desoldering Pump (optional)

    * Not strictly necessary if you solder perfectly, but nice to have

* Fume Extractor (optional)

    * Fumes emitted from soldering can be harmful, so try to have a fume extractor or work in a ventilated environment :)

* Soldering Helping Hand (optional)

    * If your helping hand has sharp metal edges that can scratch the PCB, it may be better to do without it

This equipment will be sourced by yourself.

Iron Choice
-----------

We strongly recommend using a temperature-controlled soldering iron.

When it comes to iron choice, the most important specification for this project is most likely power.

We've soldered boards with both a Miniware TS100 portable iron (on a 45W power supply) as well as an AiXun T3A Soldering Station (200W power supply), so any iron in this range of power should work fine.

However, it is easier to solder with a high-power iron, due to reasons that will be explained below.

Apart from power, we also strongly recommend using a temperature-controlled soldering iron for ease of soldering and avoiding damage to the board.

Iron Tip
--------

Apart from melting solder, the iron also serves to heat up the solder pad on the PCB. The melted solder will "stick" to the heated pin and the heated pad, hence creating an electrical connection.

Try to use an iron tip that can easily contact the solder pad with a decent amount of surface area for better heat transfer. Thin needle-like tips may not be ideal.

But if the tip on your iron is all you have, you should be fine either way.

Tip Temperature
----------------

There is no right temperature, but generally irons are set anywhere betweem 280degC and 370degC.

Generally, you want to run the iron at the lowest temperature where the solder wire still melts fully and quickly, so this will depending largely on your solder. 

If you already have a temperature you usually use for your solder, use that.


However, tip temperature may also depend on iron power.

Our 200W iron easily maintains a temperature of 280 degrees even when touching a pad connected to a large ground plane (i.e. a big piece of copper).

However, our 45W iron struggles to do this, as it cannot heat the pad up fast enough even at max power output. Hence, our 45W iron needed to be set to a higher temperature (360 degC) so that when it first makes contact with the pad it has more heat to transfer.

So depending on your solder and iron power, you may have to adjust the temperature higher. Using a higher tip temperature may not be ideal, but is better than holding the iron to the PCB for a long period of time.

Header Pins
-----------

Each of the pin header is located on the top side of the board, in the layout shown below:

Realistically, as long as you can ensure the pin header is perpendicular to the PCB surface, you can solder them on however you want.

All pins are spaced on a 2.54mm grid, so you may even be able to make a jig with protoboard.

But here is how we do it:

1. Place the header in its PCB position and rest the PCB upside-down so that we can solder the pins.

    At rest, the header will most likely not be perpendicular to the PCB (hard to achieve without a jig), but that's okay.

    Solder the leftmost pin by melting the solder onto the iron tip, while touching the pin and the solder pad of the PCB.

    When the solder appears to stick to the pad and pin, you can remove the iron.
    
    We're going to melt this solder joint later, so it doesn't need to be perfect, just good enough to hold the header to the board.

    The ground pads (marked with squares) are connected to a large ground plane, so they take more time to heat up. 
    You may need to hold the iron against the pad for a longer amount of time, or set your iron temperature higher.

2. Pick up the PCB, with the pins facing up. While melting the solder joint you just created (allowing the header to move freely), press down on the unsoldered pins and rock the header back and forth. 

    The header should settle in a stable position where it is flush and perpendicular to the PCB and resists the rocking.

    Once that position is achieved, remove the iron.

3. For the long 16-pin header, you may have to repeat steps 1 and 2 for the rightmost pin to ensure that the pins are perpendicular to PCB across the entire header.

4. Solder the remaining pins. The solder joint should ideally look like a cone while adhering to the pin and solder pad. The solder should also be fully melted, so increase your tip temperature if this isn't the case.

5. Inspect that there are no solder bridges between the pins.

6. Repeat until all the headers are done!

RJ11 Right Buckle Connector
---------------------------

1. Ensure that the connector is mounted flush against the PCB using the same trick with the pin headers. 

2. Insert all the connectors into the PCB (connector touches bottom of PCB), and rest the board. Take care not to bend any pins during insertion.

3. Solder one pin on each connector to the PCB. Once done, inspect the connector and check if it is mounted flush against the PCB. If there is a noticeable gap, you can rectify it with the same pin-header trick we used: Melt the solder on the single soldered pin, and push the connector against the PCB before removing the iron and letting the solder solidify.

4. Once all the connectors have been inspected, proceed to solder the remaining 5 pins of all the connectors. Same with the pin headers, some pins may require more time to solder than others. To ensure a good connection, try to contact the tip of the iron against the solder pad on the PCB, not just the pin.

.. warning:: Take care not to accidentally touch onboard components when soldering! You may have to orient the board in a certain manner to do this.

Buttons
-------

The buttons for EVN Alpha are clipped such that the pin does not stick out of the PCB. This is to done to avoid clashing with the 18650 battery holder.

The most important thing to note when soldering the buttons is to avoid having large blobs of solder sticking out of the PCB.

Place all 4 buttons on the PCB (on the same side as the pins), making sure that they are resting flat against the PCB.

Place the board upside down and start soldering the pins. Ensure that the solder is fully liquified and flows into the hole for good electrical contact, but avoid adding too much solder. If there is too much solder, feel free to remove some of it with a desoldering pump and try again.

The end result should be that the soldered pad appears flat, without sticking out of the PCB.

Battery Holder
--------------

Place the battery holder in the 4. Any orientation of the holder works, but you may want to orient it such that the ugly trim mark on the side of the holder is hidden away.

The PCB solder pads for the holder are much larger than the holder pins. In order to ensure that the footprint of the holder does not exceed the outline of the PCB, push the holder closer to the EV3 ports, into the PCB outline as much as possible.

Once the holder has been positioned, solder the 4 large solder pads. Ensure that your solder melts fully for a good electrical connection.

3D Printing Your Shells
-----------------------

The 3D printable shells and shell assembly instructions can be found on this `Thingiverse page`_.

.. note:: We'll add more details here soon! For now, the instructions on the Thingiverse page should suffice.

.. _Thingiverse page: https://www.thingiverse.com/thing:6726710

Conclusion
----------

Your board is now ready! 

Afterwards, proceed to `First Boot` to get your board ready for upload.

Soldering Alpha isn't as tricky as SMD reflow work, but it is quite laborious due to the sheer number of pins to be soldered.
Our priority for Kickstarter was to get all core functionality up and running, so optimizing production to scale well was a matter that received less attention.

The next few versions of Alpha will most likely be targeted towards easing production and getting more components soldered right from the factory.

In the future, we may not even sell a DIY kit! At least not one with this many unsoldered pins. In the meantime, we hope this guide was helpful and made the soldering process a little less painful.