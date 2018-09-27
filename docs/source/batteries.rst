Batteries
=========

All the three musketeers_ contain three sealed lead-acid batteries accessible through a hinged and latched rear door. The batteries charge life typically ranges from two to three hours.

.. important::
    Batteries have a significant impact on the balance and operation of your robot. Under
    most conditions, we recommend operating with three batteries. Otherwise, a single battery should be
    mounted in the center, or two batteries inserted on each side of the battery container. 

        By Mobile Robots©

.. _musketeers: robots.html

Batteries Specs
~~~~~~~~~~~~~~~
- Lead-acid
- Sealed
- 12 VDC
- 4 Ah
- With 3 batteries, 252 Wh
- Hot-swappable


Battery Indicators and Low Voltage Conditions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    The User Control Panel [1]_ has a bi-color LED labeled BATTERY that visually indicates current battery voltage. From approximately 12.5 volts and above, the LED glows bright green. The LED turns progressively orange and then red as the voltage drops to approximately 11.5 volts.

    Arually, the buzzer will sound a repetitive alarm if the battery voltage drops below 11.5 VDC. If the battery voltage drops below 11 VDC the microcontroller automatically shuts down a client connection and notifies the computer to shut down.

    .. note::
        The batteries voltage is monitored by a own package, this package if necessary notifies the user and shut down automatically the operating system. See more in `robot monitor`_.

.. _robot monitor: monitor.html#batteries
..  todo: make this package

Recharging
~~~~~~~~~~

Standart Charger
----------------
    This accessory recharge the batteries in the fast-charge mode (4A maximum current). The fast-charge mode is showed with an orange LED and trickle mode by a green LED, which the batteries are given only enough current to remain at full charge.

    .. warning::
        In the fast-charge mode, care must be taken to charge at least two batteries at once. A single battery may overcharge and thereby damage both itself and the robot.

Power Cube
----------
    This accessory allows simultaneous recharge of three batteries outside the robot. 


.. [1] See in the `manual`_.

.. _manual: https://github.com/lara-unb/amora/blob/master/pdfs/Pioneer%203AT%20Manual.pdf
