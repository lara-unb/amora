GPS Receiver
============

.. image:: /img/oemv-1.png

The robots have each one a Novatel GPS receiver model OEMV-1. We have a custom software_ to publish the GPS readings in ROS and a custom hardware_ to power the receiver.

.. _software: gps_soft.html
.. _hardware: gps.html#power-board

Specs
~~~~~
- GPS tracking
- L1, L-Band and SBAS signal tracking
- Low power consumption for longer operating time
- Single frequency


Antenna
~~~~~~~

.. image:: /img/gps_antenna.png

"The ANT-35C1GA-TW-N is an active GPS antenna, 88.9 mm (3.5”) in diameter, and designed to operate at the GPS L1 frequency of 1575.42 MHz.
Its mechanical configuration is a spherical radius molded radome which provides enhanced protection against rain and ice."[1]_

Power Board
~~~~~~~~~~~

.. figure:: /img/rev2.JPG
   :align: center
   :alt: Power Board

   Power Board

A circuit board was built to integrate GPS and IMU readings, to communicate with onboard pc and to power the GPS receiver and the IMU.

.. figure:: /img/case_gps.jpg
   :alt: case gps
   :align: center

   Power Board with GPS Receiver and IMU (below GPS) in its case.

.. [1] See more in the `antenna user guide`_.

.. _antenna user guide: https://github.com/lara-unb/amora/blob/master/pdfs/antenna_novatel.pdf
