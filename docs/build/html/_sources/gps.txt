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
- Single frequecy


Power Board
~~~~~~~~~~~

.. image:: /img/rev2.JPG
   :scale: 25%

A circuit board was built to integrate gps and imu readings, to comunicate with onborad pc and to power the gps receiver and the imu.

.. figure:: /img/case_gps.jpg
   :alt: case gps

   Power Board with GPS Receiver and IMU (below GPS) in its case.