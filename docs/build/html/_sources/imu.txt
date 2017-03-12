IMU
===

.. image:: /img/imu.png
   :scale: 50%

The MEMSense NanoIMU is a 9DOF IMU with thermometer used in the robots together with GPS receiver. We have a custom software_ and a hardware_ to power and send the readings to the computer.

.. _software: imu_soft.html
.. _hardware: imu.html#power-board

Specs
~~~~~

Gyrometer Specs
---------------
- Dynamic Range           +-300 º/s
- Offset                  +-1.5 º/s
- Cross-axis sensitivity  +-1 %
- Nonlinearity            +-0.1 % of FS (Best fit straight line)
- Noise                   0.56 (max 0.95) º/s, sigma
- Digital Sensitivity     1.3733E-2
- Bandwidth               50 Hz

Accelerometer Specs
-------------------
- Dynamic Range           +-2 g
- Offset                  +-30 mg
- Nonlinearity            +-0.4 (max +-1.0) % of FS
- Noise                   0.6 (max 0.8) mg, sigma
- Digital Sensitivity     9.1553E-5
- Bandwidth               50 Hz

Magnetometer Specs
------------------
- Dynamic Range           +-1.9 gauss
- Drift                   2700 ppm/ºC
- Nonlinearity            +-0.5 % of FS (Best fit straight line)
- Noise                   0.00056 (max 0.0015) gauss, sigma
- Digital Sensitivity     8.6975E-5
- Bandwidth               50 Hz

Thermometer Specs
-----------------
- Digital Sensitivity     1.8165E-2


Power Board
~~~~~~~~~~~

.. figure:: /img/rev2.JPG
   :align: center
   :alt: Power Board

   Power Board

The circuit board is the same for GPS receiver, more informations here_.

.. _here: gps.html#power-board
