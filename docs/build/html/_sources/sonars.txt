Sonars
======

Athos and Porthos has two sonar arrays (front and rear), while Aramis have only one (front) sonar array, sonar or ultrasoud is a sensor that uses sound wave for detect obstacles and range information for collision avoidance.

.. image:: /img/pioneer3at_sonars.png

Sonar Specs
~~~~~~~~~~~

- Range of view: 0.1 m ~ 5 m
- Aquisition rate: 25 Hz

.. warning::
  
    If the sonar doesn't view anything in its cone of view, it will send to the software the max range.

Geometry
~~~~~~~~

The position of each sonar is showed in the image below.

.. image:: /img/IeTo5.jpg

.. important::
    - All these locations are fixed in the robot.
    - There're a URDF file, describing these locations to ROS. Please see the section `description of the robot`_.

.. _description of the robot: urdf.html#urdf

Sensitivity Adjustment
~~~~~~~~~~~~~~~~~~~~~~

The driver electronics for each array is calibrated at the factory. However, you may adjust the array’s sensitivity and range to accommodate differing operating environments. The sonar gain control is on the underside of the sonar driver board, which is attached to the floor of each sonar module.

Sonar sensitivity adjustment controls are accessible directly, although you may need to remove the Gripperto access the front sonar, if you have that accessory attached. For the front sonar, for instance, locate ahole near the front underside of the array through which you can see the cap of the sonar-gain adjustment  potentiometer. Using a small flat-blade screwdriver, turn the gain control counterclockwise to make the sonar less sensitive to external noise and false echoes.

Low sonar-gain settings reduce the robot’s ability to see small objects. Under some circumstances, that is desirable. For instance, attenuate the sonar if you are operating in a noisy environment or on uneven or highly reflective floor, a heavy shag carpet, for example. If the sonar are too sensitive, they will “see” the carpet immediately ahead of the robot as an obstacle.

Increase the sensitivity of the sonar by turning the gain-adjustment screw clockwise, making them more likely to see small objects or objects at a greater distance. For instance, increase the gain if youare operating in a relatively quiet and open environment with a smooth floor surface.

    By Mobile Robots©

.. hint::

    See more in the `manual`_.

.. _manual: manual.pdf

Software
~~~~~~~~

We use the `p2os`_ to read the sonars readings and the `sonar_viz`_ to transform the readings in standart data to better maniplation.

.. _p2os: p2os.hml
.. _sonar_viz: sonar_viz.html
