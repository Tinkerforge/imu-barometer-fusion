Altitude Estimation with IMU Brick and Barometer Bricklet 
=========================================================

This is an example program doing sensor fusion between the IMU Brick and
the Barometer Bricklet.
We try to estimate a more reliable altitude by combining air pressure
measurements from the Barometer Bricklet with the acceleration measurements 
and the quaternion from the IMU Brick.

The result is drawn on a live graph with Qwt.

There is still some drift in the Barometer data that is not removed. Perhaps
we can improve that by imcorporating GPS data in the future! Also there is
likely room for improvement by simply twiddling with the velocity, position
and integral gain (KP1, KP2 and KI).

Links
-----

* Youtube video: http://youtu.be/TaqtzG7lyp0
* IMU Brick: http://www.tinkerforge.com/en/doc/Hardware/Bricks/IMU_Brick.html
* Barometer Bricklet: http://www.tinkerforge.com/en/doc/Hardware/Bricklets/Barometer.html
