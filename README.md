Cubature Kalman Filter in C++
------------------------------

Description
------------

Headers only Cubature Kalman Filter (CKF) [[1]](#ref1) implementation in C++. The CKF is a variation of the Unscented Kalman Filter (UKF) [[2]](#ref2) with a spherical-radial cubature rule at its core. 

This library makes use of the [Eigen](http://eigen.tuxfamily.org) library for lineal algebra routines and matrix operations.


Installation
-------------

This is a heades only library, for usign them in your project you only need to add them to your file and point at the correct location during compilation

```C++
	#include "CKF.cpp"
```

Example
-------

To compile the example, simply run the make command on the proper directory

```
	cd <directory where files are>
	cmake .
	make
	./example
```

The example will display a list of 20 steps where the actual state is compared with the estimated measurement from the CKF.

```
	Difference between actual states and tracked measurements:
	0: 0.12136  (1.00000-1.12136)
	1: 1.05234  (2.00000-3.05234)
	2: 0.70898  (4.00000-4.70898)
	3: 0.81015  (7.00000-7.81015)
	4: 0.10093  (11.00000-10.89907)
	5: 0.20019  (16.00000-16.20019)
	6: 0.27947  (22.00000-21.72053)
	7: 0.00576  (29.00000-28.99424)
	8: 0.03688  (37.00000-37.03688)
	9: 0.16920  (46.00000-45.83080)
	10: 1.35327  (56.00000-57.35327)
	11: 0.33129  (67.00000-67.33129)
	12: 0.52171  (79.00000-79.52171)
	13: 0.39297  (92.00000-92.39297)
	14: 0.18210  (106.00000-105.81790)
	15: 0.16014  (121.00000-120.83986)
	16: 0.16330  (137.00000-137.16330)
	17: 0.77446  (154.00000-154.77446)
	18: 0.62436  (172.00000-172.62436)
	19: 1.97809  (191.00000-192.97809)
```

Usage
------

To use it, a class with the state and measurement functions must be declared 

```C++
	class CKF2DPoint : public CKF {
	public:
		MatrixXd stateFunction (MatrixXd s)
		{
			MatrixXd state(4,1);
			state(0,0) = s(0,0)+s(2,0);	// x position in 2D point
			state(1,0) = s(1,0)+s(3,0);	// y position in 2D point
			state(2,0) = s(2,0);		// velocity in x
			state(3,0) = s(3,0);		// velocity in y
			return state;
		}
		MatrixXd measurementFunction (MatrixXd m)
		{
			MatrixXd measurement(2,1);
			measurement(0,0) = m(0,0);	// measured x position in 2D point
			measurement(1,0) = m(1,0);	// measured y position in 2D point
			return measurement;
		}
	};
```
An object can be created and the remaining required values, namely the covariance, noise covariance and measurement covariance matrices need to be stated, as well as the matrices' size in the n and m values.

```C++
	CKF2DPoint tracked_point;
	tracked_point.n = n;
	tracked_point.m = m;
	tracked_point.P = MatrixXd;	// state covriance
	tracked_point.Q = MatrixXd;	// covariance of process     (size must be nxn)
	tracked_point.R = MatrixXd;	// covariance of measurement (size must be mxm)
```

The calculations are performed with

```C++
	tracked_point.ckf(x, z); 
```

Where x is the state and z is the measurement.

For more details, please refer to the example in main.cpp

Credits
--------

Basic Code Structure taken from https://github.com/Efreeto/UKF

Cubature Kalman Filter calculations taken from https://github.com/rlabbe/filterpy/


References
----------

<a name="ref1">[1]</a> Arasaratnam, Ienkaran, and Simon Haykin. "Cubature kalman filters." IEEE Transactions on automatic control 54.6 (2009): 1254-1269.

<a name="ref2">[2]</a> Wan, Eric A., and Rudolph Van Der Merwe. "The unscented Kalman filter for nonlinear estimation." Adaptive Systems for Signal Processing, Communications, and Control Symposium 2000. AS-SPCC. The IEEE 2000. Ieee, 2000.


License
--------

MIT License

Copyright (c) 2018 Saul Armendariz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.



