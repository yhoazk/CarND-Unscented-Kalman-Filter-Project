# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/4d0420af-0527-4c9f-a5cd-56ee0fe4f09e)
for instructions and the project rubric.



### References:
[https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)


### Data
The two data files that you will be using are the same from EKF. Again each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next following values are either the 2 lidar position measurements (x,y) or the 3 radar position measurements (rho, phi, rho_dot). The next value is then the time stamp and then finally the ground truth values for x, y, vx, vy.

### NOTE:
In Data 2, the starting lidar measurements for x, y are both zero, and this special case can create problems for both the EKF and UKF lidar update states. One way to catch for this is to observe when both px, py are zero and instead set them to some small floating value.

#### Formulas:
##### L14: Generate Sigma Points
![](./ukf_l14/l14_formulas.png)
##### L17: Augmentation
![](./ukf_l17/l17_form.png)
##### L21: Sigma Point prediction
![](./ukf_l21/Formulas.png)
##### L23: Predict Mean and Covariance
![](./ukf_l23/Formulas.png)
##### L26: Predict Radar measurements
![](./ukf_l26/formulas.png)
##### L29:UKF Update
![](./ukf_l29/formulas.png)
##### L31: Parameters and consistency (NIS)
![](./nis_formula.png)
