# Session report 14: March 29, 2024

**Session objective:** 
Implement new speed PID corrections where the error is the difference between the estimated speed (which is this time calculated from a trapezoidal or triangular speed profile based on the position to be reached) and the actual speed of the robot.

---

## Reminder

During the previous sessions, I had written a function `get_phases_distance()` capable of estimating the duration of each phase (acceleration, constant speed, deceleration) needed to reach a specified distance, and another function `get_speed_target()` capable of determining the speed target to be reached at each instant based on this speed profile.

---

## Resolved issue with `get_speed_target()`

The function worked by comparing the remaining distance to travel with the distances needed to accomplish acceleration and deceleration phases to determine when to switch to the next phase. Each time it was invoked (based on sampling frequency), it incremented the speed target by either `max_acceleration` or `max_deceleration`, depending on the current phase.

However, this approach didn't work properly because there was no correlation between the sampling frequency at which acceleration was added and the distance traveled.

To address this issue, I modified the function to be based on **phase durations** rather than phase distances. For instance, the duration of the acceleration phase represents the number of times we need to increment the speed target by the `max_acceleration` coefficient to reach the desired maximum speed. 
So, each time `get_speed_target()` is called, I increment a time variable by one, representing the number of times `max_acceleration` has been added to the target speed, and when this variable reaches the acceleration time calculated by the `get_phases_distances()` function, the system can switch to the constant speed phase.

---

## Adapting `get_speed_target()` for different speeds on each wheel

To enable the use of `get_speed_target()` for rotation or reverse movement, I've introduced two coefficients, one for each wheel. These coefficients are multiplied by the final speed target to yield a rotation speed for each wheel. 
Consequently, if the left coefficient is set to -1 and the right coefficient to 1, the robot will pivot in place.

---

## Calculating error

Calculating the error was quite tricky as we now have two errors: one for linear speed and the other for angular speed. Additionally, we have the integrals of these variables, which represent linear position and angular position.

To make it easier to understand, I decomposed the problem into three functions:

1. `get_theorical_speed_and_position()`
2. `get_real_speed_and_position()`

These two functions determine five parameters:

### Linear velocity of the robot:
$$
V = leftSpeed + rightSpeed
$$

### Angular velocity:
$$
W = rightSpeedTarget - leftSpeedTarget
$$

### Angle travelled (integral of the angular velocity):
$$
Z = previousZ + W
$$

### Distance travelled on x axis (integral of x component of linear speed):
$$
X = previousX + V \cdot \cos(Z)
$$

### Distance travelled on y axis:
$$
Y = previousY + V \cdot \sin(Z)
$$

The difference between the two functions is that the first one processes these operations with **theoretical values**, which are determined relative to the speed profile, while the second one processes them with **real speeds** determined using the encoders.

---

### `get_errors()` function

This function calls the two functions above to process the error calculation (difference between theoretical speed/position and real speed/position):

$$
errorDistance = \sqrt{(X_{theorical} - X_{real})^2 + (Y_{theorical} - Y_{real})^2}
$$

Euclidean norm to determine the distance between the points  
\((X_{theorical}, Y_{theorical})\) and \((X_{real}, Y_{real})\).

$$
errorAngle = \text{atan2}(Y_{theorical} - Y_{real}, X_{theorical} - X_{real}) - Z_{real}
$$

Trigonometric relation where `atan2` is a mathematical function that computes the arctangent of the first parameter divided by the second one.

These two variables are used to calculate the errors used in our PID correction:

$$
errorXY = errorDistance \cdot \cos(errorAngle)
$$

$$
errorV = V_{theorical} - V_{real}
$$

$$
errorZ = Z_{theorical} - Z_{real}
$$

$$
errorW = W_{theorical} - W_{real}
$$

---

## Implementing the PID correction and sending commands to motors

We can now create a function that applies **Proportional** and **Integral** (and eventually **Derivative**) coefficients to our errors to send the resulting command to the motors.  
We need two different PIDs:
- One for linear speed (`errorV`)
- One for angular speed (`errorW`)

$$
pid_v = P_V \cdot errorV + I_V \cdot errorXY
$$

$$
pid_w = P_W \cdot errorW + I_W \cdot errorZ
$$

The resulting motor commands are then:

$$
LeftCommand = pid_v - pid_w
$$

$$
RightCommand = pid_v + pid_w
$$

---

## Next session tasks

- During the next session, Loïc will assemble the final version of the rolling base.  
  After that, I will have to recalibrate the robot and find the correct PID coefficients to verify everything is working correctly.
- Work on a way to send the data processed by our cameras to the Jetson.
