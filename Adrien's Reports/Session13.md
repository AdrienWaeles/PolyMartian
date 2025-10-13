# Session report 13: March 19, 2024

**Session objective:** Implement the equations from the previous session in an algorithmm to allow displacements following speed profiles.

I implemented the speed profile from the previous session divided in two main functions :

# Get_speed_target() 
This function takes in entry the remaining distance to travel, and the distance to travel during both constant speed and deceleration phases and returns speed_target, the speed we want the robot to reach.
It will be called each time before applying PID so depeding on the sample frequency.

An enum variable indicates the current speed phase which can be STATIONARY, ACCELERATION, CONSTANT_SPEED or DECELERATION.

Here is how it works : 
1) STATIONARY phase : 
When we receive a new command we switch from STATIONARY to ACCELERATION phase.
2) ACCELERATION phase : 
We simply add the maximum acceleration to the speed target. Execept when the remaining travel distance becomes inferior to the sum of deceleration and constant_speed phase distances where we will have to switch the enum to CONSTANT_SPEED phase.
3) CONSTANT SPEED phase : 
In this new phase, speed_target will remain constant to the maximal speed until the remaining distance turn below deceleration distance which implies to switch to DECELERATION PHRASE
4) DECELERATION phase :
We add the maximum deceleration (which is negative) to speed_target until reaching a value close to zero defined by a variable called precision. When the remaining distance is below the wanted precision, we can stop the robot with speed_target = 0 and switch to STATIONARY. Therefore we will avoid correcting error during a too long period of time to not loose too much time during the competition.

# Get_phases_distance()
This function is called one time after each new command and returns the distances to travel during the three trapezoidal phases depending on the following parameters : distance_setpoint, max_acceleration, max_deceleration and max_speed.

The main difficulty was due to roundings when proceeding to divisions, indeed due to our encoders which have a finite number of points, we have a limited precision so we need to round our numbers to integers. Therefore when the distance value was rounded up the robot was exceeding the target.

So a tip is to round all divisions down with the floor() function and then reduce numerical errors with the following procedure.

1) First, we calculate the acceleration_time, deceleration_time, acceleration_distance, deceleration_distance using the formulas determined in the previous report.
2) If acceleration_distance + deceleration_distance is inferior to distance_setpoint, then we have to recalculate the deceleration distance to fit with a triangle speed profile (see formula from previous report). Thus we have to calculate the maximal speed reached which is equal to the new deceleration_time multiplied by max_deceleration.
3) Then we deduce the constant speed phase duration which is : 
    - equal to zero, in a triangle speed profile
    - equal to the distance_setpoint minus acceleration and deceleration distance, in a trapezoidal speed profile
4) Last but not least, we reduce numerical errors by recalculating the deceleration_distance with our new informations and this time without division so we reduce rounding error : 

$$ 
deceleration_distance = distance_setpoint - max_speed*constantspeed_time - acceleration_distance 
$$

## Next session tasks
Finish the whole new closed loop control algorithm including speed profiles displacements to test it on our playing table.
