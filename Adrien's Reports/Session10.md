# Session report 10: February 22, 2024

**Session objective:** Take our PID to the next level by avoiding common imperfections and delve into using our cameras for shape recognition.

The PID algorithm created during the previous sessions was functional, but occasionally exhibited erratic behavior. To enhance it, I sought assistance from the excellent article 'Améliorer vos régulateurs PID' by the PM-Robotic team and the fantastic series of tutorials 'PID control' from the Matlab YouTube channel.

## Establish a proportional relationship between sample time and P, I, D coefficients.

"First of all, the initial issue was that I had established no link in my code between my P, I, D coefficients, and the sample time. So, even when I changed the sample time, I was still integrating and differentiating the error over the same time period. To avoid the need to adapt I and D correction coefficients after each modification of the sample time, I multiply my integral coefficient I at the beginning of the program by the sample time because we integrate error*dt. Similarly, I divide my derivative coefficient D by the sample time because we want to differentiate ${derror \over dt}$, where dt is the short period of time over which we are performing the operation.

## Avoid derivative kick

As our error is calculated as follows: Error = DistanceSetpoint - DistanceTravelled, if we change DistanceSetpoint, the error will change so rapidly that its derivative will tend toward infinity. This will create an undesired spike in the calculated output when applying the PID correction.

<img src="Report's images\Session10\Derivative_Kick_from_PMRobotixTeam.png" width="250">

Source : PM Robotix team

One solution is called 'Derivative on Measurement'; I explain it in the context of my algorithm:

Error = DistanceSetpoint - DistanceTravelled

So its derivative is equal to:  

$$
\frac{dError}{dt} = \frac{dDistanceSetpoint}{dt} - \frac{dDistanceTravelled}{dt}
$$

However, DistanceSetpoint is the distance instruction, so it remains constant until a new instruction is sent.  
Consequently, its derivative is equal to zero. This leads to the following result:  

$$
\frac{dError}{dt} = - \frac{dDistanceTravelled}{dt}
$$

Now that our derivative is no longer directly dependent on the Setpoint, we avoid high-speed changes and prevent spikes in output when the setpoint changes.

## Changing PID coefficients in real time
To determine P, I, D coefficients, it can be really helpful to be able to modify our coefficients in real-time to visualize the evolution and influence of our modifications on the error. It's a great time-saver because it avoids restarting the program for each modification.

However, there is a significant problem to be aware of. Instantly modifying the integral coefficient can create a bump in the output. When we multiply the I coefficient by the integration of the error, I coefficient is assumed to be constant. But this assumption is no longer valid if we modify it while the program is running. To solve this, we can simply include the I coefficient inside the integral. Now, I sum I×errorI×error to the integral term each time the output is corrected, rather than adding the error to the sum and multiplying it by I later.

## Reset Windup
This is one of the most frequent problems in PID implementation. Our system is not perfect because our motors are speed-limited. If we attempt to exceed this limit, our motors will saturate at their maximum speed, causing our system to become nonlinear. However, the main issue is that our integral term will continue to grow, assuming the motor speed is still increasing to correct the error faster. Finally, when our value drops below the saturation point, the integral term will be excessively high and unrelated to the actual error. As a result, the robot will deviate significantly from its setpoint.

This problem is well-explained further in Matlab videos: Understanding PID Control Part 2: Expanding Beyond a Simple Integral."

To address this issue, the integral term is now increased only when there is no saturation, indicating that our motors are not operating at their maximum speed. In cases of saturation, correction is applied using only a proportional coefficient.


## Set up our JeVois Pro camera for shapes recognition

The main reason we chose these specific cameras is their integration of powerful chips, enabling us to process calculations directly from the camera and then transmit the results to our Nvidia Jetson Nano. This approach allows us to leverage the computational power on the Jetson Nano for real-time strategy elaboration with artificial intelligence.

Getting acquainted with the camera was straightforward, as many programs come preinstalled on the camera chip with the Ubuntu operating system. After adjusting the focal length using the small wheel around the lens, I tested programs to recognize objects and Aruco markers.

<img src="Report's images\Session10\objects_recognition_JeVoisProCamera.jpg" width="250">

The provided software is highly comprehensive, allowing us to adjust camera settings and customize preinstalled programs based on our specific conditions, including parameters such as the size of our Aruco markers.

I achieved excellent results directly, as showcased in the following videos:

Plants detection: https://youtube.com/shorts/sOcyxpt3ze8
Aruco detection: https://youtube.com/shorts/fuHYHDNQBM0

This will significantly save us time, as we can use this algorithm to extract data from the cameras and transmit them to the Jetson, with just a need for some adaptations to fit our specific use.

## **Next session tasks:**
In the upcoming session, the PID correction will be fully implemented and configured, along with our odometry. I will focus on developing obstacle avoidance paths and establishing a method to transmit the output from our cameras to our main Nvidia Jetson Nano. This data will be utilized to inform decision-making strategies with artificial intelligence.

## Sources :
https://www.pm-robotix.eu/2022/02/02/asservissement-et-pilotage-de-robot-autonome/

https://fr.mathworks.com/videos/understanding-pid-control-part-1-what-is-pid-control--1527089264373.html
