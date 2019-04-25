# UDACITY_SELF_DRIVING_CAR_ENGINEER_NANODEGREE_project8.PID-control

# Introduction

## 1. Purpose

Purpose of this project is to drive in simulation

Simulation map is same as Behavior cloning project

But it has a big difference in the approach

Behavior cloning used deep learning to teach simulator to drive

To do that, I need to drive myself to collect steering angle data

But PID control need not any data

It just controls steering angle or speed by calculation

It is affected by only CTE (Cross Track Error) and several coefficient

So only important thing is tuning coefficients and decide how to use this result


## 2. Rubric points

1. Car should perfectly drive at least 1 cycle


## 3. Input data given by simulator / Output data for simulator

1. Input

For every "telemetry"
- CTE
- speed
- steering angle

2. Ouput
For every "telemetry", I can send
- steering angle
- throttle


## 4. My intention of algorithm

Clearly, I need to calculate steering angle by using Kp, Ki, Kd

But firstly, I concerned about controling speed or not

Controlling speed means calculate speed by using Kp, Ki, Kd every time when value is updated

I thought about the method real human drive

When I drive and have to curve, I speed up at straight lane,

and shortly before curve, slow down to pass a curve safely

And I concluded it is difficult to act like this only using CTE information

To do that, I need map data, and need to know where the curve is, so where do I have slow down

So, I excluded speed in my have to control list

The rest of duty is only parameter tuning

And I tried not using twiddle but just try case by case



# Background Learning


### 1. PID control

To control steering angle, we can calculate its angle by

alpha = -Kp * CTE -Ki * delta(CTE) - Kd * sum(CTE)

It is reason why being called PID control

There are 3 coefficient Kp, Ki, Kd

1) P means "Propotional"

It affect to steering to turn as much as you are apart from target position

2) I means "Integral"

It is important when my car allignment is not good so it cannot go straight even though steering is straight

3) D means "Differential"

It can prevent oscillation of car

When using only P term, the car will oscillate inevitably


<img src="./images/pid_control.jpg" width="400">



# Content Of This Repo
- ```src``` a directory with the project code
	- ```main.cpp``` : communicate with simulator, reads in data, calls a function in PID.h to drive
	- ```PID.h``` : header file of PID
	- ```PID.cpp``` : have functions have to be used in main.cpp



# Flow

## 1. Flow Chart

Before makeing code, I made flow chart to check total flow and check what function will be need

##### main.cpp
<img src="./images/flow_chart_main.png" width="800">


# Results

It drove well without leaving the lane


# Conclusion & Discussion

### 1. About parameter tuning

Firstly, I concluded not to use Ki

Because it is only useful when allignment of car is not good

But it is simulation, so allignment will be perfect, so I set Ki = 0.0

Secondl, I set only Kp = 0.1

And anticipated it will oscillate but I just wonder it can react at sharp curve

But without Kd, even a little curve caused oscillation and makes it larger and larger

So it cannot go farther

Thirdly, I added Kd = 0.1 (Kp = 0.1)

Relativly, I drives a little better than before one

But it is insufficient to resist to oscillation

Fourthly, I raised the Kd = 0.5 (Kp = 0.1)

Expectedly, I drives one lap successfully

Comparing to before one, it can endure better at oscillation

So I concluded Kp means the ability to react at curve

and Kd means ability of stabilization

By that conclusion, I think it will better to raise Kp more,

and to counter balance that effect, also need to raise Kd

And Kp is muliplied to CTE, but Kd is multiplied to delta(CTE)

So Kd should much larger than Kp to affect equally to steering angle

Finally I selected Kp = 0.13, Kd = 0.8, Ki = 0.0



