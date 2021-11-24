# Team 716 Basic Bot generic code
### Code that can drive, move forward in autonomous, and has mappings for speed controllers, relays, and Pneumatics


## Basic Information
This software assumes the following:
- Controls consist of 2 joystics and a USB xbox controller
- Drivetrain is 2 or 4 motor tank drive *with encoders at the gearbox*
- Mappings are as outlined in Mappings

## Controls 
### Driver
Drivetrain is tankdrive where each stick corresponds to throttle on its respective side of the robot.

**Straight drive:** 
When the left stick trigger is held, the robot will use its encoders to drive in a straight line forward or backward depending only on throttle input from the left stick y axis. 

**Hold the Line: (Untested)**
When the right stick trigger is held, the robot will use its encoders to maintain its current position regardless of stick inputs.

Driver also has control over relay 3 by pressing the right stick's top button.

### Operator
**Analog Sticks:**
The sticks and triggers correspond to the optional end effector speed controllers and are analog inputs. Pushing up on the sticks or in on the right trigger will run the speed controller forward with speed dependant on how far the input is pushed. Pushing down on the sticks or in on the left trigger will result in the same behavior in the reverse. The sticks have a 5% deadzone to prevent unintentional activation. 

**Buttons & Bumpers** are hold - based. Pressing them will activate their corresponding device and releasing them will return it to its neutral state.

**Control Locking:** By holding the d-pad, tapping a button, then releasing both, the operator can lock a device in its activated state. The device can be returned to its deactivated state by pressing and releasing its corresponding button again or with the abort button. 
![Control Mappings](https://raw.githubusercontent.com/716robotics/BaseBot/main/Operator_Control_Mapping.png)
## RoboRio Mappings (bold is required)
### PWM
Port | Description
--- | ---
0 | **Left Drivetrain 1**
1 | Left Drivetrain 2
2 | **Right Drivetrain 1**
3 | Right Drivetrain 2
4 | Aux. Speed Controller 1
5 | Aux. Speed Controller 2
6 | Aux. Speed Controller 3
7 | Aux. Relay 1
8 | Aux. Relay 2
9 | Aux. Relay 3
### DIO
Port | Description
--- | ---
0 | **Left Drivetrain A**
1 | **Left Drivetrain B**
2 | **Right Drivetrain A**
3 | **Right Drivetrain B**
4 | Unassigned
5 | Unassigned
6 | Unassigned
7 | Unassigned
8 | Unassigned
9 | Unassigned

Note: encoders are required for autonomous, straight drive, and Hold the Line to work properly but the robot can be run without them.
### PCM
Port | Description
--- | ---
Compressor | **Compressor**
0 | Pneumatic 1 A
1 | Pneumatic 1 B
2 | Pneumatic 2 A
3 | Pneumatic 2 B
4 | Pneumatic 3 A
5 | Pneumatic 3 B
6 | Pneumatic 4 A
7 | Pneumatic 4 B

note: Compressor is **only** required if any other pneumatics are in use