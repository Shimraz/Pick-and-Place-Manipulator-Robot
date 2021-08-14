# Lego EV3-Pick-and-Place-Manipulator-Robot
In this project, you will remotely access the manipulator in Figure 1 and program it in MATLAB to achieve a pick-and-place sequence of motions. The entire arm is driven by three motors (See Figure 3): • Motor A • Motor B • Motor C In addition to the three actuators, the system has five sensors: • Touch sensor #1 (See Figure 2) • Touch sensor #2 (See Figure 2) • Motor A encoder • Motor B encoder • Motor C encoder
![image](https://user-images.githubusercontent.com/43060427/129429537-ef0bb35d-3282-41e7-8642-995dbbf5406a.png)
![image](https://user-images.githubusercontent.com/43060427/129429548-386d4558-e036-431f-869f-32726d3978f4.png)
The manipulator is made of four links (See Figure 3):
• Link-1 of
• Link-2 of
• Link-3 of
• Link-4 of
Among kinematic constraints include:
• Link-1 is rigidly connected to link-2
• Link-4 is always perpendicular to the ground plane
• Rotation of link-3 is bounded by link-2 on the lower side and touch sensor #2 on the upper side
• The base rotation is bounded by the touch sensor #1 on one side
![image](https://user-images.githubusercontent.com/43060427/129429554-83b181fe-2cf3-4796-b535-53007acf25ef.png)
Last but not least, all motors are coupled through gearboxes. Of interest are gearboxes for motor A and motor B. The gear details are presented in Figure 4.
Figure
![image](https://user-images.githubusercontent.com/43060427/129429560-0341fb0e-d5f0-4165-bcd5-0c7788e09bdc.png)
The robot workplace is made of thee stations (See Figure 5):
• Station A (elevated 70mm as indicated in Figure 6)
• Station B
• Station C
![image](https://user-images.githubusercontent.com/43060427/129429567-8c3ca29d-cfea-4fbf-9816-7d384e03e173.png)
You will accomplish the following tasks:
1. Develop the inverse kinematic model (equations) for this manipulator robot
2. Develop behaviours for picking, placing and homing
3. Use the behaviours in (2) to command the manipulator to pick the ball from station C and place it at station A
4. Use the behaviours in (2) to command the manipulator to pick the ball from station A and place it at station B
5. Use the behaviours in (2) to command the manipulator to pick the ball from station B and place it at station C
![image](https://user-images.githubusercontent.com/43060427/129429584-96cf126f-3378-4ed9-b587-0d0062fdaa9a.png)
