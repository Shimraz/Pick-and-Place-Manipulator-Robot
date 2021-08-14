% The purpose of the program is to control a pick and place robot
% All sub functions will be specified below in the code

%% Set Parameters
mylego = legoev3('usb')
% Declaration of COntroller Parameters
global PBase
global PERIOD
PERIOD = 0.1;  

% Gear Ratio Parameters
global grBase
global grLink
grBase = 3;   % Due to gear ratio
grLink = 5;

% Global Variables for Kinematics
global l1
global l2
global l3
global l4
l1=50; l2=95; l3=185; l4=110;

% Declaration of Motors
global gripper_motor
global link_motor
global base_motor
gripper_motor = motor(mylego, 'A');
link_motor = motor(mylego, 'B');
base_motor = motor(mylego, 'C');

% We will set the status of the gripper motor, 0 is stop (no power)
% And 1 means power is active
global gripper_motorStatus;
gripper_motorStatus = 0;

% Set default velocites to 30
global stallSpeed
motor_speed = 30;
stallSpeed = 0;
speedBase = 0;
stop(base_motor)  %safety measures
stop(link_motor)  %safety measures
gripper_motor.Speed = stallSpeed;
link_motor.Speed = -motor_speed;
base_motor.Speed = motor_speed;

% Declaration of sensors
global ts_down
global ts_up
ts_down = touchSensor(mylego,1);
ts_up = touchSensor(mylego,3);

%% Main
% For the base motor angular rotation should 
% be negative while going from A to C
% For the link motor angular rotation should 
% be positive to move down the Z axis

flag0 = 0;
while(flag0 == 0)
    home(); % Homing the Robot
    pause(.5);
    
    % Go to C and pick

    pick('C'); % Picking the ball
     
    % Go to A and place

    place('A'); % Placing the ball
    
    % Pick up from A
    pick('A');
    
    % Place in B

    place('B');
    
    % Pick up from B
    pick('B');
    
    % Place in C
    place('C');
    
    pause(0.5)
     
    % go to any other given position aside from A, B or C
    %pick('R');
    %place('R');
    
    % Go home
    home(); 
    
    flag0 = 1;

end


%% Link Control %%

% Function for the link motor should accept three possible values: 0,1,2
% 0 is to go down to grab the ball
% 1 is to stay in the middle position to grab the ball from table
% 2 is to go up till reaching sensor
% The algorithm does not validate the location if we set 0 in position A
% The robot will crash with the table

function link_motor_mov(theta2)
    mov_flagLink = 0;
    setLink = 0;
    PLink = 0.1; 
    %KiLink = 0.1; % For Integral part of the controller
    KdLink = 0.01;
    % Parameter for tuning the mechanical offset for link motor
    link_tun = -5;
    %newValI = 0; % For Integral part of the controller
    diffAlt = 0;
    count = 1;
    global grLink
    global link_motor
    global PERIOD
    global ts_up
    global stallSpeed
    
    switch theta2
        case 2   % Move up
            setLink = 0;
            % We move to the upper part at a constant speed till
            % touching the sensor
            while (readTouch(ts_up) == 0)
              link_motor.Speed = -30;  
            end
            link_motor.Speed = stallSpeed;
            resetRotation(link_motor);
            return;
        otherwise
            setLink = (theta2+link_tun)*grLink; % The desired value
    end
      
    while mov_flagLink == 0  
        % The current value from the encoder
        encoder_link = readRotation(link_motor);
    
        % Error
        diff = setLink - encoder_link;
    
        % Proportional part
        newValP = int8(diff*PLink); 
    
        % Differential part
        if (count == 1)
            diffAlt = diff;
        end

    newValD = int8((KdLink/PERIOD)*(diff-diffAlt));
    
    diffAlt = diff;
    
    %PD Controller
    newVal_l = newValP + newValD;
    
    % Save the data;
    % newVal_DB(count) = newVal;
    % Saturation part not to use speed with impossible system conditions
    if newVal_l >= -35 && newVal_l <= 0
        newVal_l = -35;
    elseif newVal_l > 0 && newVal_l < 20
        newVal_l = 25;
    elseif newVal_l >= 30
        newVal_l = 30;
    end
    
    link_motor.Speed= newVal_l;  
        if (diff >= -5 && diff <=5)
            mov_flagLink = 1;
            link_motor.Speed = stallSpeed;
        end
    count = count + 1;
    pause(PERIOD); 
    end
end

%% Base Control
% This functions helps to move the base motor to 3 positions
% 0 to be in A, 0 deg
% 1 to be in B, 90 deg
% 2 to be in C, 180 deg
function base_motor_mov(theta1)
   mov_flagBase = 0;
   setBase = 0;
   PBase = 0.15;
   % Parameter for tuning the mechanical offset for base motor
   base_tun = -12;
   global base_motor
   global grBase
   global PERIOD
   global stallSpeed
   global ts_down
   
    switch theta1
        case 0  % Move 0 degrees (left area)
            % Go to constant speed till you reach the sensor!
            setBase = 0;
            while (readTouch(ts_down) == 0 )
                base_motor.Speed = 30;
            end
            base_motor.Speed = stallSpeed;
            resetRotation(base_motor);
            return;
     
        otherwise
            setBase = (theta1+base_tun)*grBase; % Desired value
    end
    
    while (mov_flagBase == 0)
    % Reading the current value from the encoder 
    encoder_base = readRotation(base_motor);

    % Error
    diff = setBase - encoder_base;
    
    % Proportional component
    newVal = int8(diff*PBase);
    
   % Saturation points not to let the motor reach impossible conditions
    if newVal >= -20 && newVal < 0
        newVal = -18;
    elseif newVal >= 0 && newVal <= 20
        newVal = 18;
    end;
    base_motor.Speed= newVal; 
        if (diff >= -5 && diff <= 5) % If the error is 0
            mov_flagBase = 1;
            base_motor.Speed =stallSpeed;
        end
    pause(PERIOD); 
    end
end

%% Gripper Control
%This function is to control the gripper with two states
% 1 for close
% 0 for open

function gripper_motor_mov (mov)
 global gripper_motor
 global stallSpeed
 global gripper_motorStatus
 
    if gripper_motorStatus == 0 && mov == 1
        gripper_motor.Speed = 60;
        pause(0.1)
        gripper_motor.Speed = stallSpeed;
        gripper_motorStatus = 1;
    elseif gripper_motorStatus == 1 && mov == 0
        gripper_motor.Speed = -60;
        pause(0.1)
        gripper_motor.Speed = stallSpeed;
        gripper_motorStatus = 0;
    end
    
end
%% Inverse kinematics theta_1
function Theta_1 = inv_kin_theta1(X, Y , Z)

global  l1 l2 l3 l4;

if(X<=0)
    Theta_1 = atand(Y/X);
else
    Theta_1 = atand(Y/X)-180;
end

end

%% Inverse Kinematic theta_2
function Theta_2 = inv_kin_theta2(X, Y , Z)

global  l1 l2 l3 l4;

Theta_2 = 45 - asind((Z-l1-l2*sind(45)+l4)/l3);

end

%% Picking function
function pick(pos)
global  l1 l2 l3 l4;
    switch pos
        case 'A'
            A = [-(l3 - l2*cosd(45)) 0 0]
            theta_1 = inv_kin_theta1(A(1),A(2),A(3));
            theta_2 = inv_kin_theta2(A(1),A(2),A(3));
        case 'B'
            B = [0 -(l3 - l2*cosd(45)) -70]
            theta_1 = inv_kin_theta1(B(1),B(2),B(3));
            theta_2 = inv_kin_theta2(B(1),B(2),B(3));
        case 'C'
            C = [(l3 - l2*cosd(45)) 0 -70]
            theta_1 = inv_kin_theta1(C(1),C(2),C(3));
            theta_2 = inv_kin_theta2(C(1),C(2),C(3));
        otherwise % any other position
            R = [(l3 - l2*cosd(45)) (l3 - l2*cosd(45)) 0] % the coordinates of a random point
            theta_1 = inv_kin_theta1(R(1),R(2),R(3));
            theta_2 = inv_kin_theta2(R(1),R(2),R(3));
    end
    base_motor_mov(theta_1);
    gripper_motor_mov(0);
    link_motor_mov(theta_2);
    gripper_motor_mov(1);
    link_motor_mov(2);

end


%% Placing function
function place(pos)
    global  l1 l2 l3 l4;
    switch pos
        case 'A'
            A = [-(l3 - l2*cosd(45)) 0 0];
            theta_1 = inv_kin_theta1(A(1),A(2),A(3));
            theta_2 = inv_kin_theta2(A(1),A(2),A(3));
        case 'B'
            B = [0 -(l3 - l2*cosd(45)) -70];
            theta_1 = inv_kin_theta1(B(1),B(2),B(3));
            theta_2 = inv_kin_theta2(B(1),B(2),B(3));
        case 'C'
            C = [(l3 - l2*cosd(45)) 0 -70];
            theta_1 = inv_kin_theta1(C(1),C(2),C(3));
            theta_2 = inv_kin_theta2(C(1),C(2),C(3));
        otherwise
            R = [(l3 - l2*cosd(45)) (l3 - l2*cosd(45)) 0]
            theta_1 = inv_kin_theta1(R(1),R(2),R(3));
            theta_2 = inv_kin_theta2(R(1),R(2),R(3));
    end
    base_motor_mov(theta_1);
    link_motor_mov(theta_2);
    gripper_motor_mov(0);
    pause(1);
    link_motor_mov(2);
    
end

%% Homing function
function home()
% Go to Home
% For the homming we are using the motor_speed
home = 0;
global ts_up
global ts_down
global base_motor
global link_motor
global gripper_motor
global gripper_motorStatus
stallSpeed = 0;

while (home == 0)
    % Evaluate the status of the sensors
    if (readTouch(ts_up) == 0 || readTouch(ts_down) == 0)    
        if readTouch(ts_up) == 0 % If the up sensor is off 
            link_motor.Speed = -40;
            start(link_motor);  % we start the motor with motor_speed
        % If the final position is reached then we set motor to stall state   
        elseif readTouch(ts_up) == 1   
            link_motor.Speed = stallSpeed;
        end
        % if the bottom sensor is off
        if readTouch(ts_down) == 0 && readTouch(ts_up) == 1  
           base_motor.Speed = 30;
           start(base_motor);  % here the speed is positive
        elseif readTouch(ts_down) == 1
           base_motor.Speed = stallSpeed;  % set to stall speed
        end
       
    else  % Now that we are at home, both sensors turn on
        home = 1;
        base_motor.Speed = stallSpeed;
        % Reset Encoders
        resetRotation(base_motor);
        resetRotation(link_motor);
       
       % Now we open/close the gripper
       stop(gripper_motor);
       gripper_motor.Speed = 60; %-60 if is close (in initial condition) +60 if is open (in initial condition)
       start(gripper_motor)
       pause (0.1);
       gripper_motor.Speed = stallSpeed;
       gripper_motorStatus = 1;  % 1 if was close, 0 if was open
   end
end

end