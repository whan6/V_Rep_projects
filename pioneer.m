%% ENPM808F HW2
% Wenqi Han
% Built the model after learning V-rep from https://www.youtube.com/watch?v=mal48Vd-DQY&t=89s
% This pioneer has: 
% 1 input Pioneer_p3dx_ultrasonicSensor that read in distance in the front;
% Left and right wheel that send velocity to MATLAB;
% The robot has 4 options for the user to control the robot - 
% w - forwad, a - turn left, d - turn right, s - stop;  
% When asked for direction, it is assumed the user will alway give a
% response sometime. 
% If the user didn't give response in time and the robot gets too close to 
% the wall, the program will tell the user to give either turn left or turn
% right order to save the robot. Going forwars order will result in an 
% error message.
%%%


clc
clear all

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server')
    disp('Choose what you want to do: a - turn right; d - turn left; s - stop; w - forward faster');
    
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [returnCode,lfront_Sensor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_blocking);
   
    
    % code here;
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.3,vrep.simx_opmode_blocking);
    
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,lfront_Sensor,vrep.simx_opmode_streaming);
    
    run = true;
    while run
        
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,lfront_Sensor,vrep.simx_opmode_buffer);
        lfdetectedPoint = detectedPoint;
        fprintf('You are %d m away from the wall!\n',norm(lfdetectedPoint));

           
        if  norm(lfdetectedPoint)>25
            disp('If the robot is trapped in some please, please give turn left or right direction order');
            prompt = 'Choose what you want to do or it will keep moving in the same direction!';
            info = input(prompt,'s');
            
            
            if strcmp(info,'d')==1
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                
                pause(3);
            elseif strcmp(info,'a')==1
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.3,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
                
                pause(3);
            elseif strcmp(info,'w')==1
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.8,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.8,vrep.simx_opmode_blocking);
                
            elseif strcmp(info,'s')==1
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,-0.2,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,-0.2,vrep.simx_opmode_blocking);
                
                pause(10);
            elseif strcmp(info,'0')==1
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                vrep.simxFinish(-1);
                run = false;
                
            end
                    elseif norm(detectedPoint)<1.5
            
            
                        prompt = 'You are about to crash!';
                        info = input(prompt,'s');
            
            
                        if strcmp(info,'d')==1
                            disp('You are turing right');
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
                            disp('Your left wheel speed is 0.3 m/s');
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                            pause(5);
                        elseif strcmp(info,'a')==1
                            disp('You are turing left');
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.3,vrep.simx_opmode_blocking);
                            disp('Your right wheel speed is 0.3 m/s');
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
                            pause(5);
                        elseif strcmp(info,'w')==1
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.6,vrep.simx_opmode_blocking);
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.6,vrep.simx_opmode_blocking);
                            disp('Opps, will not solve the problem');
                        elseif strcmp(info,'s')==1
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                            disp('You stopped the robot');
                        elseif strcmp(info,'z')==1
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
                            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                            vrep.simxFinish(-1);
                            run = false;
                        end
            
        end
    end
end


vrep.delete(); % call the destructor!
