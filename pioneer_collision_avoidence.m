%% ENPM808F HW2
% Wenqi Han
% Built the model after learning V-rep from https://www.youtube.com/watch?v=mal48Vd-DQY&t=89s
% This pioneer has:
% 4 input Pioneer_p3dx_ultrasonicSensor that read in distance in the front
% (2 at each side).
% 2 Vision_Sensor. One senses the environment in the front, and returns 
% back images. The other one sense light on the floor in front of the
% robot. In this program the robot will always turn right quickly to avoid
% the light.
% The robot has 4 options for the user to control the robot
% w - forwad, a - turn left, d - turn right, s - stop



clc
clear all

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19998,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server')
    disp('Choose what you want to do: a - turn right; d - turn left; s - stop; w - forward faster');
    
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [returnCode,rfront_Sensor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [returnCode,right_Sensor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking);
    [returnCode,left_Sensor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
    [returnCode,lfront_Sensor]=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_blocking);
    [returnCode,myCam]=vrep.simxGetObjectHandle(clientID, 'Vision_sensor',vrep.simx_opmode_blocking);
    [returnCode,lightCam]=vrep.simxGetObjectHandle(clientID, 'Vision_sensor0',vrep.simx_opmode_blocking);
    
    % code here;
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.9,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.9,vrep.simx_opmode_blocking);
    
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,lfront_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,rfront_Sensor,vrep.simx_opmode_streaming);
    [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_streaming);
    [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,lightCam,1,vrep.simx_opmode_streaming);
    
    
    run = true;
    while run
        [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
        imshow(image);
        [returnCode, resolution, lightimage]=vrep.simxGetVisionSensorImage2( clientID,lightCam,1,vrep.simx_opmode_buffer);
        I = im2double(lightimage);
        
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,rfront_Sensor,vrep.simx_opmode_buffer);
        rfdetectedPoint = detectedPoint;
        fprintf('You are %d m away from the wall!\n',norm(rfdetectedPoint));
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,lfront_Sensor,vrep.simx_opmode_buffer);
        lfdetectedPoint = detectedPoint;
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_buffer);
        rdetectedPoint = detectedPoint;
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor,vrep.simx_opmode_buffer);
        ldetectedPoint = detectedPoint;
        
        %         fprintf('You are %d m away from the wall!\n',norm(fdetectedPoint));
        %         fprintf('You are %d m away from the left!\n',norm(ldetectedPoint));
        %
        if  norm(lfdetectedPoint)<0.2 || norm(rfdetectedPoint)<0.2
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.5,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
            disp('Obstacle Avoidance - Turn around please wait');
            pause(5);
            
            [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
            imshow(image);
        
        elseif (norm(lfdetectedPoint)<0.3 || norm(rfdetectedPoint)<0.33) ||(norm(ldetectedPoint)<0.75 || norm(rdetectedPoint)<0.7 )
            
            if norm(rdetectedPoint)>=norm(ldetectedPoint)
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                disp('Turn right!');
                pause(1.5);
                [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
                imshow(image);
            elseif norm(rfdetectedPoint)>=norm(lfdetectedPoint)
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
                disp('Turn right!');
                [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
                imshow(image);
            else
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.3,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
                pause(1);
                disp('Turn left!');
                [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
                imshow(image);
                pause(1);
            end
        elseif norm(lfdetectedPoint)<0.5 || norm(rfdetectedPoint)<0.5
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.4,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.4,vrep.simx_opmode_blocking);
            disp('Obstacle Avoidance - driving slowly please wait')
            [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
            imshow(image);    
            
        else
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.9,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.9,vrep.simx_opmode_blocking);
            
            
        end
        
        
        if sum(I(1,:)) > 27
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.7,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.4,vrep.simx_opmode_blocking);
            disp('Turn right to avoide light!');
            pause(1);
        end
        
        
      
        % Please uncomment these code to enable the user interact function
        
        %         if norm(rfdetectedPoint)>50 && norm(lfdetectedPoint)>50
        %             disp('If the robot is trapped in some please, please give any direction');
        %             prompt = 'Choose what you want to do or it will keep moving in the same direction!';
        %             info = input(prompt,'s');
        %
        %
        %             if strcmp(info,'d')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
        %                 imshow(image);
        %                 pause(3);
        %             elseif strcmp(info,'a')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.3,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
        %                 imshow(image);
        %                 pause(3);
        %             elseif strcmp(info,'w')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.8,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.8,vrep.simx_opmode_blocking);
        %                 [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
        %                 imshow(image);
        %             elseif strcmp(info,'s')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2( clientID,myCam,1,vrep.simx_opmode_buffer);
        %                 imshow(image);
        %                 pause(10);
        %             elseif strcmp(info,'0')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
        %                 vrep.simxFinish(-1);
        %                 run = false;
        %
        %             end
        %         elseif norm(detectedPoint)<1.5
        %
        %
        %             prompt = 'You are about to crash!';
        %             info = input(prompt,'s');
        %
        %
        %             if strcmp(info,'a')==1
        %                 disp('You are turing right');
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.3,vrep.simx_opmode_blocking);
        %                 disp('Your left wheel speed is 0.3 m/s');
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
        %
        %             elseif strcmp(info,'d')==1
        %                 disp('You are turing left');
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.3,vrep.simx_opmode_blocking);
        %                 disp('Your right wheel speed is 0.3 m/s');
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
        %
        %             elseif strcmp(info,'w')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0.6,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0.6,vrep.simx_opmode_blocking);
        %                 disp('You are going forward at speed 0.6 m/s');
        %             elseif strcmp(info,'s')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
        %                 disp('You stopped the robot');
        %             elseif strcmp(info,'z')==1
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
        %                 [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
        %                 vrep.simxFinish(-1);
        %                 run = false;
        %             end
        %
    end
end



vrep.delete(); % call the destructor!
