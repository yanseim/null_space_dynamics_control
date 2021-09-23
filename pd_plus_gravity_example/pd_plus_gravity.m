% 20210622
% according file is pdplusgravity.ttt(note that the gripper in this scene is not dynamic.)
clear
close all
clc

%% Parameters
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'column';
lbr.Gravity = [0 0 -9.81];
forceLimit = 5000;
displayOn=false;
jointNum = 7;

Q=[]; QDOT=[]; U=[];XD = [];X = [];T = [];logeePos = [];log_t = [];R = [];

%% pd controller parameters
Kp = 40*diag([0.2,1,1,1,0.5,0.5,0.1]);
Kv = 1.4*diag([3.5,4,2.5,3.5,1.5,1,1]);

%% Connect to the Vrep
% load api library
addpath('../libs');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
% close all the potential link
vrep.simxFinish(-1);   
% wait for connecting vrep, detect every 0.2s
while true
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    if clientID>-1 
        break;
    else
        pause(0.2);
        disp('please run the simulation on vrep...')
    end
end
disp('Connection success!')
% set the simulation time step
tstep = 0.005;  % 5ms per simulation step
vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot);
% open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID,true);

%% Simulation Initialization
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);

% get the joint handles
jointName = 'iiwa_joint_';
jointHandle=zeros(jointNum,1); % column vector
for i=1:jointNum 
    [res,returnHandle]=vrep.simxGetObjectHandle(clientID,[jointName,int2str(i)],vrep.simx_opmode_blocking);
    if res==0
        jointHandle(i)=returnHandle;
    else
        fprintf('can not get the handle of joint %d!!!\n',i);
    end
end

% get the ee handle
[res,eeHandle] = vrep.simxGetObjectHandle(clientID,'iiwa_link_ee_kuka_visual',vrep.simx_opmode_blocking);
if res==0
    jointHandle(i)=returnHandle;
else
    fprintf('can not get the handle of ee %d!!!\n',i);
end

vrep.simxSynchronousTrigger(clientID);
disp('Handles available!')

% first call to read the joints' configuration, the mode has to be
% simx_opmode_streaming, afterwards simx_opmode_buffer
jointConfig=zeros(jointNum,1); jointVeloc = zeros(jointNum,1); jointTorque = zeros(jointNum,1); 
for i=1:jointNum
    [~,jointConfig(i)]=vrep.simxGetJointPosition(clientID,jointHandle(i),vrep.simx_opmode_streaming);
    [~,jointVeloc(i)]=vrep.simxGetObjectFloatParameter(clientID,jointHandle(i),2012,vrep.simx_opmode_streaming);% joint velocity 
    [~,jointTorque(i)]=vrep.simxGetJointForce(clientID,jointHandle(i),vrep.simx_opmode_streaming);
end

% end effector position
[~,eePos]=vrep.simxGetObjectPosition(clientID,eeHandle,-1,vrep.simx_opmode_streaming);
eePos = eePos';

% joints position. for debug use
jointPos = zeros(3,7); 
for i=1:jointNum 
    [~,rowVec]=vrep.simxGetObjectPosition(clientID,jointHandle(i),-1,vrep.simx_opmode_streaming);
    jointPos(:,i) = rowVec.';
end
    
% get simulation time
currCmdTime=vrep.simxGetLastCmdTime(clientID);
lastCmdTime=currCmdTime;
vrep.simxSynchronousTrigger(clientID);         % every time we call this function, verp is triggered

%% Simulation Start
disp('being in loop!');
t = vrep.simxGetLastCmdTime(clientID)/1000;
tInit = t
while (vrep.simxGetConnectionId(clientID) ~= -1)  % vrep connection is still active
    
    % 0. time update
    currCmdTime=vrep.simxGetLastCmdTime(clientID);
    dt=(currCmdTime-lastCmdTime)/1000;              % simulation step, unit: s 
    
    % 1. read the joints' configuration (position and velocity)
    for i=1:jointNum
        [~,jang]=vrep.simxGetJointPosition(clientID,jointHandle(i),vrep.simx_opmode_buffer);
        [~,jvel] = vrep.simxGetObjectFloatParameter(clientID,jointHandle(i),2012,vrep.simx_opmode_buffer);
        jointConfig(i)=jang;
        jointVeloc(i)=jvel;
    end

    % set desired q(only once)
    if ~exist('dq','var')      
        dq = jointConfig;
        dq(2) = dq(2)+0.2;
        dq(3) = dq(3)+0.2;
        dq(4) = dq(4)+0.2;
    end
    
    if ~exist('jointConfigLast','var')
        jointConfigLast = jointConfig;
    end
    
    if ~exist('jointVelocLast','var')
        jointVelocLast = jointVeloc;
    end
    
    q=jointConfig;
    qdot=jointVeloc;    
    qdotdot = (qdot-jointVelocLast)./dt;% column vector

    % 2. display the acquisition data, or store them if plotting is needed.
    if displayOn==true
        disp('joints config q :');       disp(q'.*180/pi);
        disp('joints config qdot :');      disp(qdot'.*180/pi);
    end

   % 3. calculate tau
    tau_g = gravityTorque(lbr,q);
    
    tau = -Kp*(q-dq)-Kv*qdot+tau_g;    
    tau(7)=0;% the tau(7) is set to 0. since its mass is too small for torque control. otherwise it will induce instability
    
    % 4. set the torque in vrep way
        for i=1:jointNum
                if tau(i)<-forceLimit
                    setForce=-forceLimit;
                elseif tau(i)>forceLimit
                    setForce=+forceLimit;
                else
                    setForce=tau(i); % set a trememdous large velocity for the screwy operation of the vrep torque control implementaion
                end
                vrep.simxSetJointTargetVelocity(clientID, jointHandle(i), sign(setForce)*1e10, vrep.simx_opmode_oneshot);% 决定了力的方向
                tau(i)=setForce;
                if setForce<0
                    setForce = -setForce;
                end
                vrep.simxSetJointForce(clientID, jointHandle(i),setForce , vrep.simx_opmode_oneshot);           
        end

    % data recording for plotting
    U = [U tau];
    Q=[Q q];  
    QDOT=[QDOT qdot]; 

    % 4. update vrep(the server side) matlab is client

    lastCmdTime=currCmdTime;
    jointConfigLast=q;    
    jointVelocLast = qdot;

    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    t=t+dt; % updata simulation time
    T = [T t];
    disptime = sprintf('the time is %f s', t);
    disp(disptime);
end
vrep.simxFinish(-1);  % close the link
vrep.delete();        % destroy the 'vrep' class

%%
figure(1)
for j=1:jointNum
    subplot(4,2,j);
    plot(T,Q(j,:).*180/pi,'b'); hold on;
    scatter(T(end),dq(j).*180/pi);
    titletext = sprintf('joint %i angle',j);
    subtitle(titletext);
    if j == 6||j==7
        xlabel('time [s]');
    end
    if j==1||j==3||j==5||j==7
        ylabel('angle [deg]');
    end
    grid;
end