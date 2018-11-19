function [states, correctedDelAng, correctedDelVel]  = PredictStates( ...
    states, ... % previous state vector (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias)
    delAng, ... % IMU delta angle measurements, 3x1 (rad) 变化的角度观测值是用来计算四元数的中间值
    delVel, ... % IMU delta velocity measurements 3x1 (m/s) 变化的速度值是用来计算速度与位置的中间值
    dt, ... % accumulation time of the IMU measurement (sec) IMU测量的累积时间
    gravity, ... % acceleration due to gravity (m/s/s)
    latitude) % WGS-84 latitude (rad)

% define persistent variables for previous delta angle and velocity which
% are required for sculling and coning error corrections
persistent prevDelAng;%persistent 变量将命名的变量一直保存在内存中
if isempty(prevDelAng)
    prevDelAng = delAng;
end

persistent prevDelVel;
if isempty(prevDelVel)
    prevDelVel = delVel;
end

persistent Tbn_prev;
if isempty(Tbn_prev)
    Tbn_prev = Quat2Tbn(states(1:4));
end

% Remove sensor bias errors
delAng = delAng - states(11:13);
delVel = delVel - states(14:16);

% Correct delta velocity for rotation and skulling
% Derived from Eqn 25 of:
% "Computational Elements For Strapdown Systems"
% Savage, P.G.
% Strapdown Associates
% 2015, WBN-14010
% correctedDelVel= delVel + ...
%     0.5*cross(prevDelAng + delAng , prevDelVel + delVel) + 1/6*cross(prevDelAng + delAng , cross(prevDelAng + delAng , prevDelVel + delVel)) +  1/12*(cross(prevDelAng , delVel) + cross(prevDelVel , delAng));
correctedDelVel= delVel;%为什么讲了公式没有用上呢

% Calculate earth delta angle spin vector 计算地球的角度自旋矢量
delAngEarth_NED(1,1) = 0.000072921 * cos(latitude) * dt;
delAngEarth_NED(2,1) = 0.0;
delAngEarth_NED(3,1) = -0.000072921 * sin(latitude) * dt;

% Apply corrections for coning errors and earth spin rate
% Coning correction from :
% "A new strapdown attitude algorithm", 
% R. B. MILLER, 
% Journal of Guidance, Control, and Dynamics
% July, Vol. 6, No. 4, pp. 287-291, Eqn 11 
% correctedDelAng   = delAng - 1/12*cross(prevDelAng , delAng) - transpose(Tbn_prev)*delAngEarth_NED;
correctedDelAng   = delAng - transpose(Tbn_prev)*delAngEarth_NED;

% Save current measurements
prevDelAng = delAng;
prevDelVel = delVel;

% Convert the rotation vector to its equivalent quaternion
% 处理与观测模型的pdf中有讲这个，但是与其不同 RotToQuat
deltaQuat = RotToQuat(correctedDelAng);

% Update the quaternions by rotating from the previous attitude through
% the delta angle rotation quaternion 该试子与pdf同
states(1:4) = QuatMult(states(1:4),deltaQuat);

% Normalise the quaternions 归一化四元数
states(1:4) = NormQuat(states(1:4));

% Calculate the body to nav cosine matrix 再用新的估计出的四元素计算计算出转换矩阵，为什么不用平滑后的值呢?
Tbn = Quat2Tbn(states(1:4));
Tbn_prev = Tbn;

% transform body delta velocities to delta velocities in the nav frame 同pdf
delVelNav = Tbn * correctedDelVel + [0;0;gravity]*dt;

% take a copy of the previous velocity
prevVel = states(5:7);

% Sum delta velocities to get the velocity
states(5:7) = states(5:7) + delVelNav(1:3);

% integrate the velocity vrctor to get the position using trapezoidal
% integration
states(8:10) = states(8:10) + 0.5 * dt * (prevVel + states(5:7));

end