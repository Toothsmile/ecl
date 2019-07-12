function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NE position innovations (m)
    varInnov] ... % NE position innovation variance (m^2)
    = FusePosition( ...
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measPos, ... % NE position measurements (m)
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_OBS,... % position observation variance (m)^2
    R_HGT) %position height variance (m)^2
%fuse position只估计了
% innovation = zeros(1,2);
% varInnov = zeros(1,2);
% H = zeros(2,24);

innovation = zeros(1,3);
varInnov = zeros(1,3);
H = zeros(3,24);
for obsIndex = 1:3
    
    % velocity states start at index 8
    stateIndex = 7 + obsIndex;

    % Calculate the velocity measurement innovation
    innovation(obsIndex) = states(stateIndex) - measPos(obsIndex);%新息
    
    % Calculate the observation Jacobian
    H(obsIndex,stateIndex) = 1;
    if(obsIndex~=3)
        varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + R_OBS);%计算位置协方�?
    else
        varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + R_HGT);
    end
    
end

% Apply an innovation consistency check 更新的一致�?�?��？？�?
% 这个应该就是预测残差如果大于预测协方差的y则为滤波发散
for obsIndex = 1:3
    
    if (innovation(obsIndex)^2 / (gateSize^2 * varInnov(obsIndex))) > 1.0
        return;
    end
    
end

% Calculate Kalman gains and update states and covariances
for obsIndex = 1:3
    
    % Calculate the Kalman gains 
    K = (P*transpose(H(obsIndex,:)))/varInnov(obsIndex);
    
    % Calculate state corrections
    xk = K * innovation(obsIndex);
    
    % Apply the state corrections
    states = states - xk;
    
    % Update the covariance
    P = P - K*H(obsIndex,:)*P;
    
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    % 还是使协方差矩阵强制对齐
    P = 0.5*(P + transpose(P));
    
    % ensure diagonals are positive
    for i=1:24
        if P(i,i) < 0
            P(i,i) = 0;
        end
    end
    
end

end