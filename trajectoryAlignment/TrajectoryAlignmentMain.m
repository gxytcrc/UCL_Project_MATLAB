clear all;
addpath('external_library')
groundTruth = importdata('GroundTruthdouble.mat');
slamResult = textread('Trajectory\KeyFrameTrajectory.txt');
N = length(slamResult);
%% obtain the data
%translation data
Ps = zeros(N,3);
Pg = zeros(N,3);
%rotation data;
Rs = zeros(3,3,N);
Rg = zeros(3,3,N);
scale_init = 0;
for i = 1:N
    t_s = (slamResult(i, 2:4));
    q_s = slamResult(i, 5:8);
    time_s = (slamResult(i, 1) * 10)/30.353424;
    time_g = time_s - 6.78;
    [~,ng] = min(abs(time_g - groundTruth(:,2)));
    t_g = (groundTruth(ng, 7:9));
    q_g = groundTruth(ng, 3:6);
    %calculate the rotation matrix
    R_s = quat2dcm(q_s);
    R_g = quat2dcm(q_g);
    %store the data
    Ps(i,:) = t_s * 100;
    Pg(i,:) = t_g * 100; 
    Rs(:,:,i) = R_s;
    Rg(:,:,i) = R_g;
    if i~= 1
        d_s = norm(t_s-t_sp);
        d_g = norm(t_g-t_gp);
        scale_init = scale_init + d_g/d_s;
    end
    t_sp = t_s;
    t_gp = t_g;
end
scale_init = scale_init/(N-1);
Ps = Ps * scale_init;
%% ICP method
%get the 3D point
R_sum = eye(3);
t_sum = [0;0;0];
for loop = 1:1000
    %calculate the average
    As = mean(Ps);
    Ag = mean(Pg);
    %calcualte the covarance
    Cs = zeros(N,3);
    Cg = zeros(N,3);
    for i = 1:N
        Cs(i,:) = Ps(i,:) - As;
        Cg(i,:) = Pg(i,:) - Ag;
    end
    A = Cg'*Cs;
    [U,S,V] = svd(A);
    R = V*U';
    t = As' - R*Ag';
    for i = 1:N
        Pg(i,:) = (R*Pg(i,:)'+t)';
    end
    R_sum = R * R_sum;
    t_sum = t_sum+t;
end
scatter3(Ps(:,1),Ps(:,2),Ps(:,3),100,'.');
hold on;
scatter3(Pg(:,1),Pg(:,2),Pg(:,3),100,'.');
legend("SLAM result", "Ground Truth");
%% Hand eye calibration
R_rs = zeros(3,3,N-1);
R_rg = zeros(3,3,N-1);
T_rs = zeros(3,N-1);
T_rg = zeros(3,N-1);
AR = zeros(9,9);
A = zeros(3*(N-1),4);
B = zeros(3*(N-1),1);
for i = 1:N
    R_s = Rs(:,:,i);
    R_g = R_sum * Rg(:,:,i);
    t_s = Ps(i,:)';
    t_g = Pg(i,:)';
    if i~= 1
        R1 = R_s * inv(R_sp);
        R2 = R_g * inv(R_gp);
        t1 = (t_s - R1*t_sp);
        t2 = t_g - R2*t_gp;
        product = eye(9)-kron(R1,R2);
        AR((i-2)*9+1:(i-1)*9, 1:9) = product;
        A((i-2)*3+1:(i-1)*3, 1:3) = R1-eye(3);
        A((i-2)*3+1:(i-1)*3, 4) = -t1;
        %store the data
        R_rs(:,:,i-1) = R1;
        R_rg(:,:,i-1) = R2;
        T_rs(:,i-1) = t1;
        T_rg(:,i-1) = t2;
    end
    R_sp = R_s;
    t_sp = t_s;
    R_gp = R_g;
    t_gp = t_g;
end
[V,D] = eig(AR'*AR);
R_add = [(V(1:3,1)),(V(4:6,1)), (V(7:9,1))];

[U,S,V] = svd(R_add);
R_add = U*V.';
if det(R_add)<0
    R_add = U*diag([1,1,-1])*V.';
end

for i = 1:N-1
    R1 = R_rs(:,:,i);
    R2 = R_rg(:,:,i);
    R2 = R_add*R2*inv(R_add);
    D_R2(i) = norm((R1-R2)./R1);
end
R_add = eye(3);
for i=1:N-1
    t1 = T_rs(:,i);
    t2 = T_rg(:,i);
    B((i-1)*3+1:(i)*3) = -(R_add*t2);
end
X = ((A'*A)\A')*B;
t_add = X(1:3);
s_add = X(4);
%% process ICP
%slamResult2 = textread('KeyFrameTrajectory5.txt');
slamResult2 = textread('Trajectory\AllKeyFrameTrajectory.txt');
N = length(slamResult2);
Plot_s = zeros(N,3);
Plot_g = zeros(N,3);
for i = 1:N
    id = i;
    t_s = (slamResult2(id, 2:4));
    time_s = (slamResult2(id, 1) * 10)/30.353424;
    time_g = time_s - 6.86;
    [~,ng] = min(abs(time_g - groundTruth(:,2)));
    t_g = (groundTruth(ng, 7:9))*100;
    Plot_s(i,:) = t_s * (scale_init*100) * s_add;
    Plot_g(i,:) = (R_add*(R_sum * t_g'+t_sum))';
end
%the icp is using the external function, the reference is shown in the
%bottom
[R, t] = icp(Plot_s', Plot_g', 1000, 'Matching', 'kDtree');

%% plot the data
slamResult3 = textread('Trajectory\AllKeyFrameTrajectory.txt');
N = length(slamResult3);
Plot_s = zeros(N,3);
Plot_g = zeros(N,3);
for i = 1:N
    id = i;
    t_s = (slamResult3(id, 2:4));
    time_s = (slamResult3(id, 1) * 10)/30.353424;
    time_g = time_s - 6.86;
    [~,ng] = min(abs(time_g - groundTruth(:,2)));
    t_g = (groundTruth(ng, 7:9))*100;
    Plot_s(i,:) = t_s * (scale_init*100) * s_add;
    Plot_g(i,:) = (R_add*(R_sum * t_g'+t_sum))';
end

for i= 1:N
    Plot_g(i,:) = (R*Plot_g(i,:)'+t)';
end
figure;
scatter3(Plot_s(:,1),Plot_s(:,2),Plot_s(:,3),100,'.');
hold on;
scatter3(Plot_g(:,1),Plot_g(:,2),Plot_g(:,3),100,'.');
axis equal;
legend("SLAM result", "Ground Truth");
% hold on;
% plot3(Plot_s(:,1),Plot_s(:,2),Plot_s(:,3),'LineWidth', 5);
% hold on;
% plot3(Plot_g(:,1),Plot_g(:,2),Plot_g(:,3),'LineWidth', 5);
axis equal;
error = Plot_g - Plot_s;
for i = 1:N
    dis(i) = (error(i,1)^2 + error(i,2)^2 + error(i,3)^2)^0.5;
end
eva = mean(dis);
%%
% icp() is the external libraries from Jakob Wilm, https://uk.mathworks.com/matlabcentral/fileexchange/27804-iterative-closest-point 