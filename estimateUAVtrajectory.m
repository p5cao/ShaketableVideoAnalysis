phantom4Param = load('Phantom4_Video.mat');
Phantom4_Video = phantom4Param.Phantom4_Video;
groundMarkerSQ = load('ground_marker_sequence_0515.mat');
roofMarkerSQ = load('roof_marker_sequence_0515.mat');

% On ground 21 px = 0.457 m

timestamps = seconds(groundMarkerSQ.ground_marker_sequence.D);

G_marker0 = groundMarkerSQ.ground_marker_sequence.ground_marker0_px; %upperleft
G_marker0 = filloutliers(G_marker0 ,"linear");
G_marker1 = groundMarkerSQ.ground_marker_sequence.ground_marker1_px; %lowerleft
G_marker1 = filloutliers(G_marker1 ,"linear");
G_marker2 = groundMarkerSQ.ground_marker_sequence.ground_marker2_px; %bottom, set at origin
G_marker2 = filloutliers(G_marker2 ,"linear");
G_marker4 = groundMarkerSQ.ground_marker_sequence.ground_marker4_px;
G_marker4 = filloutliers(G_marker4 ,"nearest","OutlierLocations",isoutlier(G_marker4));
figure(2)

subplot(2,1,1);
plot(timestamps, G_marker0(:,1),"Color",'r','LineWidth',2)
hold on
plot(timestamps, G_marker1(:,1),"Color",'g','LineWidth',2)
hold on
plot(timestamps, G_marker2(:,1),"Color",'b','LineWidth',2)

hold on
plot(timestamps, G_marker4(:,1),"Color",'c','LineWidth',2)
legend('Ground Marker 0','Ground marker 1','Ground marker 2','Ground marker 4')
xlim([0 61]);
xlabel('sec');ylabel('pixel')
title('Y Pixels Ground targets')

subplot(2,1,2);
plot(timestamps, G_marker0(:,2),"Color",'r','LineWidth',2)
hold on
plot(timestamps, G_marker1(:,2),"Color",'g','LineWidth',2)
hold on
plot(timestamps, G_marker2(:,2),"Color",'b','LineWidth',2)
hold on
plot(timestamps, G_marker4(:,2),"Color",'c','LineWidth',2)
legend('Ground Marker 0','Ground marker 1','Ground marker 2', 'Ground marker 4')
xlim([0 61]);
xlabel('sec');ylabel('pixel')
title('X Pixels Ground targets')


%% convert to coordinates in world frame

% Assume all ground points stationary
% marker 2
X_2 = 0; Y_2 = 0; Z_2 = 0;
% marker 0
X_0 = mean(G_marker0(:,1)-G_marker2(:,1))*0.457/21; %m
Y_0 = mean(-G_marker0(:,2)+G_marker2(:,2))*0.457/21; %m
Z_0 = 0; %m
% marker 1
X_1 = mean(G_marker1(:,1)-G_marker2(:,1))*0.457/21; %m
Y_1 = mean(-G_marker1(:,2)+G_marker2(:,2))*0.457/21; %m
Z_1 = 0; %m
% marker 4
X_4 = mean(G_marker4(:,1)-G_marker2(:,1))*0.457/21; %m
Y_4 = mean(-G_marker4(:,2)+G_marker2(:,2))*0.457/21; %m
Z_4 = 0; %m

worldPoints = [X_0 Y_0 Z_0; X_1 Y_1 Z_1; X_2 Y_2 Z_2; X_4 Y_4 Z_4];


%% retrieve UAV drift/ Camera pose trajectory
focalLength = Phantom4_Video.FocalLength;
principalPoint = Phantom4_Video.PrincipalPoint;
imageSize = [2160 3840];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

cameraPoseWorld = {};
cameraPositionWorld = [timestamps zeros(numel(timestamps),3)];

for i = 1:numel(timestamps)
imagePoints = [G_marker0(i,:); G_marker1(i,:); G_marker2(i,:); G_marker4(i,1) 1190];

[worldPose,inlierIdx,status] = estworldpose(imagePoints,worldPoints,intrinsics,MaxNumTrials = 10000, MaxReprojectionError=20);

if status == 0
    cameraPoseWorld{i,1} = worldPose.A;
    cameraPositionWorld(i, 2:4) = worldPose.Translation;
end

end

cameraPositionWorld(:, 2:4) = filloutliers(cameraPositionWorld(:, 2:4), ...
    "nearest","OutlierLocations",isoutlier(cameraPositionWorld(:, 2:4)));

% [envHigh, envLow] = envelope(cameraPositionWorld(:, 2:4),1,'peak');
% envMean = (envHigh+envLow)/2;

cameraPositionWorld(:, 2:4) = sgolayfilt(cameraPositionWorld(:, 2:4),1,7);

figure(3)
plot(timestamps, cameraPositionWorld(:, 2),"Color",'r','LineWidth',2)
hold on
plot(timestamps, cameraPositionWorld(:, 3),"Color",'g','LineWidth',2)
hold on
plot(timestamps, cameraPositionWorld(:, 4),"Color",'b','LineWidth',2)
hold on 
legend('UAV X','UAV Y','UAV Z')

figure(4)
plot3(cameraPositionWorld(:, 2), cameraPositionWorld(:, 3), cameraPositionWorld(:, 4))
% plot3(envMean(:, 1), exponentialMA(:, 2), exponentialMA(:, 3) )
hold on
scatter3(cameraPositionWorld(1, 2), cameraPositionWorld(1, 3), cameraPositionWorld(1, 4), 'red','filled',Marker='o')
hold on
scatter3(cameraPositionWorld(end, 2), cameraPositionWorld(end, 3), cameraPositionWorld(end, 4), 'green','filled',Marker='o')
legend('Drifting trajectory','Start','End')

%% Getting Roof Marker Displacements using Xiang's 2021 paper equation 3b


% solve for s

R_marker0 = roofMarkerSQ.roof_marker_sequence.roof_marker0_px; %upperleft
R_marker0 = filloutliers(R_marker0 ,"linear");
R_marker1 = roofMarkerSQ.roof_marker_sequence.roof_marker1_px; %left
R_marker1 = filloutliers(R_marker1 ,"linear");
R_marker2 = roofMarkerSQ.roof_marker_sequence.roof_marker2_px; %lowerleft
R_marker2 = filloutliers(R_marker2 ,"linear");
R_marker3 = roofMarkerSQ.roof_marker_sequence.roof_marker3_px; %lowerright
R_marker3 = filloutliers(R_marker3 ,"linear");

K = Phantom4_Video.K;
M1 = K*diag([1 -1 -1]);
M2 = K;
% All Rs are close to diag([1 -1 -1])
% assume z is unchanged
z_tilder = 34.4932; %m (roof height agl) 

R_marker0_Positions = [timestamps zeros(numel(timestamps),3)];
s = 0;
for i = 1:numel(timestamps)
    T_w = cameraPoseWorld{i,1};
    R = T_w(1:3,1:3);
    translation = cameraPositionWorld(i, 2:4)';
    R_inv_K_inv_uv1 = inv(R)*inv(K)*[R_marker0(i,:)'; 1];
    R_inv_t1t2t3 = inv(R)*translation;       
    %get s
    %if i == 1
    s = (z_tilder+R_inv_t1t2t3(3))/R_inv_K_inv_uv1(3);
    %end
    R_marker0_worldPosition = s*R_inv_K_inv_uv1-R_inv_t1t2t3;
    R_marker0_Positions(i,2:4) = R_marker0_worldPosition;
end
R_marker1_Positions = [timestamps zeros(numel(timestamps),3)];
s = 0;
for i = 1:numel(timestamps)
    T_w = cameraPoseWorld{i,1};
    R = T_w(1:3,1:3);
    translation = cameraPositionWorld(i, 2:4)';
    R_inv_K_inv_uv1 = inv(R)*inv(K)*[R_marker1(i,:)'; 1];
    R_inv_t1t2t3 = inv(R)*translation;       
    %get s
    %if i == 1
    s = (z_tilder+R_inv_t1t2t3(3))/R_inv_K_inv_uv1(3);
    %end
    R_marker1_worldPosition = s*R_inv_K_inv_uv1-R_inv_t1t2t3;
    R_marker1_Positions(i,2:4) = R_marker1_worldPosition;
end

R_marker2_Positions = [timestamps zeros(numel(timestamps),3)];

for i = 1:numel(timestamps)
    T_w = cameraPoseWorld{i,1};
    R = T_w(1:3,1:3);
    translation = cameraPositionWorld(i, 2:4)';
    R_inv_K_inv_uv1 = inv(R)*inv(K)*[R_marker2(i,:)'; 1];
    R_inv_t1t2t3 = inv(R)*translation;       
    %get s
    %if i == 1
    s = (z_tilder+R_inv_t1t2t3(3))/R_inv_K_inv_uv1(3);
    %end
    R_marker2_worldPosition = s*R_inv_K_inv_uv1-R_inv_t1t2t3;
    R_marker2_Positions(i,2:4) = R_marker2_worldPosition;
end

R_marker3_Positions = [timestamps zeros(numel(timestamps),3)];

for i = 1:numel(timestamps)
    T_w = cameraPoseWorld{i,1};
    R = T_w(1:3,1:3);
    translation = cameraPositionWorld(i, 2:4)';
    R_inv_K_inv_uv1 = inv(R)*inv(K)*[R_marker3(i,:)'; 1];
    R_inv_t1t2t3 = inv(R)*translation;       
    %get s
    %if i == 1
    s = (z_tilder+R_inv_t1t2t3(3))/R_inv_K_inv_uv1(3);
    %end
    R_marker3_worldPosition = s*R_inv_K_inv_uv1-R_inv_t1t2t3;
    R_marker3_Positions(i,2:4) = R_marker3_worldPosition;
end


figure()
subplot(2,1,1);
plot(timestamps, R_marker0_Positions(:, 2),"Color",'r','LineWidth',2)
hold on
plot(timestamps, R_marker1_Positions(:, 2),"Color",'g','LineWidth',2)
hold on
plot(timestamps, R_marker2_Positions(:, 2),"Color",'b','LineWidth',2)

hold on
plot(timestamps, R_marker3_Positions(:, 2),"Color",'y','LineWidth',2)
legend('Roof Marker 0','Roof marker 1','Roof marker 2','Roof marker 4')
xlim([0 61]);
xlabel('sec');ylabel('meter')
title('X Coordinates Roof targets')

subplot(2,1,2);
plot(timestamps, R_marker0_Positions(:, 3),"Color",'r','LineWidth',2)
hold on
plot(timestamps, R_marker1_Positions(:, 3),"Color",'g','LineWidth',2)
hold on
plot(timestamps, R_marker2_Positions(:, 3),"Color",'b','LineWidth',2)
hold on
plot(timestamps, R_marker3_Positions(:, 3),"Color",'y','LineWidth',2)
legend('Roof Marker 0','Roof marker 1','Roof marker 2','Roof marker 4')
xlim([0 61]);
xlabel('sec');ylabel('meter')
title('Y Coordinates Roof targets')
