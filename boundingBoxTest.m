im = imread('1500.png');
im_gray = rgb2gray(im);

ax1 = figure(1);
imshow(im_gray)
hold on
% ground_marker_mask = zeros(size(im_gray));

da = 100; % drift adjustment

marker0_roi = [395-da 1170-da; 395-da 1220+da; 433+da 1220+da; 433+da 1170-da; 395-da 1170-da];
marker1_roi = [1533-da 1075-da; 1533-da 1110+da; 1565+da 1110+da; 1565+da 1075-da; 1533-da 1075-da];
marker2_roi = [1666-da 1860-da; 1666-da 1910+da; 1713+da 1910+da; 1713+da 1860-da; 1666-da 1860-da];
marker3_roi = [388-da 2670-da; 388-da 2710+da; 427+da 2710+da; 427+da 2670-da; 388-da 2670-da];

marker4_roi = [1176-da 2537-da; 1176-da 2579+da; 1224+da 2579+da; 1224+da 2537-da; 1176-da 22537-da];

ground_marker_rois = {marker0_roi;marker1_roi;marker2_roi;marker3_roi};

marker_crop = im_gray(marker0_roi(1,1):marker0_roi(3,1), ...
                  marker0_roi(1,2):marker0_roi(2,2));

figure(1)
[xbox0,ybox0] = getBoundingBoxGround(im_gray, 220, marker0_roi,'sobel');

[xbox1,ybox1] = getBoundingBoxGround(im_gray, 200, marker1_roi,'sobel');

[xbox2,ybox2] = getBoundingBoxGround(im_gray, 210, marker2_roi,'canny');

% [xbox3,ybox3] = getBoundingBox(im_gray, 240, marker3_roi);

[xbox4,ybox4] = getBoundingBoxGround(im_gray, 250, marker4_roi,'canny');

%Set marker0 to be the origin of world frame


%% Get bounding box of dynamic targets on building

dba = 100; % drift and building dynamic response adjustment

roof_marker0_roi = [600-dba 1589-dba; 600-dba 1647+dba; 646+dba 1647+dba; ...
    646+dba 1589-dba; 600-dba 1589-dba];
roof_marker1_roi = [879-dba 1498-dba; 879-dba 1542+dba; 915+dba 1542+dba; ...
    915+dba 1498-dba; 879-dba 1498-dba];
roof_marker2_roi = [1340-dba 1531-dba; 1340-dba 1560+dba; 1381+dba 1560+dba;...
    1381+dba 1531-dba; 1340-dba 1531-dba];
roof_marker3_roi = [1507-dba 2325-dba; 1507-dba 2370+dba; 1542+dba 2370+dba;...
    1542+dba 2325-dba; 1507-dba 2325-dba];
roof_marker4_roi = [942-dba 2364-dba; 942-dba 2410+dba; 980+dba 2410+dba;...
    942-dba 2410+dba; 942-dba 2364-dba];
roof_marker5_roi = [579-dba 2324-dba; 579-dba 2365+dba; 622+dba 2365+dba; ...
    579-dba 2365+dba; 579-dba 2324-dba];

marker_crop = im_gray(roof_marker1_roi(1,1):roof_marker1_roi(3,1), ...
                  roof_marker1_roi(1,2):roof_marker1_roi(2,2));
% figure(2)
% imshow(marker_crop)

ax1 = figure(1);
[roof_xbox0,roof_ybox0] = getBoundingBoxRoof(im_gray,roof_marker0_roi, 70);

[roof_xbox1,roof_ybox1] = getBoundingBoxRoof(im_gray,roof_marker1_roi, 70);

[roof_xbox2,roof_ybox2] = getBoundingBoxRoof(im_gray,roof_marker2_roi, 70);

[roof_xbox3,roof_ybox3] = getBoundingBoxRoof(im_gray,roof_marker3_roi, 70);

[roof_xbox4,roof_ybox4] = getBoundingBoxRoof(im_gray,roof_marker4_roi, 100);

[roof_xbox5,roof_ybox5] = getBoundingBoxRoof(im_gray,roof_marker5_roi, 110);
