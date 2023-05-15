clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
writeToDisk = true;
movieFullFileName = fullfile('2023-04-18-WN_Rx-plane_view.mp4');

videoObject = VideoReader(movieFullFileName);
frameRate = videoObject.FrameRate;
% Determine how many frames there are.
numberOfFrames = videoObject.NumberOfFrames;

clear videoObject
grayscaleMovieFileName = fullfile('GrayBBox-2023-04-18-WN_Rx-plane_view.mp4');

frameFolderName = sprintf('Plan_view_frames');

writerObj = VideoWriter(grayscaleMovieFileName, 'MPEG-4');
writerObj.FrameRate = 6;
writerObj.Quality = 100;
open(writerObj);
timestamps = 0.0:1.0/frameRate:1.0/frameRate*(numberOfFrames-1);

frameFolderName = sprintf('Plan_view_frames');

outputFolder = sprintf('grayscale_bounding_frames');
% Create the folder if it doesn't exist already.
if ~exist(outputFolder, 'dir')
	mkdir(outputFolder);
end

ground_marker0_px = [];
ground_marker1_px = [];
ground_marker2_px = [];
ground_marker4_px = [];


roof_marker0_px = [];
roof_marker1_px = [];
roof_marker2_px = [];
roof_marker3_px = [];
roof_marker4_px = [];
roof_marker5_px = [];


for frame = 1 : 5: numberOfFrames

    imageName = sprintf('%4.4d.png', frame);
    inputFullFileName = fullfile(frameFolderName, imageName);
    im = imread(inputFullFileName);
    im_gray = rgb2gray(im);
    clear im

    ax1 = figure(1);
    imshow(im_gray)
    hold on
    % ground_marker_mask = zeros(size(im_gray));
    
    da = 80; % drift adjustment
    
    marker0_roi = [395-da 1170-da; 395-da 1220+da; 433+da 1220+da; 433+da 1170-da; 395-da 1170-da];
    marker1_roi = [1533-da 1075-da; 1533-da 1110+da; 1565+da 1110+da; 1565+da 1075-da; 1533-da 1075-da];
    marker2_roi = [1666-da 1860-da; 1666-da 1910+da; 1713+da 1910+da; 1713+da 1860-da; 1666-da 1860-da];
    marker3_roi = [388-da 2670-da; 388-da 2710+da; 427+da 2710+da; 427+da 2670-da; 388-da 2670-da];
    
    marker4_roi = [1176-da 2537-da; 1176-da 2579+da; 1224+da 2579+da; 1224+da 2537-da; 1176-da 2537-da];
    
    ground_marker_rois = {marker0_roi;marker1_roi;marker2_roi;marker3_roi};
    
    marker_crop = im_gray(marker0_roi(1,1):marker0_roi(3,1), ...
                      marker0_roi(1,2):marker0_roi(2,2));
    
    figure(1)
    [xbox0,ybox0] = getBoundingBoxGround(im_gray, 230, marker0_roi,'sobel');
    ground_marker0_px = [ground_marker0_px; [(xbox0(1)+xbox0(2))/2 (ybox0(2)+ybox0(3))/2]];

    [xbox1,ybox1] = getBoundingBoxGround(im_gray, 200, marker1_roi,'sobel');
    ground_marker1_px = [ground_marker1_px; [(xbox1(1)+xbox0(2))/2 (ybox1(2)+ybox1(3))/2]];
    
    [xbox2,ybox2] = getBoundingBoxGround(im_gray, 210, marker2_roi,'canny');
    ground_marker2_px = [ground_marker2_px; [(xbox2(1)+xbox2(2))/2 (ybox2(2)+ybox2(3))/2]];
    
    % [xbox3,ybox3] = getBoundingBox(im_gray, 240, marker3_roi);
    
    [xbox4,ybox4] = getBoundingBoxGround(im_gray, 250, marker4_roi,'canny');
    ground_marker4_px = [ground_marker4_px; [(xbox4(1)+xbox4(2))/2 (ybox4(2)+ybox4(3))/2]];
    
    %Set marker0 to be the origin of world frame
    
    
    %% Get bounding box of dynamic targets on building
    
    dba = 150; % drift and building dynamic response adjustment
    
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
    [roof_xbox0,roof_ybox0] = getBoundingBoxRoof(im_gray,roof_marker0_roi, 110);
    if ~isempty([roof_xbox0,roof_ybox0])
        roof_marker0_px = [roof_marker0_px; [(roof_xbox0(2)+roof_xbox0(3))/2 ...
            (roof_ybox0(1)+roof_ybox0(2))/2]];
    else
        roof_marker0_px = [roof_marker0_px; roof_marker0_px(end,:)];
    end

    [roof_xbox1,roof_ybox1] = getBoundingBoxRoof(im_gray,roof_marker1_roi, 70);
    if ~isempty([roof_xbox1,roof_ybox1])
        roof_marker1_px = [roof_marker1_px; [(roof_xbox1(2)+roof_xbox1(3))/2 ...
            (roof_ybox1(1)+roof_ybox1(2))/2]];
    else
        roof_marker1_px = [roof_marker1_px; roof_marker1_px(end,:)];
    end

    [roof_xbox2,roof_ybox2] = getBoundingBoxRoof(im_gray,roof_marker2_roi, 70);
    if ~isempty([roof_xbox2,roof_ybox2])
        roof_marker2_px = [roof_marker2_px; [(roof_xbox2(2)+roof_xbox2(3))/2 ...
            (roof_ybox2(1)+roof_ybox2(2))/2]];
    else
        roof_marker2_px = [roof_marker2_px; roof_marker2_px(end,:)];
    end

    [roof_xbox3,roof_ybox3] = getBoundingBoxRoof(im_gray,roof_marker3_roi, 70);
    if ~isempty([roof_xbox3,roof_ybox3])
        roof_marker3_px = [roof_marker3_px; [(roof_xbox3(2)+roof_xbox3(3))/2 ...
            (roof_ybox3(1)+roof_ybox3(2))/2]];
    else
        roof_marker3_px = [roof_marker3_px; roof_marker3_px(end,:)];
    end

    [roof_xbox4,roof_ybox4] = getBoundingBoxRoof(im_gray,roof_marker4_roi, 100);
    if ~isempty([roof_xbox4,roof_ybox4])
        roof_marker4_px = [roof_marker4_px; [(roof_xbox4(2)+roof_xbox4(3))/2 ...
            (roof_ybox4(1)+roof_ybox4(2))/2]];
    else
        roof_marker4_px = [roof_marker4_px; roof_marker4_px(end,:)];
    end

    [roof_xbox5,roof_ybox5] = getBoundingBoxRoof(im_gray,roof_marker5_roi, 110);
    if ~isempty([roof_xbox5,roof_ybox5])
        roof_marker5_px = [roof_marker5_px; [(roof_xbox5(2)+roof_xbox5(3))/2 ...
            (roof_ybox5(1)+roof_ybox5(2))/2]];
    else
        roof_marker5_px = [roof_marker5_px; roof_marker5_px(end,:)];
    end

    %% use this to save images
    % iptsetpref('ImshowBorder','tight');
    % outputFullFileName =  fullfile(outputFolder, imageName);
    % imwrite(getframe(gca).cdata,outputFullFileName)


    % Update user with the progress.  Display in the command window.
	if writeToDisk
		progressIndication = sprintf('Wrote frame %4d of %d.', frame, numberOfFrames);
	else
		progressIndication = sprintf('Processed frame %4d of %d.', frame, numberOfFrames);
	end
	disp(progressIndication);

end

rowTimes = timestamps(1:5:end)';
D = seconds(rowTimes);
ground_marker_sequence = timetable(D,ground_marker0_px,ground_marker1_px,ground_marker2_px,ground_marker4_px);
roof_marker_sequence = timetable(D,roof_marker0_px,roof_marker1_px,roof_marker2_px,roof_marker3_px, ...
    roof_marker4_px, roof_marker5_px);
%% save as a video

thisFrame = imread(outputFullFileName);
vidHeight = size(thisFrame,1);
vidWidth = size(thisFrame,2);
% First get a cell array with all the frames.
allTheFrames = cell((frame-1)/5+1,1);
allTheFrames(:) = {zeros(vidHeight, vidWidth, 3, 'uint8')};

allTheColorMaps = cell((frame-1)/5+1,1);
allTheColorMaps(:) = {zeros(256, 3)};

recalledMovie = struct('cdata', allTheFrames, 'colormap', allTheColorMaps);

for frame = 1 : 5: numberOfFrames
	% Construct an output image file name.
	outputBaseFileName = sprintf('%4.4d.png', frame);
	outputFullFileName = fullfile(outputFolder, outputBaseFileName);
	% Read the image in from disk.
	thisFrame = imread(outputFullFileName);
	% Convert the image into a "movie frame" structure.
	recalledMovie(frame) = im2frame(thisFrame);
	% Write this frame out to a new video file.
	writeVideo(writerObj, thisFrame);

    % Update user with the progress.  Display in the command window.
	% if writeToDisk
	% 	progressIndication = sprintf('Wrote frame %4d of %d.', frame, numberOfFrames);
	% else
	progressIndication = sprintf('Processed frame %4d of %d.', frame, numberOfFrames);
	% end
	disp(progressIndication);
end
close(writerObj);
