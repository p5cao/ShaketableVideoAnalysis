function [xBox,yBox] = getBoundingBoxGround(im_gray,threshold, marker0_roi, edge_detect)

if nargin == 3
    edge_detect = 'canny';
end
marker_crop = im_gray(min(marker0_roi(1,1),marker0_roi(3,1)):max(marker0_roi(1,1),marker0_roi(3,1)), ...
                 min(marker0_roi(1,2),marker0_roi(2,2)):max(marker0_roi(1,2),marker0_roi(2,2)));


marker_crop_00 = reshape(marker_crop,[size(marker_crop,1)*size(marker_crop,2) 1]);

for ii = 1:length(marker_crop_00)
    if marker_crop_00(ii) >= threshold
        marker_crop_00(ii) = uint8(255);
    else
        marker_crop_00(ii) = uint8(0);
    end
end
marker_crop_00 = reshape(marker_crop_00,[size(marker_crop,1) size(marker_crop,2)]);


BW2 = edge(marker_crop_00,edge_detect);

[B,L] = bwboundaries(BW2, 'noholes');
%select the longest boundary sequence to plot a bounding box
boundary_length = 0;
for ii = 1:length(B)
    if length(B{ii,1})>boundary_length
        boundary_length = length(B{ii,1});
        boundary = B{ii,1};
    else
        continue
    end
end
x = boundary(:,2);
y = boundary(:,1);
x1 = min(x);
x2 = max(x);
y1 = min(y);
y2 = max(y);
xBox = [x1, x2, x2, x1, x1] + min(marker0_roi(1,2),marker0_roi(2,2))-1;
yBox = [y1, y1, y2, y2, y1]+ min(marker0_roi(1,1),marker0_roi(3,1))-1;


hold on 

plot(xBox, yBox, 'r-',  'LineWidth' ,2);