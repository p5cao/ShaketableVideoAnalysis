function [xBox_out,yBox_out] = getBoundingBoxRoof(im_gray, roof_marker0_roi, threshold)

% if nargin == 3
%     edge_detect = 'canny';
%     % edge_detect = 'sobel';
% end
marker_crop = im_gray(min(roof_marker0_roi(1,1),roof_marker0_roi(3,1)):max(roof_marker0_roi(1,1),roof_marker0_roi(3,1)), ...
                 min(roof_marker0_roi(1,2),roof_marker0_roi(2,2)):max(roof_marker0_roi(1,2),roof_marker0_roi(2,2)));

marker_crop_00 = reshape(marker_crop,[size(marker_crop,1)*size(marker_crop,2) 1]);

for ii = 1:length(marker_crop_00)
    if marker_crop_00(ii) < threshold
        marker_crop_00(ii) = uint8(0);
    else
        marker_crop_00(ii) = uint8(255);
    end
end
marker_crop_00 = reshape(marker_crop_00,[size(marker_crop,1) size(marker_crop,2)]);

% imshow(marker_crop_00)

I = marker_crop_00;
%if strcmp(pattern, 'wb')
Ibw = ~imbinarize(I);
% else
%     if strcmp(pattern, 'bw')
%         Ibw = imbinarize(I);
%     end
% end
% imshow(Ibw)
Ifill = imfill(Ibw,'holes');
Iarea = bwareaopen(Ifill,60);
Ifinal = bwlabel(Iarea);
% for debug
% imshow(Ifinal);
box_struct = regionprops(Ifinal,'boundingbox');

marker_bounding_boxes = {};
kk = 1; 
for ii = 1:numel(box_struct)
    % if kk >=3
    %     break
    % end
    bb = box_struct(ii).BoundingBox;
    % for debug
    %rectangle('position',bb,'edgecolor','r','linewidth',2);
    if (bb(3)>=7 && bb(3)<=11) && (bb(4)>=7 && bb(4)<=11) 
        if kk == 1
            marker_bounding_boxes{kk,1} = bb;
            kk = kk+1;
        else
            if kk >= 2
                for jj = 1:numel(marker_bounding_boxes)
                    last_bb = marker_bounding_boxes{jj,1};
                    diff_bb = last_bb - bb;
                    if (abs(diff_bb(1))<=13) && ...
                            (abs(diff_bb(2))<=13)
                        marker_bounding_boxes{numel(marker_bounding_boxes)+1,1} = bb;
                    end
                end
            kk = kk + 1;
            end
        end
    % else 
    %     if (bb(3)>=20 && bb(3)<=25) && (bb(4)>=20 && bb(4)<=25)
    %     marker_bounding_boxes{kk,1} = bb;
    %     %kk = kk+1;
    % end
    end
end
x = []; y= [];

for jj = 1:numel(marker_bounding_boxes)
    bbox = marker_bounding_boxes{jj,1};
    x = [x; bbox(2);bbox(2)+bbox(4)];
    y = [y; bbox(1);bbox(1)+bbox(3)];
end

x1 = min(x);
x2 = max(x);
y1 = min(y);
y2 = max(y);
xBox = [x1, x2, x2, x1, x1]; %+ min(roof_marker0_roi(:,2));
yBox = [y1, y1, y2, y2, y1]; %+ min(roof_marker0_roi(:,1));
% imshow(marker_crop_00)
% hold on
% plot(yBox, xBox, 'g-',  'LineWidth' ,2);

xBox_out = yBox + min(roof_marker0_roi(1,2),roof_marker0_roi(2,2))-1;
yBox_out = xBox+ min(roof_marker0_roi(1,1),roof_marker0_roi(3,1))-1;

hold on
plot(xBox_out, yBox_out, 'g-',  'LineWidth' ,2);