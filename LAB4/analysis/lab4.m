% Shaoshu Xu EECE5554 Lab4

clear;
clc;

%% Part 1 - camera calibration image preprocessing
%-------------------------------------------------
%!!!! Comment this part if running Part 2 !!!! 
%-------------------------------------------------

% resize the calibration images
filepathsrc = './calibration6/';
for i = 1:12
    temp = imread([filepathsrc, 'IMG_', num2str(2055+i), '.JPG']); % IMG2056.JPG - IMG2067.JPG
    temp = imresize(temp, [1008, 1344]); % resize
    %temp = imrotate(temp, 270); % rotate
    imwrite(temp, [filepathsrc, 'small_IMG', num2str(2055+i), '.JPG']); % write
end

%-------------------------------------------------
%!!!! Comment this part if running Part 2 !!!! 
%-------------------------------------------------

%% Part 1 - calculate the mean projection error by load variable "ex1.mat"
%-------------------------------------------------
%!!!! Comment this part if running Part 2 !!!! 
%-------------------------------------------------

mean(ex(1, :)) % "ex1.mat" was saved during the calibration part
mean(ex(2, :))
scatter((ex(1, :)), (ex(2, :)) );

%-------------------------------------------------
%!!!! Comment this part if running Part 2 !!!! 
%-------------------------------------------------

%% Part 2 - mural on the Latino Students Center building
% please run this part section by section

%% load undistort image (from camera calibration tool) and pre-processing
all_img_part2 = {};
filepathsrc = './calibration6/';
%filepathsrc = './calibration7/';
for i = 1:7
    %temp = imread([filepathsrc, 'small_IMG', num2str(2078+i), '_rect' '.jpg']);
    temp = imread([filepathsrc, 'small_IMG', num2str(1910+i), '.jpg']); % small_IMG1911.JPG - small_IMG1917.JPG
    %temp = imresize(temp, [1008, 1344]); % resize
    %temp = imrotate(temp, 270); % rotate
    temp = rgb2gray(temp); % rgb to gray
    all_img_part2(i) = {temp}; 
end

%% get features from Harris feature detector and register image pairs

% initialize features for all_img_part2(1)
I = cell2mat(all_img_part2(1));
[y,x,m] = harris(I, 1500, 'tile', [15 15], 'disp'); % [y,x,m] = harris(I, 3000, 'tile', [35 35], 'disp');
[features, valid_corners] = extractFeatures(I, [x, y]); 

% initialize all the transforms to the identity matrix
numImages = numel(all_img_part2);
tforms(numImages) = affine2d(eye(3)); % affine transform, because each images were taken with same relative orientation
%tforms(numImages) = projective2d(eye(3));

% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
    pointsPrevious = valid_corners;
    featuresPrevious = features;
        
    % Read I(n).
    I = cell2mat(all_img_part2(n)); 
    
    % Save image size.
    imageSize(n,:) = size(I);
    
    % Detect and extract Harris features for I(n).
    [y,x,m] = harris(I, 1500, 'tile', [15 15], 'disp'); % [y,x,m] = harris(I, 3000, 'tile', [35 35], 'disp');
    [features, valid_corners] = extractFeatures(I, [x, y]); 
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
    
    matchedPoints = valid_corners(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'affine', 'Confidence', 99.9, 'MaxNumTrials', 2000);  % 'affine' or 'projective'
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

%% inverting the transform for the center image and applying that transform to all the others

% Compute the output limits for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

%% initialize the panorama
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width], 'like', I);

%% make panoramic mosaic 
blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

position = zeros(numImages, 2);

% Create the panorama.
for i = 1:numImages
    
    I = cell2mat(all_img_part2(i));
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Estimate the camera center relative to the panorama
    [position(i, 1), position(i, 2)] = findPosition(mask);

    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
imshow(panorama)
hold on
scatter(position(:, 1), position(:, 2), 180, 'filled', 'r');
hold off


%% Part 3.1 - cinder block images w/ 50% overlap
% use above code in Part 2
% filepathsrc = './calibration7/';
% small_IMG2097.JPG - small_IMG2102.JPG

%% Part 3.2 - RUGGLES graffiti art w/ 15% overlap
% use above code in Part 2
% filepathsrc = './calibration7/';
% small_IMG2079.JPG - small_IMG2084.JPG
