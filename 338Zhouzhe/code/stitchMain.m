clear;  %clear all previous data
%%
%%Step 1 : Find input images path
imagesFolder = 'input';  %Find the direction folder of input images

images = dir(imagesFolder); %Input all images of imagesFolder to variable images

images = images(arrayfun(@(x) x.name(1) ~= '.', images)); %files which have ./ are removed
%%
%%Step 2 : SIFT feature points computation, build the SIFT descriptors
for i = 1 : length(images)
    filename = fullfile(imagesFolder, images(i).name);
    input_image = single(rgb2gray(imread(filename)));
    [keypoints{i}, descriptors{i}] = vl_sift(input_image);
end


for i=1:length(images)-1
    
    % it is more easy to use that transfer relevant data struture to
    % variables
    descriptors1 = descriptors{i};
    descriptors2 = descriptors{i + 1};
    keypoints1 = keypoints{i};
    keypoints2 = keypoints{i + 1};
    
    %%Step 3. Match SIFT descriptors
    [matches, scores] = vl_ubcmatch(descriptors1, descriptors2) ;
    im1_ftr_pts = keypoints1([2 1], matches(1, :))';
    im2_ftr_pts = keypoints2([2 1], matches(2, :))';
    
    
   %%Step 4. call ransac to compute homographies and remove wrong matches 
   %call ransac method to calculate H and store homography matrices into
   %HArrayList
    HArrayList{i} = ransac(im1_ftr_pts, im2_ftr_pts);
end

%% Step 5. Wrap images
%
% Choose a reference imgae first, usually choose the middle one to decrease
% distortion
Homographies{1} = eye(3); %eye(3) creates identity matrix
for i = 1:size(HArrayList, 2)
    Homographies{i+1} = Homographies{i} * HArrayList{i};
    %Homographies is a list store 3 x 3 gomography matrices
end

% compute output picture size
min_row = 1;
min_col = 1;
max_row = 0;
max_col = 0;

% loop for every image
for i=1:length(Homographies)
    currentImage = imread(fullfile(imagesFolder, images(i).name));
    [rows,cols,~] = size(currentImage);
    
   %A matrix store four coordinate date of the picture
    cornerMatrix = cat(3, [1,1,1]', [1,cols,1]', [rows, 1,1]', [rows,cols,1]');
    
   %Adjust four coordinate data according to the reference image
    for j=1:4
        result = Homographies{i}*cornerMatrix(:,:,j);
    
        min_row = floor(min(min_row, result(1)));
        min_col = floor(min(min_col, result(2)));
        max_row = ceil(max(max_row, result(1)));
        max_col = ceil(max(max_col, result(2))); 
    end
    
end

% Calculate output image size
ouoputImage_height = max_row - min_row + 1;
ouoputImage_width = max_col - min_col + 1;

% Calculate off set value
row_offset = 1 - min_row;
col_offset = 1 - min_col;

% Perform inverse mapping for each input image
for i=1:length(Homographies)
    
    % Create a list of all pixels' coordinates in output image
    [x,y] = meshgrid(1:ouoputImage_width, 1:ouoputImage_height);
    % Create list of all row coordinates and column coordinates in separate
    % vectors, x and y, including offset
    x = reshape(x,1,[]) - col_offset;
    y = reshape(y,1,[]) - row_offset;
    
    % Create homogeneous coordinates for each pixel in output image
    pan_pts(1,:) = y;
    pan_pts(2,:) = x;
    pan_pts(3,:) = ones(1,size(pan_pts,2));
    
    % Perform inverse warp to compute coordinates in current input image
    image_coords = Homographies{i}\pan_pts;
    row_coords = reshape(image_coords(1,:),ouoputImage_height, ouoputImage_width);
    col_coords = reshape(image_coords(2,:),ouoputImage_height, ouoputImage_width);
    currentImage = im2double(imread(fullfile(imagesFolder, images(i).name)));
    
    % color value adjust
    currentWarpedImage = zeros(ouoputImage_height, ouoputImage_width, 3);
    for channel = 1 : 3
        currentWarpedImage(:, :, channel) = ...
            interp2(currentImage(:,:,channel), ...
            col_coords, row_coords, 'linear', 0);
    end
    %add current picture to arrary list wrapedImages to blend later
    wrapedImages{i} = currentWarpedImage;      
end

%% Step 6. Blend images
% Initialize output image to black (0)
ouoputImage_image = zeros(ouoputImage_height, ouoputImage_width, 3);
output_image = wrapedImages{1};
for i = 2 : length(wrapedImages)    
    %Call Blender function to blend separate images together by using
    %feathering
    output_image = Blender(output_image, wrapedImages{i});
end
%Write output image to correct direction
imwrite(output_image, 'output/stitched.png');
%Show the output image
imshow(output_image);
%%
