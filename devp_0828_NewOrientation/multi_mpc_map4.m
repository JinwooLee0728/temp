%% ############ MULTIAGENT TRANSPORTATION PORJECT: 2025.08.28 #############
%% File for Logical Occupancy Matrix Generation for Map 4

% 1. Generating a Logical Occupancy Matrix from the png Image
image=imread("hospital_map.png");   % Read image

if length(image(1, 1, :))==3               % If image is a color image...
    gray_image=rgb2gray(image);            % Grayscale
    map.logicalMat=imbinarize(gray_image); % Binarize
else
    map.logicalMat=imbinarize(image);      % Directly binarize if black-and-white image
end

map.logicalMat=~map.logicalMat;  % This way, obstacles are represented by zeros

clear image gray_image