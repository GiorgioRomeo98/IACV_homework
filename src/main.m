%% clear all the variable
close all
clear
clc

%% add necessary path
addpath(genpath([pwd, filesep, 'utils']));
addpath(genpath([pwd, filesep, 'images']));


%% set variables
debug = false;


%% load the image
img = imread('images/Villa.png');

if debug
    figure("Name","Original image"), imshow(img);
end


%% F1. Feature extraction
img_edges = edge_detection(img,debug);

img_lines = line_extraction(img, img_edges, debug);

img_corners = corner_detection(img, debug);


%% G1. 2D reconstruction of a horizontal section
% stratified appraoch: image scene --> affine reconstruction --> shape reconstruction

[img_affine, H_affine, points, points_affine] = affine_reconstruction(img, debug);

[img_shape, H_shape, points_shape, ratio_f2_f3] = shape_reconstruction(img_affine, points_affine, debug);

% compute the composite transformation:
% image scene --> affine reconstruction --> shape reconstruction
% img -> affine -> euclidean -> rotation
% H_r is the transformation from the original image to the shape reconstruction.
H_r = H_shape * H_affine;


%% G2. Calibration

[K, points] = camera_calibration(img, points, H_r, debug);


%% G3. Reconstruction of a vertical facade

img_vertical_facade_reconstruction = vertical_reconstruction(img, K, points, debug);


%% G4. Localization














