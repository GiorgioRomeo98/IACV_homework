%% clear all the variable
close all
clear
clc

%% add necessary path
addpath(genpath([pwd, filesep, 'utils']));
addpath(genpath([pwd, filesep, 'images']));


%% set variables
debug = true;
FNT_SZ = 20;


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

[img_affine, points_affine] = affine_reconstruction(img, debug);

[img_shape, points_shape] = shape_reconstruction(img_affine, points_affine, debug);


%% G2. Calibration












