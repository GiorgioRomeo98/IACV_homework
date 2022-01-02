%% clear all the variable
close all
clear
clc

%% add necessary path
addpath(genpath([pwd, filesep, 'utils']));
addpath(genpath([pwd, filesep, 'images']));


%% set global variables
debug = true;
FNT_SZ = 28;


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