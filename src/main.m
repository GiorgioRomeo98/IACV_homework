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
    figure, imshow(img);
end