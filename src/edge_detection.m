function img_edges = edge_detection(img_rgb, debug)
    
    % edge_detection performs pre-processing of the input image img_rgb and
    % detect the edges of the image by Canny edge detection
    %
    % output
    % img_edges: binary image containing 1s where the function employed 
    % finds edges in the grayscale or binary image I and 0s elsewhere
    %
    % input parameters
    % img_rgb: input image over which we compute the edges
    % debug: true to display the images
    
    
    %% convert the input image in greyscale and normalize
    img_grey = double(rgb2gray(img_rgb))./255;
    if debug
        figure("Name","Original image in grayscale"), imshow(img_grey);
    end
    
    
    %% apply Canny edge detection
    % edge(I,method,threshold,sigma) returns a binary image containing 1s
    % where the function finds edges in the grayscale or binary image I and
    % 0s elsewhere, using the edge-detection algorithm specified by "method". 
    % If the additional parameters are specified: the returned edges are
    % stronger than "threshold". "sigma" is the standard deviation of the
    % Gaussian filter
    
    [BW,th] = edge(img_grey,'canny');
    % we use the threshold identified by the default canny algorithm as a
    % starting point to tune the "threshold" and "sigma" parameters
    if debug
        figure("Name", "Edges detected through Canny method (default setting)");
        imshow(BW)
    end
    
    % after several attempts threshold = th*0.5 and sigma = sqrt(3)
    % give good result
    BW = edge(img_grey,'canny', th*0.5, sqrt(3));
    if debug
        figure("Name", 'Canny Edge Detection');
        imshow(BW, "Border", 'tight')
        saveas(gcf, "images/image_edges.png");
    end
    
    img_edges = BW;

    
    %% testing
%     coeff = 0.49;
%     for i= 1:11
%         coeff = coeff + 0.01;
%         th_new = th .* coeff;
%         BW = edge(img_grey,'canny', th_new);
%         figure;
%         imshow(BW)
%     end

% sos = sqrt(2);
%     for i= 1:50
%         sos = sos + 0.02;
%         BW = edge(img_grey,'canny', th.*0.55, sos);
%         figure;
%         imshow(BW)
%     end





end