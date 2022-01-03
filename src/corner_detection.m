function img_corners = corner_detection(img_rgb,debug)
    
    % CORNER_DETECTION performs pre-processing of the input image img_rgb and
    % detect the corners of the image by Harris corner detection
    %
    % output
    % img_corners: contains information about the feature points (corners)  
    % detected in a 2-D input image, img_rgb.
    %
    % input parameters
    % img_rgb: input image over which we compute the corners
    % debug: true to display the images

    img_rgb = double(rgb2gray(img_rgb))./255;
    %y = imadjust(y,[],[],0.8);
     
    dx = [-1 0 1; -1 0 1; -1 0 1];   % Derivative masks
    dy = dx';
    
    Ix = convn(img_rgb, dx, 'same');      % Image derivatives
    Iy = convn(img_rgb, dy, 'same');
    
    % set the parameter for Gaussian convolution used in Harris Corner Detector
    SIGMA_gaussian = 0.35;
    g = fspecial('gaussian',max(1,fix(3*SIGMA_gaussian)+1), SIGMA_gaussian);
    
    Ix2 = convn(Ix.^2, g, 'same'); % Smoothed squared image derivatives
    Iy2 = convn(Iy.^2, g, 'same');
    Ixy = convn(Ix.*Iy, g, 'same');
    
    % cim = det(M) - k trace(M)^2.
    k = 0.015;
    cim = (Ix2.*Iy2 - Ixy.^2) - k * (Ix2 + Iy2);
    
    
    %% remove boundaries of cim which is going to have large values because of zero padding of the image
    BORDER=20;
    cim(1:BORDER,:)=0;
    cim(end-BORDER:end,:)=0;
    cim(:,end-BORDER:end)=0;
    cim(:,1:BORDER)=0;
    
    %% Thresholding the cim
    T=mean(cim(:));
    CIM=cim;
    CIM(find(cim<T))=0;
    % similarly one could use the Otzu method
    
    
    %% perform nonlocal maximum suppression on the thresholded measure
    % this value needs to be adjusted also depending on the image size
    support=true(25);
    % compute maximum over a square neighbor of size 25 x 25
    
    maxima=ordfilt2(CIM,sum(support(:)),support);
    % determine the locations where the max over the neigh or 25 x 25 corresponds to the cim values
    [loc_x,loc_y]=find((cim==maxima).*(CIM>0));
    
    
    % draw a cross on the image in the local maxima
    if debug
        figure("Name", 'Local maxima of Harris measure'), imshow(img_rgb,[], "Border", 'tight'), hold on,
        plot(loc_y,loc_x,'g+', 'LineWidth', 0.5)
        saveas(gcf, "images/image_corners.png");
    end
    
    img_corners = [loc_x,loc_y];
    
    
end

