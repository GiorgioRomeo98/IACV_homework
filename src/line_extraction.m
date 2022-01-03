function img_lines = line_extraction(img, img_edges, debug)
    
    % LINE_EXTRACTION finds the straight lines of the image applying Hough
    % transform
    %
    % output
    % img_lines: detected lines of the input image 
    %
    % input parameters
    % img: original image
    % img_edges: input image obtained as a result of the Canny algorithm for edge detection 
    % debug: true to display the images
    
    %% Hough transform
    % Create the Hough transform using the binary image.
    [H,theta,rho] = hough(img_edges,'RhoResolution',0.65,'Theta',-90:0.5:89.5);
    
    
    % Find peaks in the Hough transform of the image and plot them.
    P  = houghpeaks(H,40,'threshold',ceil(0.3*max(H(:))));
    if debug
        x = theta(P(:,2)); y = rho(P(:,1));
        figure("Name", "peaks in the Hough transform"),
        imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
        xlabel('\theta'), ylabel('\rho');
        axis on, axis normal, hold on;
        plot(x,y,'s','color','white');
    end

    % Find lines and plot them.
    img_lines = houghlines(img_edges,theta,rho,P,'FillGap',400,'MinLength',50);
    
    if debug
        figure("Name", "Extracted lines"), imshow(img, "Border", 'tight'), hold on
        plot_lines(img_lines);
        saveas(gcf, "images/image_lines.png");
    end
    
    

end