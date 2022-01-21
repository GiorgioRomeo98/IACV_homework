function  [img_vertical_rectification, ratio_f3_height_f3_length] = vertical_reconstruction(img, K, points, debug)
    % VERTICAL_RECONSTRUCTION performs the rectification of a vertical facade
    %
    % output
    % img_vertical_rectification: image obtained from the image in input
    % after performing rectification of the vertical facade
    %
    % input
    % img: original image over which we apply the rectification of a
    % vertical facade
    % K: calibration matrix
    % points: main points obtained from the original image
    % debug: true to display the images and results


    %% compute the image of the absolute conic through the calibration matrix K
    IAC = inv(K * K');
  
    IAC = IAC / norm(IAC);


    %% image of the line at the infinite belonging to a plane orthogonal to the horizontal one
    vertical_im_line_infty = cross(points.vertical_vp_1, points.vertical_vp_2);

    % normalize the image of the line at the infinity
    vertical_im_line_infty = vertical_im_line_infty / vertical_im_line_infty(3);


    %% assign the coefficients to write the equations
    a1 = IAC(1,1);
    b1 = IAC(1,2)*2;
    c1 = IAC(1,3)*2;
    d1 = IAC(2,2);
    e1 = IAC(2,3)*2;
    f1 = IAC(3,3);
    a2 = vertical_im_line_infty(1);
    b2 = vertical_im_line_infty(2);
    c2 = vertical_im_line_infty(3);


    %% build the system and find the intersection between image of the absolute conic and the line at the infinity
    syms 'x';
    syms 'y';
    
    % image of absolute conic
    eq1 = a1*x^2 + b1*x*y + d1*y^2 + c1*x + e1*y + f1;
    
    % image of line at infinity
    eq2 = a2*x + b2*y + c2;
    
    eqns = [eq1 == 0, eq2 == 0];
    S = solve(eqns, [x,y]);
    
    s1 = [double(S.x(1)); double(S.y(1)); 1];
    s2 = [double(S.x(2)); double(S.y(2)); 1];


    %% compute the rectification matrix
    II = s1;
    JJ = s2;
    imDCCP = II*JJ.' + JJ*II.';
    imDCCP = imDCCP./norm(imDCCP);
    
    [U,D,~] = svd(imDCCP);  % ~ = U.'
    D(3,3) = 1;
    H_rect = inv(U*sqrt(D));


    %% apply the rectification to the image
    tform = projective2d(H_rect');
    img_vertical_rectification = imwarp(img,tform);
    img_vertical_rectification = imrotate(img_vertical_rectification, 180);


    %% apply the rectification to the main points
    [upper_left_corner_rect(1),upper_left_corner_rect(2)]  = transformPointsForward(tform,points.upper_left_corner(1),points.upper_left_corner(2));
    [upper_right_corner_rect(1),upper_right_corner_rect(2)] = transformPointsForward(tform,points.upper_right_corner(1),points.upper_right_corner(2));
    [lower_right_corner_rect(1),lower_right_corner_rect(2)] = transformPointsForward(tform,points.lower_right_corner(1),points.lower_right_corner(2));
    [lower_left_corner_rect(1),lower_left_corner_rect(2)] = transformPointsForward(tform,points.lower_left_corner(1),points.lower_left_corner(2));
    

    %% change main points assignment in order to match the original configuration
    % the image rotation create a mismatch since the points do not rotate
    % as the rectified image does
    temp = upper_left_corner_rect;
    upper_left_corner_rect = lower_right_corner_rect;
    lower_right_corner_rect = temp;
    temp = upper_right_corner_rect;
    upper_right_corner_rect = lower_left_corner_rect;
    lower_left_corner_rect = temp;


    %% shift main points to better plot them
    shift(1) = 541 - lower_right_corner_rect(1);
    shift(2) = 1766 - lower_right_corner_rect(2);
    upper_left_corner_rect(1) = upper_left_corner_rect(1)+shift(1);
    upper_left_corner_rect(2) = upper_left_corner_rect(2)+shift(2);
    upper_right_corner_rect(1) = upper_right_corner_rect(1)+shift(1);
    upper_right_corner_rect(2) = upper_right_corner_rect(2)+shift(2);
    lower_right_corner_rect(1) = lower_right_corner_rect(1)+shift(1);
    lower_right_corner_rect(2) = lower_right_corner_rect(2)+shift(2);
    lower_left_corner_rect(1) = lower_left_corner_rect(1)+shift(1);
    lower_left_corner_rect(2) = lower_left_corner_rect(2)+shift(2);


    %% compute the ratio between facade 3 height and facade 3 length
    facade_3_length = norm(lower_right_corner_rect-lower_left_corner_rect, 2);
    facade_3_height = norm(upper_right_corner_rect-lower_right_corner_rect, 2);

    ratio_f3_height_f3_length = facade_3_height / facade_3_length;     


    %% plot the result
    if debug
        FNT_SZ = 20;
        figure('Name', 'Rectification of the facade 3 (vertical facade)');
        view(180,90);
        imshow(img_vertical_rectification, 'Border', 'tight'); hold on;

        text(upper_left_corner_rect(1), upper_left_corner_rect(2), 'e', 'FontSize', FNT_SZ, 'Color', 'g')
        text(upper_right_corner_rect(1), upper_right_corner_rect(2), 'f', 'FontSize', FNT_SZ, 'Color', 'g')
        text(lower_right_corner_rect(1), lower_right_corner_rect(2), 'g', 'FontSize', FNT_SZ, 'Color', 'g')
        text(lower_left_corner_rect(1), lower_left_corner_rect(2), 'h', 'FontSize', FNT_SZ, 'Color', 'g')
    
        plot([upper_left_corner_rect(1), upper_right_corner_rect(1)], [upper_left_corner_rect(2), upper_right_corner_rect(2)], 'g');
        plot([upper_right_corner_rect(1), lower_right_corner_rect(1)], [upper_right_corner_rect(2), lower_right_corner_rect(2)], 'g');
        plot([lower_right_corner_rect(1), lower_left_corner_rect(1)], [lower_right_corner_rect(2), lower_left_corner_rect(2)], 'g');
        plot([lower_left_corner_rect(1), upper_left_corner_rect(1)], [lower_left_corner_rect(2), upper_left_corner_rect(2)], 'g');

        saveas(gcf, 'images/image_rectification_vertical_facade.png');

        fprintf('The ratio between facade 3 height and facade 3 length after rectification vertical facade is: %f/%f = %f\n', facade_3_height, facade_3_length, ratio_f3_height_f3_length);

    end
 



end

