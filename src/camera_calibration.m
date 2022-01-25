function [K, points] = camera_calibration(img, points, H, debug)
    % CAMERA_CALIBRATION returns the calibration matrix K
    %
    % output
    % K: calibration matrix
    % points: updated struct of main points obtained from the original image
    % 
    % input:
    % img: original image
    % points: main points obtained from the original image
    % H: homography for the shape reconstruction from the original image
    % debug: true to display the images and results



    %% assign other main points useful for camera calibration
    points.upper_left_corner = [276;490;1];
    points.upper_right_corner = [728;507;1];
    points.lower_right_corner = [803;1296;1];
    points.lower_left_corner = [203;1310;1];


    %% vertical vanishing points
    % compute vertical lines
    left_vertical_line = cross(points.lower_left_corner, points.upper_left_corner);
    right_vertical_line = cross(points.lower_right_corner, points.upper_right_corner);
    upper_horizontal_line = cross(points.upper_left_corner,  points.upper_right_corner);
    lower_horizontal_line = cross(points.lower_left_corner,  points.lower_right_corner);

    %compute vertical vanishing points
    vertical_vp_1 = cross(left_vertical_line, right_vertical_line);
    vertical_vp_2 = cross(upper_horizontal_line, lower_horizontal_line);

    %normalize vanishing points
    vertical_vp_1 = vertical_vp_1/vertical_vp_1(3);
    vertical_vp_2 = vertical_vp_2/vertical_vp_2(3);

    % assign the vertical vanishing points to the struct
    points.vertical_vp_1 = vertical_vp_1;
    points.vertical_vp_2 = vertical_vp_2;


    %% plot the main points and lines to show horizontal vanishing points
    if debug
        FNT_SZ = 20;
        figure("Name","Original image - horizontal vanishing points"), imshow(img, "Border", 'tight'), hold on;
        text(points.upper_left_point(1), points.upper_left_point(2), 'a', 'FontSize', FNT_SZ, 'Color', 'b')
        text(points.upper_right_point(1), points.upper_right_point(2), 'b', 'FontSize', FNT_SZ, 'Color', 'b')
        text(points.lower_right_point(1), points.lower_right_point(2), 'c', 'FontSize', FNT_SZ, 'Color', 'b')
        text(points.lower_left_point(1), points.lower_left_point(2), 'd', 'FontSize', FNT_SZ, 'Color', 'b')
        
        plot([points.upper_left_point(1), points.horizontal_vp_1(1)], [points.upper_left_point(2), points.horizontal_vp_1(2)], 'b');
        plot([points.lower_left_point(1), points.horizontal_vp_1(1)], [points.lower_left_point(2), points.horizontal_vp_1(2)], 'b');
        plot([points.upper_left_point(1), points.horizontal_vp_2(1)], [points.upper_left_point(2), points.horizontal_vp_2(2)], 'b');
        plot([points.upper_right_point(1), points.horizontal_vp_2(1)], [points.upper_right_point(2), points.horizontal_vp_2(2)], 'b');
        
        text(points.horizontal_vp_1(1), points.horizontal_vp_1(2), 'vp1', 'FontSize', FNT_SZ, 'Color', 'r')
        text(points.horizontal_vp_2(1), points.horizontal_vp_2(2), 'vp2', 'FontSize', FNT_SZ, 'Color', 'r')
        
        plot([points.horizontal_vp_1(1), points.horizontal_vp_2(1)], [points.horizontal_vp_1(2), points.horizontal_vp_2(2)], 'r--')

    end


    %% plot the main points and lines to show vertical vanishing points
    if debug
        FNT_SZ = 20;
        figure("Name","Original image - vertical vanishing points"), imshow(img, "Border", 'tight'), hold on;

        text(points.upper_left_corner(1), points.upper_left_corner(2), 'e', 'FontSize', FNT_SZ, 'Color', 'g')
        text(points.upper_right_corner(1), points.upper_right_corner(2), 'f', 'FontSize', FNT_SZ, 'Color', 'g')
        text(points.lower_right_corner(1), points.lower_right_corner(2), 'g', 'FontSize', FNT_SZ, 'Color', 'g')
        text(points.lower_left_corner(1), points.lower_left_corner(2), 'h', 'FontSize', FNT_SZ, 'Color', 'g')

        plot([points.lower_left_corner(1), points.vertical_vp_1(1)], [points.lower_left_corner(2), points.vertical_vp_1(2)], 'g');
        plot([points.lower_right_corner(1), points.vertical_vp_1(1)], [points.lower_right_corner(2), points.vertical_vp_1(2)], 'g');
        plot([points.upper_left_corner(1), points.upper_right_corner(1)], [points.upper_left_corner(2), points.upper_right_corner(2)], 'g');
        plot([points.lower_left_corner(1), points.lower_right_corner(1)], [points.lower_left_corner(2), points.lower_right_corner(2)], 'g');
        saveas(gcf, "images/image_original_vertical_vp.png");
        plot([points.upper_left_corner(1), points.vertical_vp_2(1)], [points.upper_left_corner(2), points.vertical_vp_2(2)], 'g');
        plot([points.lower_left_corner(1), points.vertical_vp_2(1)], [points.lower_left_corner(2), points.vertical_vp_2(2)], 'g');

        %text(points.vertical_vp_1(1), points.vertical_vp_1(2), 'vp3', 'FontSize', FNT_SZ, 'Color', 'r')
        %text(points.vertical_vp_2(1), points.vertical_vp_2(2), 'vp4', 'FontSize', FNT_SZ, 'Color', 'r')

        plot([points.vertical_vp_1(1), points.vertical_vp_2(1)], [points.vertical_vp_1(2), points.vertical_vp_2(2)], 'r--')

        saveas(gcf, "images/image_original_two_vertical_vp.png");

    end

    
    %% compute the image of the line at the infinity
    im_line_infty = cross(points.horizontal_vp_1, points.horizontal_vp_2);

    % normalize the image of the line at the infinity
    im_line_infty = im_line_infty / im_line_infty(3);


    %% compute image of the absolute conic
    H = inv(H);
    IAC = get_image_absolute_conic(im_line_infty, points.vertical_vp_1, points.horizontal_vp_1, points.horizontal_vp_2, H);


    %% get the intrinsic parameter before the denormalization
    alfa = sqrt(IAC(1,1));
    u0 = -IAC(1,3)/(alfa^2);
    v0 = -IAC(2,3);
    fy = sqrt(IAC(3,3) - (alfa^2)*(u0^2) - (v0^2));
    fx = fy / alfa;
    K = [fx 0 u0; 0 fy v0; 0 0 1];


    %% get intrinsic after denormalization
    fx = K(1,1);
    fy = K(2,2);
    u0 = K(1,3);
    v0 = K(2,3);
    aspect_ratio = fx/fy;


    %% show the results
    if debug
        fprintf("The focal distance along x axis is: %f\n\n", fx);
        fprintf("The focal distance along y axis is: %f\n\n", fy);
        fprintf("The aspect ration is: %f/%f = %f\n\n", fx, fy, aspect_ratio);
        fprintf("The principal points are: U_0 = %f, V_0 = %f\n\n", u0, v0);
        fprintf("The calibration matrix K is: "); display(K);
    end
    

end

