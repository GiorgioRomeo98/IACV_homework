function [img_affine_reconstruction, H_affine, points, points_affine] = affine_reconstruction(img,debug)
    % AFFINE_RECONSTRUCTION computes the affine mapping of the image scene
    % given in input
    %
    % output
    % img_affine_reconstruction: image obtained from applying the affine
    % reconstruction to the input image
    % H_affine: rectification matrix for affine reconstruction from
    % original image
    % points: main points obtained from the original image and needed to
    % perform affine rectification
    % points_affine: main points obtained from applying the affine
    % reconstruction to the original main points (istantiated at the
    % beginning of the function)
    % 
    % input
    % img: original image over which we apply the affine reconstrution
    % debug: true to display the images
    

    % the image of the line at the infinity must be mapped onto itself
    % (this occurs if and only if the projective transformation H is affine)
    % H = [1    0    0
    %      0    1    0
    %      l1   l2   l3]
    % where l1, l2, l3 are the coordinates of the normalized line at the
    % infinite

    
    %% istantiate main points
    % istantiate four points used to compute the parallel lines
    upper_left_point = [227; 334; 1];   % a
    upper_right_point = [821; 364; 1];  % b
    lower_right_point = [732; 552; 1];  % c
    lower_left_point = [271; 535; 1];   % d

    % istantiate two points belonging to the shadow vertical line
    lower_shadow_point = [343; 1300; 1];
    middle_shadow_point = [342; 1065; 1];
    

    %% compute lines
    upper_horizontal_line = cross(upper_left_point, upper_right_point);
    lower_horizontal_line = cross(lower_left_point, lower_right_point);
    left_oblique_line = cross(upper_left_point, lower_left_point);
    right_oblique_line = cross(upper_right_point, lower_right_point);
    shadow_line = cross(lower_shadow_point, middle_shadow_point);


    %% compute intersection between the shadow vertical line and the lower_horizontal_line
    upper_shadow_point = cross(shadow_line, lower_horizontal_line);

    % normalize
    upper_shadow_point = upper_shadow_point/upper_shadow_point(3);

    
    %% horizontal vanishing points
    % compute vanishing points
    vanishing_point_1 = cross(upper_horizontal_line, lower_horizontal_line);
    vanishing_point_2 = cross(left_oblique_line, right_oblique_line);
    
    % normalize vanishing points
    vanishing_point_1 = vanishing_point_1 / vanishing_point_1(3);
    vanishing_point_2 = vanishing_point_2 / vanishing_point_2(3);
    

    %% image of the line at the infinity
    % compute the image of the line at the infinity
    im_line_infty = cross(vanishing_point_1, vanishing_point_2);
    
    % normalize the image of the line at the infinity
    im_line_infty = im_line_infty / im_line_infty(3);
    

    %% plot main points, parallel lines, vanishing points, line at the infinite
    if debug
        FNT_SZ = 20;
        figure("Name","Original image"), imshow(img, "Border", 'tight'), hold on;
        text(upper_left_point(1), upper_left_point(2), 'a', 'FontSize', FNT_SZ, 'Color', 'b')
        text(upper_right_point(1), upper_right_point(2), 'b', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_right_point(1), lower_right_point(2), 'c', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_left_point(1), lower_left_point(2), 'd', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_shadow_point(1), lower_shadow_point(2), 'e', 'FontSize', FNT_SZ, 'Color', 'g')
        text(lower_shadow_point(1), middle_shadow_point(2), 'f', 'FontSize', FNT_SZ, 'Color', 'g')
        text(upper_shadow_point(1), upper_shadow_point(2), 'g', 'FontSize', FNT_SZ, 'Color', 'g')
        
        plot([upper_left_point(1), vanishing_point_1(1)], [upper_left_point(2), vanishing_point_1(2)], 'b');
        plot([lower_left_point(1), vanishing_point_1(1)], [lower_left_point(2), vanishing_point_1(2)], 'b');
        plot([upper_left_point(1), vanishing_point_2(1)], [upper_left_point(2), vanishing_point_2(2)], 'b');
        plot([upper_right_point(1), vanishing_point_2(1)], [upper_right_point(2), vanishing_point_2(2)], 'b');
        plot([lower_shadow_point(1), upper_shadow_point(1)], [lower_shadow_point(2), upper_shadow_point(2)], 'g');
       
        text(vanishing_point_1(1), vanishing_point_1(2), 'vp1', 'FontSize', FNT_SZ, 'Color', 'r')
        text(vanishing_point_2(1), vanishing_point_2(2), 'vp2', 'FontSize', FNT_SZ, 'Color', 'r')
        
        plot([vanishing_point_1(1), vanishing_point_2(1)], [vanishing_point_1(2), vanishing_point_2(2)], 'r--')

        saveas(gcf, "images/image_original_lines.png");

    end

    
    %% build the rectification matrix
    H_affine = [eye(2),zeros(2,1); im_line_infty(:).'];
    
    if debug
        display(H_affine);

        % we can check that H^-T* imLinfty is the line at infinity in its canonical form:
        fprintf('The vanishing line is mapped to:\n');
        disp(inv(H_affine)'*im_line_infty);
    end

    
    %% rectify the image
    tform = projective2d(H_affine');
    img_affine_reconstruction = imwarp(img, tform);
    img_affine_reconstruction = imcrop(img_affine_reconstruction,[6010, 8332, 3000, 2000]);


    %% rectify the main points
    [upper_left_point_affine(1),upper_left_point_affine(2)]  = transformPointsForward(tform,upper_left_point(1),upper_left_point(2));
    [upper_right_point_affine(1),upper_right_point_affine(2)] = transformPointsForward(tform,upper_right_point(1),upper_right_point(2));
    [lower_right_point_affine(1),lower_right_point_affine(2)] = transformPointsForward(tform,lower_right_point(1),lower_right_point(2));
    [lower_left_point_affine(1),lower_left_point_affine(2)] = transformPointsForward(tform,lower_left_point(1),lower_left_point(2));
    [upper_shadow_point_affine(1),upper_shadow_point_affine(2)] = transformPointsForward(tform,upper_shadow_point(1),upper_shadow_point(2));


    %% assign the original main points to the return struct points
    points = struct('upper_left_point', upper_left_point, ...
                    'upper_right_point', upper_right_point, ...
                    'lower_right_point', lower_right_point, ...
                    'lower_left_point', lower_left_point, ...
                    'upper_shadow_point', upper_shadow_point, ...
                    'horizontal_vp_1', vanishing_point_1, ...
                    'horizontal_vp_2', vanishing_point_2);

    %% assign the rectified main points to the return struct points_affine
    points_affine = struct('upper_left_point', [upper_left_point_affine';1], ...
                           'upper_right_point', [upper_right_point_affine';1], ...
                           'lower_right_point', [lower_right_point_affine';1], ...
                           'lower_left_point', [lower_left_point_affine';1], ...
                           'upper_shadow_point', [upper_shadow_point_affine';1]);


    %% compute the ratio between facade 3 and the horizontal shadow segment dg (invariant ratio of length of collinear lines)
    facade_3_length = norm(lower_right_point_affine-lower_left_point_affine, 2);
    shadow_segment_length = norm(upper_shadow_point_affine-lower_left_point_affine, 2);
    
    ratio = facade_3_length/shadow_segment_length;



    %% show the result
    if debug
        
        figure("Name", 'Affine rectification');
        imshow(img_affine_reconstruction, "Border", 'tight'); hold on;

        text(upper_left_point_affine(1), upper_left_point_affine(2), 'a', 'FontSize', FNT_SZ, 'Color', 'b')
        text(upper_right_point_affine(1), upper_right_point_affine(2), 'b', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_right_point_affine(1), lower_right_point_affine(2), 'c', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_left_point_affine(1), lower_left_point_affine(2), 'd', 'FontSize', FNT_SZ, 'Color', 'b')
        text(upper_shadow_point_affine(1), upper_shadow_point_affine(2), 'g', 'FontSize', FNT_SZ, 'Color', 'g')

        plot([upper_left_point_affine(1), lower_left_point_affine(1)], [upper_left_point_affine(2), lower_left_point_affine(2)], 'b');
        plot([upper_right_point_affine(1), lower_right_point_affine(1)], [upper_right_point_affine(2), lower_right_point_affine(2)], 'b');
        plot([upper_left_point_affine(1), upper_right_point_affine(1)], [upper_left_point_affine(2), upper_right_point_affine(2)], 'b');
        plot([lower_left_point_affine(1), lower_right_point_affine(1)], [lower_left_point_affine(2), lower_right_point_affine(2)], 'b');

        fprintf('The ratio between facade 3 and the shadow segment dg after affine reconstruction is: %f/%f = %f\n', facade_3_length, shadow_segment_length, ratio);

        saveas(gcf, "images/image_affine_reconstruction.png");
    end

end

