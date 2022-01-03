function img_affine_reconstruction = affine_reconstruction(img,debug)
    % AFFINE_RECONSTRUCTION computes the affine mapping of the image scene
    % given in input
    %
    % output
    % img_affine_reconstruction: image obtained from applying the affine
    % reconstruction to the input image
    % 
    % input
    % img: original image over which we apply the affine reconstrution
    % debug: true to display the images
    

    % the image of the line at the infinity must be mapped into itself
    % (this occurs if and only if the projective transformation H is affine)
    % H = [1    0    0
    %      0    1    0
    %      l1   l2   l3]
    % where l1, l2, l3 are the coordinates of the normalized line at the
    % infinite


    % istantiate the four points used to compute the parallel lines
    upper_left_point = [199; 561; 1];   % a
    upper_right_point = [854; 584; 1];  % b
    lower_right_point = [746; 733; 1];  % c
    lower_left_point = [254; 722; 1];   % d
    
    % compute lines
    upper_horizontal_line = cross(upper_left_point, upper_right_point);
    lower_horizontal_line = cross(lower_left_point, lower_right_point);
    left_oblique_line = cross(upper_left_point, lower_left_point);
    right_oblique_line = cross(upper_right_point, lower_right_point);
     
    % compute vanishing points
    vanishing_point_A = cross(upper_horizontal_line, lower_horizontal_line);
    vanishing_point_B = cross(left_oblique_line, right_oblique_line);
    
    % normalize vanishing points
    vanishing_point_A = vanishing_point_A / vanishing_point_A(3);
    vanishing_point_B = vanishing_point_B / vanishing_point_B(3);

    % compute the image of the line at the infinity
    im_line_inf = cross(vanishing_point_A, vanishing_point_B);
    
    % normalize the image of the line at the infinity
    im_line_inf = im_line_inf / im_line_inf(3);
    

    %% plot points, parallel lines, vanishing points, line at the infinite
    if debug
        FNT_SZ = 20;
        figure("Name","Original image"), imshow(img), hold on;
        text(upper_left_point(1), upper_left_point(2), 'a', 'FontSize', FNT_SZ, 'Color', 'b')
        text(upper_right_point(1), upper_right_point(2), 'b', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_right_point(1), lower_right_point(2), 'c', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_left_point(1), lower_left_point(2), 'd', 'FontSize', FNT_SZ, 'Color', 'b')
        
        plot([upper_left_point(1), vanishing_point_A(1)], [upper_left_point(2), vanishing_point_A(2)], 'b');
        plot([lower_left_point(1), vanishing_point_A(1)], [lower_left_point(2), vanishing_point_A(2)], 'b');
        plot([upper_left_point(1), vanishing_point_B(1)], [upper_left_point(2), vanishing_point_B(2)], 'b');
        plot([upper_right_point(1), vanishing_point_B(1)], [upper_right_point(2), vanishing_point_B(2)], 'b');
        
        text(vanishing_point_A(1), vanishing_point_A(2), 'v1', 'FontSize', FNT_SZ, 'Color', 'b')
        text(vanishing_point_B(1), vanishing_point_B(2), 'v2', 'FontSize', FNT_SZ, 'Color', 'b')
        
        plot([vanishing_point_A(1), vanishing_point_B(1)], [vanishing_point_A(2), vanishing_point_B(2)], 'r--')

    end

    
    %% build the rectification matrix
    H_affine = [eye(2),zeros(2,1); im_line_inf(:)'];
    
    if debug
        % we can check that H^-T* imLinfty is the line at infinity in its canonical form:
        fprintf('The vanishing line is mapped to:\n');
        disp(inv(H_affine)'*im_line_inf);
    end

    
    %% rectify the image and show the result
    tform = projective2d(H_affine');
    img_affine_reconstruction = imwarp(img, tform);
    img_affine_reconstruction = imcrop(img_affine_reconstruction,[5290, 7500, 3000, 2000]);

    if debug
        figure("Name", 'Affine rectification')
        imshow(img_affine_reconstruction, "Border", 'tight');
        saveas(gcf, "images/image_affine_reconstruction.png");
    end

end

