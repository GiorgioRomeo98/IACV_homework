function [img_shape_reconstruction, H_shape, points_shape, ratio_f2_f3] = shape_reconstruction(img_affine,points_affine, debug)
    %SHAPE_RECONSTRUCTION computes the shape reconstruction of the affine 
    % image scene given in input
    %
    % output
    % img_shape_reconstruction: image obtained from applying the shape
    % reconstruction to the input image
    % H_shape: rectification matrix for shape reconstruction from
    % affine image (affine rectification of the original image)
    % points_shape: main points obtained from applying the shape
    % reconstruction to the affine main points
    % ratio_f2_f3: ration between length of facade 2 and length facade 3
    % 
    % input
    % img_affine: affine image scene over which we apply the affine reconstrution
    % points_affine: main points obtained from applying the shape reconstruction
    % to the original main points
    % debug: true to display the images
    

    % the image of the circular points I and J must be mapped onto themselves
    % (this occurs if and only if the projective transformation H is a similarity)
    % we will use an equivalent information: degenerate conic dual to (I', J')
    % where I' and J' are the images of the circular points


    %% istantiate main affine points
    upper_left_point_affine = points_affine.upper_left_point;
    upper_right_point_affine = points_affine.upper_right_point;
    lower_right_point_affine = points_affine.lower_right_point;
    lower_left_point_affine = points_affine.lower_left_point;
    upper_shadow_point_affine = points_affine.upper_shadow_point;
    
    %% store the constraints in a matrix A (we need at least two indipendent constraints)
    % pair of orthogonal lines gives rise to a constraint on s
    % [l(1)*m(1),l(1)*m(2)+l(2)*m(1), l(2)*m(2)]*s = 0

    l = cross(lower_left_point_affine, lower_right_point_affine);
    m = cross(lower_left_point_affine, upper_left_point_affine);
    
    A(1,:) = [l(1)*m(1),l(1)*m(2)+l(2)*m(1), l(2)*m(2)];


    % invariant ratio of lengths gives rise to a constraint on s
    % [seg_1(1)^2-k*seg_2(1)^2, seg_1(1)*seg_1(2)-k*seg_2(1)*seg_2(2), seg_1(2)^2-k*seg_2(2)^2]*s = 0
    k = (1/3.9)^2;  % squared ratio
    seg_1 = upper_shadow_point_affine - lower_left_point_affine;
    seg_2 = upper_left_point_affine - lower_left_point_affine;

    A(2,:) = [seg_1(1)^2-k*seg_2(1)^2, 2*(seg_1(1)*seg_1(2)-k*seg_2(1)*seg_2(2)), seg_1(2)^2-k*seg_2(2)^2];
    

    %% solve the system
    %S = [x(1) x(2); x(2) 1];
    [~,~,v] = svd(A);
    s = v(:,end); %[s11,s12,s22];
    S = [s(1),s(2); s(2),s(3)];

    
    %% compute the rectifying homography
    imDCCP = [S,zeros(2,1); zeros(1,3)]; % the image of the circular points
    [U,D,V] = svd(S);

    A = U*sqrt(D)*V';
    H = eye(3);
    H(1,1) = A(1,1);
    H(1,2) = A(1,2);
    H(2,1) = A(2,1);
    H(2,2) = A(2,2);

    H_shape = inv(H);
    tform = projective2d(H_shape');
    img_shape_reconstruction = imwarp(img_affine,tform);
    img_shape_reconstruction = imcrop(img_shape_reconstruction,[0, 380, 3140, 3000]);


    %% rectify the main points
    [upper_left_point_shape(1),upper_left_point_shape(2)]  = transformPointsForward(tform,upper_left_point_affine(1),upper_left_point_affine(2));
    [upper_right_point_shape(1),upper_right_point_shape(2)] = transformPointsForward(tform,upper_right_point_affine(1),upper_right_point_affine(2));
    [lower_right_point_shape(1),lower_right_point_shape(2)] = transformPointsForward(tform,lower_right_point_affine(1),lower_right_point_affine(2));
    [lower_left_point_shape(1),lower_left_point_shape(2)] = transformPointsForward(tform,lower_left_point_affine(1),lower_left_point_affine(2));
    [upper_shadow_point_shape(1),upper_shadow_point_shape(2)] = transformPointsForward(tform,upper_shadow_point_affine(1),upper_shadow_point_affine(2));
    

    %% shift main points to better plot them
    shift(1) = 1696 - upper_right_point_shape(1);
    shift(2) = 731 - upper_right_point_shape(2);
    upper_left_point_shape(1) = upper_left_point_shape(1)+shift(1);
    upper_left_point_shape(2) = upper_left_point_shape(2)+shift(2);
    upper_right_point_shape(1) = upper_right_point_shape(1)+shift(1);
    upper_right_point_shape(2) = upper_right_point_shape(2)+shift(2);
    lower_right_point_shape(1) = lower_right_point_shape(1)+shift(1);
    lower_right_point_shape(2) = lower_right_point_shape(2)+shift(2);
    lower_left_point_shape(1) = lower_left_point_shape(1)+shift(1);
    lower_left_point_shape(2) = lower_left_point_shape(2)+shift(2);
    upper_shadow_point_shape(1) = upper_shadow_point_shape(1)+shift(1);
    upper_shadow_point_shape(2) = upper_shadow_point_shape(2)+shift(2);


    %% assign the rectified main points to the return struct points_shape
    points_shape = struct('upper_left_point', [upper_left_point_shape';1], ...
                       'upper_right_point', [upper_right_point_shape';1], ...
                       'lower_right_point', [lower_right_point_shape';1], ...
                       'lower_left_point', [lower_left_point_shape';1], ...
                       'upper_shadow_point', [upper_shadow_point_shape';1]);


    %% compute the ratio between facade 3 and the horizontal shadow segment (invariant ratio of length)
    facade_3_length = norm(lower_right_point_shape-lower_left_point_shape, 2);
    shadow_segment_length = norm(upper_shadow_point_shape-lower_left_point_shape, 2);
    
    ratio_fs = facade_3_length/shadow_segment_length;   % ratio facade and shadow


    %% compute the ratio between facade 2 and facade 3 (invariant ratio of length)
    facade_2_length = norm(upper_left_point_shape-lower_left_point_shape, 2);
    
    ratio_f2_f3 = facade_2_length/facade_3_length;  % ratio facade 2 and facade 3


    %% show the result
    if debug

        FNT_SZ = 20;
        figure;
        imshow(img_shape_reconstruction, 'Border', 'tight'); hold on;
    
        text(upper_left_point_shape(1), upper_left_point_shape(2), 'a', 'FontSize', FNT_SZ, 'Color', 'b')
        text(upper_right_point_shape(1), upper_right_point_shape(2), 'b', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_right_point_shape(1), lower_right_point_shape(2), 'c', 'FontSize', FNT_SZ, 'Color', 'b')
        text(lower_left_point_shape(1), lower_left_point_shape(2), 'd', 'FontSize', FNT_SZ, 'Color', 'b')
        text(upper_shadow_point_shape(1), upper_shadow_point_shape(2), 'g', 'FontSize', FNT_SZ, 'Color', 'g')
    
        plot([upper_left_point_shape(1), lower_left_point_shape(1)], [upper_left_point_shape(2), lower_left_point_shape(2)], 'b');
        plot([upper_right_point_shape(1), lower_right_point_shape(1)], [upper_right_point_shape(2), lower_right_point_shape(2)], 'b');
        plot([upper_left_point_shape(1), upper_right_point_shape(1)], [upper_left_point_shape(2), upper_right_point_shape(2)], 'b');
        plot([lower_left_point_shape(1), lower_right_point_shape(1)], [lower_left_point_shape(2), lower_right_point_shape(2)], 'b');

        saveas(gcf, "images/image_shape_reconstruction.png");

        fprintf('The ratio between facade 3 and the shadow segment dg after shape reconstruction is: %f/%f = %f\n', facade_3_length, shadow_segment_length, ratio_fs);
        fprintf('The ratio between facade 2 and facade 3 after shape reconstruction is: %f/%f = %f\n', facade_2_length, facade_3_length, ratio_f2_f3);

    end



end

