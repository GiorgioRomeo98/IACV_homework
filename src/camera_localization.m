function [cameraPosition, cameraRotation] = camera_localization(K, points, ratio_f2_f3, ratio_f3_height_f3_width, debug)
    % CAMERA_LOCALIZATION locates the camera with respect facade 3
    
    % input
    % points: main points highlighted in the original image
    % ratio_f2_f3: ratio between facade 2 width and facade 3 width
    % atio_f3_height_f3_width: ration between facade 3 height and width
    % debug: true to print all the images
    

    % we first extract the rotation of the vertical plane with respect
    % the camera frame, then we compute the rotation and translation of the
    % camera with respect to the plane
    LENGTH_LONGSIDE = 1500; % height of facade 3 (it will be scaled later on)


    %% fit the homography to restore real measure (possible since we know the aspect ratio)
    % build up the rectangle of the vertical facade using its real measure
    lower_left_point = [0 0];
    upper_left_point = [0 LENGTH_LONGSIDE];
    lower_right_point = [LENGTH_LONGSIDE/ratio_f3_height_f3_width 0];
    upper_right_point = [LENGTH_LONGSIDE/ratio_f3_height_f3_width LENGTH_LONGSIDE];

    points.upper_left_corner = [272; 535; 1];
    points.upper_right_corner = [732; 552; 1];
    points.lower_right_corner = [802; 1295; 1];
    points.lower_left_corner = [202; 1312; 1];

    % fit the homography from scene to image 
    H_homography = fitgeotrans([upper_left_point; lower_left_point; lower_right_point; upper_right_point], ...
                              [points.upper_left_corner(1:2).'; points.lower_left_corner(1:2).'; points.lower_right_corner(1:2).'; points.upper_right_corner(1:2).'], ...
                              'projective');

    H_homography = H_homography.T.';

    if debug
        display(H_homography);
    end

    % extract columns
    h1 = H_homography(:,1);
    h2 = H_homography(:,2);
    h3 = H_homography(:,3);
    
    % normalization factor
    lambda = 1 / norm(K \ h1);
    
    % r2 = K^-1 * h1 normalized
    r2 = (K \ h1) * lambda; % r2 = j
    r3 = (K \ h2) * lambda; % r3 = k
    r1 = cross(r2,r3); % r1 = i
    
    %% compute rotation of the world with respect to the camera (R cam -> world)
    % in this case the world is the vertical facade
    R = [r1, r2, r3];

    % due to noise in the data R may be not a true rotation matrix.
    % approximate it through svd, obtaining a orthogonal matrix
    [U, ~, V] = svd(R);
    R = U * V';
    
    % Compute translation vector. This vector is the position of the plane
    % with respect to the reference frame of the camera
    T = (K \ (lambda * h3));
    
    cameraRotation = R.';
    % since T is expressed in the camera reference frame we want it in the plane
    % reference frame, R.' is the rotation of the camera with respect to the plane
    cameraPosition = -R.'*T;

    scaling = 1.50 / cameraPosition(3);
    cameraPosition = cameraPosition .* scaling;
    
    if debug
        display(cameraPosition)
        display(cameraRotation)
    end

    %% Display orientation and position of camera from vertical facade 3
    if debug

        figure('Name', 'Camera location')
        plotCamera('Location', cameraPosition, 'Orientation', cameraRotation.', 'Size', 0.5);
        hold on;
        facade_2_width = ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_width * scaling;
        facade_3_width = LENGTH_LONGSIDE/ratio_f3_height_f3_width * scaling;
        upper_left_point = upper_left_point * scaling;
        lower_left_point = lower_left_point * scaling;
        lower_right_point = lower_right_point * scaling;
        upper_right_point = upper_right_point * scaling;

        pcshow([[upper_left_point; lower_left_point; lower_right_point; upper_right_point], ...
        zeros(size([upper_left_point; lower_left_point; lower_right_point; upper_right_point],1), 1)], ...
        'red','VerticalAxisDir', 'up', 'MarkerSize', 20);
    
        patch(zeros(size([upper_left_point; lower_left_point; lower_right_point; upper_right_point],1), 1),...
              [upper_left_point(1); lower_left_point(1); lower_right_point(1); upper_right_point(1)], ...
              [upper_left_point(2); lower_left_point(2); lower_right_point(2); upper_right_point(2)], ...
              'green');
    
        patch([facade_2_width, facade_2_width, 0, 0],...
              [upper_left_point(1); lower_left_point(1); lower_left_point(1); upper_left_point(1)], ...
              [upper_left_point(2); lower_left_point(2); lower_left_point(2); upper_left_point(2)], ...
              'red');

        patch([facade_2_width, facade_2_width, facade_2_width, facade_2_width], ...
              [upper_right_point(1); lower_right_point(1); lower_right_point(1)+facade_3_width; upper_right_point(1)+facade_3_width], ...
              [upper_right_point(2); lower_right_point(2); lower_right_point(2); upper_right_point(2)], ...
              'green');         
    
        patch([facade_2_width, facade_2_width, 0, 0],...
              [upper_right_point(1); lower_right_point(1); lower_right_point(1); upper_right_point(1)], ...
              [upper_right_point(2); lower_right_point(2); lower_right_point(2); upper_right_point(2)], ...
              'red');

        patch([facade_2_width, facade_2_width, facade_2_width, facade_2_width], ...
              [upper_left_point(1); lower_left_point(1); lower_left_point(1)-facade_3_width; upper_left_point(1)-facade_3_width], ...
              [upper_left_point(2); lower_left_point(2); lower_left_point(2); upper_left_point(2)], ...
              'green');        

        xlabel('X')
        ylabel('Y')
        zlabel('Z')

    end


end

