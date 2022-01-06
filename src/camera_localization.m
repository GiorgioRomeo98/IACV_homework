function camera_localization(K, points, ratio_f2_f3, ratio_f3_height_f3_length, debug)
    % CAMERA_LOCALIZATION Summary of this function goes here
    % Detailed explanation goes here


    % we first extract the rotation of the (vertical ??????) plane with respect
    % the camera frame, then we compute the rotation and translation of the
    % camera with respect to the plane
    LENGTH_LONGSIDE = 1500;


    %% fit the homography to restore real measure (possible since we know the aspect ratio)
    % build up the rectangle of the vertical facade (face??????) using its real measure
    lower_left_point = [0 0];
    upper_left_point = [0 LENGTH_LONGSIDE];
    lower_right_point = [LENGTH_LONGSIDE/ratio_f3_height_f3_length 0];
    upper_right_point = [LENGTH_LONGSIDE/ratio_f3_height_f3_length LENGTH_LONGSIDE];

    % fit the homography from scene to image 
    H_omography = fitgeotrans([upper_left_point; lower_left_point; lower_right_point; upper_right_point], ...
                              [points.upper_left_corner(1:2).'; points.lower_left_corner(1:2).'; points.lower_right_corner(1:2).'; points.upper_right_corner(1:2).'], ...
                              'projective');

    H_omography = H_omography.T.';

    % extract columns
    h1 = H_omography(:,1);
    h2 = H_omography(:,2);
    h3 = H_omography(:,3);
    
    % normalization factor.
    lambda = 1 / norm(K \ h1);
    
    % r1 = K^-1 * h1 normalized
    r1 = (K \ h1) * lambda;
    r2 = (K \ h2) * lambda;
    r3 = cross(r1,r2);
    
    %% compute rotation of the world with respect to the camera (R cam -> world)
    % in this case the world is the vertical facade (face ?????)
    R = [r1, r3, r3];

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


    %% Display orientation and position from vertical facace (face ?????)
%     if debug
        figure('Name', 'Camera location')
        plotCamera('Location', cameraPosition, 'Orientation', cameraRotation.', 'Size', 20);
        hold on;

        pcshow([[upper_left_point; lower_left_point; lower_right_point; upper_right_point], ...
                zeros(size([upper_left_point; lower_left_point; lower_right_point; upper_right_point],1), 1)], ...
                'red','VerticalAxisDir', 'up', 'MarkerSize', 20);
    
        patch([upper_left_point(1); lower_left_point(1); lower_right_point(1); upper_right_point(1)], ...
              [upper_left_point(2); lower_left_point(2); lower_right_point(2); upper_right_point(2)], ...
              'green');
    
        patch([upper_left_point(1); lower_left_point(1); lower_left_point(1); upper_left_point(1)], ...
              [upper_left_point(2); lower_left_point(2); lower_left_point(2); upper_left_point(2)], ...
              [ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, 0, 0], ...
              'red');
    
        patch([upper_right_point(1); lower_right_point(1); lower_right_point(1); upper_right_point(1)], ...
              [upper_right_point(2); lower_right_point(2); lower_right_point(2); upper_right_point(2)], ...
              [ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, 0, 0], ...
              'red');

        xlabel('X')
        ylabel('Y')
        zlabel('Z')
    
        saveas(gcf, 'images/camera_location.png')

%     end


%         figure('Name', 'Camera location')
%         plotCamera('Location', cameraPosition, 'Orientation', cameraRotation.', 'Size', 20);
%         hold on;
% 
%         pcshow([[upper_left_point; lower_left_point; lower_right_point; upper_right_point], ...
%                 zeros(size([upper_left_point; lower_left_point; lower_right_point; upper_right_point],1), 1)], ...
%                 'red','VerticalAxisDir', 'up', 'MarkerSize', 20);
%     
%         patch([upper_left_point(1); lower_left_point(1); lower_right_point(1); upper_right_point(1)], ...
%               zeros(size([upper_left_point; lower_left_point; lower_right_point; upper_right_point],1), 1),...
%               [upper_left_point(2); lower_left_point(2); lower_right_point(2); upper_right_point(2)], ...
%               'green');
%     
%         patch([upper_left_point(1); lower_left_point(1); lower_left_point(1); upper_left_point(1)], ...
%               [ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, 0, 0],...
%               [upper_left_point(2); lower_left_point(2); lower_left_point(2); upper_left_point(2)], ...
%               'red');
%     
%         patch([upper_right_point(1); lower_right_point(1); lower_right_point(1); upper_right_point(1)], ...
%               [ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, ratio_f2_f3*LENGTH_LONGSIDE/ratio_f3_height_f3_length, 0, 0],...
%               [upper_right_point(2); lower_right_point(2); lower_right_point(2); upper_right_point(2)], ...
%               'red');
% 
%         xlabel('X')
%         ylabel('Y')
%         zlabel('Z')
%     
%         saveas(gcf, 'images/camera_location.png')


end

