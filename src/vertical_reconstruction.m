function  img_vertical_rectification = vertical_reconstruction(img, K, debug)
    % VERTICAL_RECONSTRUCTION performs the rectification of a vertical facade
    %
    % output
    % img_vertical_rectification: image obtained from the image in input
    % after performing rectification of the vertical facade
    %
    %input
    %
    img_vertical_rectification = 0;


    %% comput the image of the absolute conic through the calibration matrix K
    imIAC = inv(K * K');
    imIAC = imIAC ./ norm(imIAC);

    display(imIAC);
    sos = inv(K * K.');
    display(sos);


end

