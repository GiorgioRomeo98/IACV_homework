function image_absolute_conic = get_image_absolute_conic(l_inf, vp, vp1, vp2, H)
    % GET_IMAGE_ABSOLUTE_CONIC returns the image of the absolute conic
    %
    % output
    % image_absolute_conic: image of the absolute conic
    % 
    % input
    % l_infs: image of the line at the infinite
    % vp: vertical vanishing point
    % vp1, vp2: first and second horizontal vanishing point (computed for
    % the affine reconstruction)
    % H: homography for the shape reconstruction from the original image


    % matrix parametrization
    syms a b c d;
    omega = [a 0 b; 0 1 c; b c d];
    
    X = []; % should be nxm matrix (n is the number of constraints (at least 4), m is 4 (unknowns)
    Y = []; % should be n matrix of target values
    
    %% add constraints on l_inf and vp
    % [l_inf] x W vp = 0 (2 constraints)

    % first compute the element of l_inf
    l1 = l_inf(1,1);
    l2 = l_inf(2,1);
    l3 = l_inf(3,1);
  
    % vector product matrix
    lx = [0 -l3 l2; l3 0 -l1; -l2 l1 0];

    eqn = [lx(1,:)*omega*vp == 0, lx(2,:)*omega*vp == 0];
    
    
    % cast equations into matrix form
    [A,y] = equationsToMatrix(eqn,[a,b,c,d]);
    % concatenate matrices
    X = [X;double(A)];
    Y = [Y;double(y)];


    %% add constraints on vanishing points
    % vp1' W vp2 = 0
    eqn = [vp1.' * omega * vp2 == 0];
        
    % cast equations into matrix form
    [A,y] = equationsToMatrix(eqn,[a,b,c,d]);

    % concatenate matrices
    X = [X;double(A)];
    Y = [Y;double(y)];
    
    %% add constraints on homography
    % scaling factor needed in order to get an homogeneous matrix
    % get columns
    h1 = H(:,1);
    h2 = H(:,2);

    % first constraint: h1' w h2 = 0
    eq1 = h1.' * omega * h2 == 0;
    % second constraint h1'w h1 = h2' w h2
    eq2 = h1.' * omega * h1 == h2.' * omega * h2;

    [A,y] = equationsToMatrix([eq1,eq2],[a,b,c,d]);
    A = double(A);
    y = double(y);

    % concatenate matrices
    X = [X;A];
    Y = [Y;y];
    
    %% solve the system and find the image of the absolute conic
    W = linsolve(X,Y);
    
    % W = X.'*X \ (X.'*Y)
    % image of the absolute conic
    image_absolute_conic = double([W(1,1) 0 W(2,1); 0 1 W(3,1); W(2,1) W(3,1) W(4,1)]);

    

end


