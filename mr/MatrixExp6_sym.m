function T = MatrixExp6_sym(se3mat,theta)

omgtheta = so3ToVec(se3mat(1: 3, 1: 3));

omgmat = se3mat(1: 3, 1: 3); 
T = [MatrixExp3_sym(se3mat(1: 3, 1: 3),theta), ...
     (eye(3) * theta + (1 - cos(theta)) * omgmat ...
      + (theta - sin(theta)) * omgmat * omgmat) ...
        * se3mat(1: 3, 4);
     0, 0, 0, 1];

end