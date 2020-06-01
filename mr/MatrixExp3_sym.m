function  R = MatrixExp3_sym(so3mat,theta)


omgmat = so3mat;
R = simplify(eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat);

end