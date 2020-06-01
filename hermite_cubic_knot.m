function [p,dp,ddp, p_grad, dp_grad, ddp_grad] = hermite_cubic_knot(cur_t, t0, x0, dx0, t1, x1, dx1)
% x0 dx0 x1 dx1 should all be size Nx1

% syms cur_t t0 t1 x0 dx0 x1 dx1 real
% t = (cur_t-t0)/(t1-t0);
% control_mtx = [ 2 -2  1  1;
%                -3  3 -2 -1;
%                 0  0  1  0;
%                 1  0  0  0];
% vec = [x0;
%        x1;
%        dx0;
%        dx1];
% coeffi = control_mtx*vec;
% p =   [  t^3 t^2 t 1]*coeffi;   
% dp =  [3*t^2 2*t 1 0]*coeffi;   
% ddp = [  6*t   2 0 0]*coeffi;   

p = x1*((3*(cur_t - t0)^2)/(t0 - t1)^2 + (2*(cur_t - t0)^3)/(t0 - t1)^3) - x0*((3*(cur_t - t0)^2)/(t0 - t1)^2 + (2*(cur_t - t0)^3)/(t0 - t1)^3 - 1) + dx0*(t0 - t1)*((2*(cur_t - t0)^2)/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3 + (cur_t - t0)/(t0 - t1)) + dx1*((cur_t - t0)^2/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3)*(t0 - t1);
dp = x1*((3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (6*(cur_t - t0)^2)/(t0 - t1)^3) - x0*((3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (6*(cur_t - t0)^2)/(t0 - t1)^3) + dx1*((2*cur_t - 2*t0)/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3)*(t0 - t1) + dx0*(t0 - t1)*(1/(t0 - t1) + (2*(2*cur_t - 2*t0))/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3);
ddp = x1*(6/(t0 - t1)^2 + (6*(2*cur_t - 2*t0))/(t0 - t1)^3) - x0*(6/(t0 - t1)^2 + (6*(2*cur_t - 2*t0))/(t0 - t1)^3) + dx1*(2/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3)*(t0 - t1) + dx0*(4/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3)*(t0 - t1);


% return gradient with respect to t0 t1 x0 dx0 x1 dx1
if nargout > 3
    p_grad = spalloc(size(x0,1), 2+4*size(x0,1), 6*size(x0,1));
    for i = 1:size(x0,1)
        % very sensitive
        p_grad(:,1) = x0*((3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (12*(cur_t - t0)^2)/(t0 - t1)^3 + (6*(cur_t - t0)^3)/(t0 - t1)^4) - x1*((3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (12*(cur_t - t0)^2)/(t0 - t1)^3 + (6*(cur_t - t0)^3)/(t0 - t1)^4) + dx0*((2*(cur_t - t0)^2)/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3 + (cur_t - t0)/(t0 - t1)) + dx1*((cur_t - t0)^2/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3) - dx1*(t0 - t1)*((2*cur_t - 2*t0)/(t0 - t1)^2 + (5*(cur_t - t0)^2)/(t0 - t1)^3 + (3*(cur_t - t0)^3)/(t0 - t1)^4) - dx0*(t0 - t1)*(1/(t0 - t1) + (2*(2*cur_t - 2*t0))/(t0 - t1)^2 + (7*(cur_t - t0)^2)/(t0 - t1)^3 + (3*(cur_t - t0)^3)/(t0 - t1)^4 + (cur_t - t0)/(t0 - t1)^2);
        p_grad(i,1+i) = 1 - (2*(cur_t - t0)^3)/(t0 - t1)^3 - (3*(cur_t - t0)^2)/(t0 - t1)^2;
        p_grad(i,1+i+size(x0,1)) = (t0 - t1)*((2*(cur_t - t0)^2)/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3 + (cur_t - t0)/(t0 - t1));
        p_grad(:,2+2*size(x0,1)) = x1*((6*(cur_t - t0)^2)/(t0 - t1)^3 + (6*(cur_t - t0)^3)/(t0 - t1)^4) - dx1*((cur_t - t0)^2/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3) - x0*((6*(cur_t - t0)^2)/(t0 - t1)^3 + (6*(cur_t - t0)^3)/(t0 - t1)^4) - dx0*((2*(cur_t - t0)^2)/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3 + (cur_t - t0)/(t0 - t1)) + dx0*(t0 - t1)*((4*(cur_t - t0)^2)/(t0 - t1)^3 + (3*(cur_t - t0)^3)/(t0 - t1)^4 + (cur_t - t0)/(t0 - t1)^2) + dx1*((2*(cur_t - t0)^2)/(t0 - t1)^3 + (3*(cur_t - t0)^3)/(t0 - t1)^4)*(t0 - t1);
        p_grad(i,2+i+2*size(x0,1)) = (3*(cur_t - t0)^2)/(t0 - t1)^2 + (2*(cur_t - t0)^3)/(t0 - t1)^3;
        p_grad(i,2+i+3*size(x0,1)) = ((cur_t - t0)^2/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3)*(t0 - t1);
    end
    dp_grad = spalloc(size(x0,1), 2+4*size(x0,1), 6*size(x0,1));
    for i = 1:size(x0,1)
        dp_grad(:,1) = x0*(6/(t0 - t1)^2 + (12*(2*cur_t - 2*t0))/(t0 - t1)^3 + (18*(cur_t - t0)^2)/(t0 - t1)^4) - x1*(6/(t0 - t1)^2 + (12*(2*cur_t - 2*t0))/(t0 - t1)^3 + (18*(cur_t - t0)^2)/(t0 - t1)^4) + dx1*((2*cur_t - 2*t0)/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3) + dx0*(1/(t0 - t1) + (2*(2*cur_t - 2*t0))/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3) - dx1*(t0 - t1)*(2/(t0 - t1)^2 + (5*(2*cur_t - 2*t0))/(t0 - t1)^3 + (9*(cur_t - t0)^2)/(t0 - t1)^4) - dx0*(t0 - t1)*(5/(t0 - t1)^2 + (7*(2*cur_t - 2*t0))/(t0 - t1)^3 + (9*(cur_t - t0)^2)/(t0 - t1)^4);
        dp_grad(i,1+i) = - (3*(2*cur_t - 2*t0))/(t0 - t1)^2 - (6*(cur_t - t0)^2)/(t0 - t1)^3;
        dp_grad(i,1+i+size(x0,1)) = (t0 - t1)*(1/(t0 - t1) + (2*(2*cur_t - 2*t0))/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3);
        dp_grad(:,2+2*size(x0,1)) = x1*((6*(2*cur_t - 2*t0))/(t0 - t1)^3 + (18*(cur_t - t0)^2)/(t0 - t1)^4) - dx0*(1/(t0 - t1) + (2*(2*cur_t - 2*t0))/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3) - x0*((6*(2*cur_t - 2*t0))/(t0 - t1)^3 + (18*(cur_t - t0)^2)/(t0 - t1)^4) - dx1*((2*cur_t - 2*t0)/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3) + dx1*((2*(2*cur_t - 2*t0))/(t0 - t1)^3 + (9*(cur_t - t0)^2)/(t0 - t1)^4)*(t0 - t1) + dx0*(t0 - t1)*(1/(t0 - t1)^2 + (4*(2*cur_t - 2*t0))/(t0 - t1)^3 + (9*(cur_t - t0)^2)/(t0 - t1)^4);
        dp_grad(i,2+i+2*size(x0,1)) = (3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (6*(cur_t - t0)^2)/(t0 - t1)^3;
        dp_grad(i,2+i+3*size(x0,1)) = ((2*cur_t - 2*t0)/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3)*(t0 - t1);
    end
    
    ddp_grad = spalloc(size(x0,1), 2+4*size(x0,1), 6*size(x0,1));
    for i = 1:size(x0,1)
        ddp_grad(:,1) = dx1*(2/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3) + dx0*(4/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3) + x0*(24/(t0 - t1)^3 + (18*(2*cur_t - 2*t0))/(t0 - t1)^4) - x1*(24/(t0 - t1)^3 + (18*(2*cur_t - 2*t0))/(t0 - t1)^4) - dx1*(10/(t0 - t1)^3 + (9*(2*cur_t - 2*t0))/(t0 - t1)^4)*(t0 - t1) - dx0*(14/(t0 - t1)^3 + (9*(2*cur_t - 2*t0))/(t0 - t1)^4)*(t0 - t1);
        ddp_grad(i,1+i) = - 6/(t0 - t1)^2 - (6*(2*cur_t - 2*t0))/(t0 - t1)^3;
        ddp_grad(i,1+i+size(x0,1)) = (4/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3)*(t0 - t1);
        ddp_grad(:,2+2*size(x0,1)) = x1*(12/(t0 - t1)^3 + (18*(2*cur_t - 2*t0))/(t0 - t1)^4) - dx0*(4/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3) - x0*(12/(t0 - t1)^3 + (18*(2*cur_t - 2*t0))/(t0 - t1)^4) - dx1*(2/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3) + dx1*(4/(t0 - t1)^3 + (9*(2*cur_t - 2*t0))/(t0 - t1)^4)*(t0 - t1) + dx0*(8/(t0 - t1)^3 + (9*(2*cur_t - 2*t0))/(t0 - t1)^4)*(t0 - t1);
        ddp_grad(i,2+i+2*size(x0,1)) = 6/(t0 - t1)^2 + (6*(2*cur_t - 2*t0))/(t0 - t1)^3;    % x1
        ddp_grad(i,2+i+3*size(x0,1)) = (2/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3)*(t0 - t1);
    end
    
end

end