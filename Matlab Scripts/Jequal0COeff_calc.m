clear, clc
%A = 0.0493; % Aspect Ratio
s = 0.1;
rR = 0.7;
theta = 11 * pi / 180;
alpha = 4:2:16;
alpha = alpha * pi / 180;
phi = theta - alpha;
%cl = 2 * pi .* sin(alpha);
lmbda1 = [-0.012, 0.204, 0.410, 0.610, 0.780, 0.964, 1.136];
lmbda2 = [0.036, 0.088, 0.132, 0.164, 0.178, 0.186, 0.180];
%cd_f = (cl .* cos(phi) - lmbda) ./ sin(phi)
%cd_poly = polyfit(alpha, cd_f, length(alpha));
cl_f = lmbda1 .* cos(phi) + lmbda2 .* sin(phi);
cd_f = lmbda2 .* cos(phi) - lmbda1 .* sin(phi);
cd_poly = @(x) interp1(alpha, cd_f, x);
cl_poly = @(x) interp1(alpha, cl_f, x);
cl = cl_poly(alpha);
cd = cd_poly(alpha);
%cl = 0.11 .* sin(alpha);
%cl = 2 .* alpha;
%cd = (cl .^ 2) .* cot(phi);
%cd = cl .* tan(phi);
%cd = 0.008 - 0.003 .* cl + 0.01 .* cl .^ 2;
%cd = (cl .^ 2) / (pi * A);
lambda1 = (cl .* cos(phi) - cd .* sin(phi))
lambda2 = (cl .* sin(phi) + cd .* cos(phi))
a = (.5 * s .* lambda1) ./ (1 - cos(2 .* phi) - .5 .* s .* lambda1)
a_ = (.5 * s .* lambda2) ./ (sin(2 .* phi) + 0.5 * s .* lambda2)
J = pi * rR .* ((1 - a_) ./ (1 + a)) .* tan(phi)

thrust_coefficient = 0.25 * pi ^ 3 * rR ^ 3 * s .* (1 - a_) .^ 2 .* lambda1 .* sec(phi) .^ 2
torque_coefficient = 0.125 * pi ^ 3 * rR ^ 4 * s .* (1 - a_) .^ 2 .* lambda2 .* sec(phi) .^ 2
plot(J, thrust_coefficient, 'g')
hold on
plot(J, 10 * torque_coefficient, 'r')