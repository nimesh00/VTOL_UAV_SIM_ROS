clear, clc

%% forming cl and cd curves from the example data in H.Glaubert (Pg: 214)
lmbda1 = [-0.012, 0.204, 0.410, 0.610, 0.780, 0.964, 1.136];
lmbda2 = [0.036, 0.088, 0.132, 0.164, 0.178, 0.186, 0.180];
theta_ = 24 * pi / 180;
alpha_ = 4:2:16;
alpha_ = alpha_ * pi / 180;
phi_ = theta_ - alpha_;
cl_f = lmbda1 .* cos(phi_) + lmbda2 .* sin(phi_);
cd_f = lmbda2 .* cos(phi_) - lmbda1 .* sin(phi_);
cd_poly = @(x) interp1(alpha_, cd_f, x);
cl_poly = @(x) interp1(alpha_, cl_f, x);

%% calculating cl and cd
%cl = cl_poly(alpha_);
%cd = cd_poly(alpha_);

RPM = 1:10:25000;
omega = 2 * pi .* RPM / 60;
D_inch = 10;
pitch_inch = 4.5;
pitch = 0.0254 * pitch_inch; % m (4.5")
R = 0.0254 * (D_inch / 2); % m (D = 10")
rho = 1.225; % air density
c = 0.02; % chord length (2cm) ASSUMED
N = 2;

r = (0.1 * R):0.01:R;
theta = atan(pitch / (2 * pi * 0.7 * R));
s_r = (N * c ./ (2 * pi .* r));
for i = 1:length(r)
  alpha = 4:2:16;
  alpha = alpha * pi / 180;
  phi = theta - alpha;
  cl = cl_poly(alpha);
  cd = cd_poly(alpha);
  s = s_r(i);
  rR = r(i) / R;
  lambda1 = (cl .* cos(phi) - cd .* sin(phi));
  lambda2 = (cl .* sin(phi) + cd .* cos(phi));
  a = (.5 * s .* lambda1) ./ (1 - cos(2 .* phi) - .5 .* s .* lambda1);
  a_ = (.5 * s .* lambda2) ./ (sin(2 .* phi) + 0.5 * s .* lambda2);
  J = pi * rR .* ((1 - a_) ./ (1 + a)) .* tan(phi);
  thrust_coefficient = 0.25 * pi ^ 3 * rR ^ 3 * s .* (1 - a_) .^ 2 .* lambda1 .* sec(phi) .^ 2;
  torque_coefficient = 0.125 * pi ^ 3 * rR ^ 4 * s .* (1 - a_) .^ 2 .* lambda2 .* sec(phi) .^ 2;
  if (i == int16(0.1 * length(r)))
    plot(J, thrust_coefficient, 'g')
    hold on
    plot(J, 10 * torque_coefficient, 'r')
  end
end