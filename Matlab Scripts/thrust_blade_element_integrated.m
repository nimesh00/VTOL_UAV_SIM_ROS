clear all, close all, clc

cl = 0.1;
cd = 0.05;

RPM = 1:10:25000;
omega = 2 * pi .* RPM / 60;
D_inch = 50;
pitch_inch = 4.5;
pitch = 0.0254 * pitch_inch; % m (4.5")
R = 0.0254 * (D_inch / 2); % m (D = 10")
rho = 1.225; % air density
l = 0.02; % chord length (2cm) ASSUMED

Vac = 0;

% integrated using wolfram integrator
%integral = @(x) (cl * log(sin(x / 2)) - cl * log(cos(x / 2)) + cd * csc(x));
integral = @(x) (cd / 3) * csc(x) ^ 3 - 0.5 * cl * (-log(sin(x / 2)) + log(cos(x / 2)) + cot(x) * csc(x));

r1 = 0.1 * R;
r2 = 0.8 * R;

%theta1 = atan(pitch / (2 * pi * r1));
%theta2 = atan(pitch / (2 * pi * r2));
%
%f_thrust = integral(theta2) - integral(theta1);

%cx = ((pitch .* omega) / (2 * pi) - Vac);

%thrust_per_blade = (pitch * rho / (4 * pi)) * cx .^ 2 * f_thrust;

a = 200;

%cu = omega .* R ^ 2 / (2 * sqrt(a * (2 + a))) - Vac;
cu = omega .* pitch;
cs = (1 + 2 * a) * cu;
c = (1 + a) * cu;

thrust = 0.5 * rho * (pi * R ^ 2) .* (cs .^ 2 - cu .^ 2);
torque = 0.5 * rho * (pi * R ^ 2) .* cu .* (cs .^ 2 - cu .^ 2) ./ omega;
%thrust_per_blade = 1.55e-6 .* omega .^ 2;

plot(RPM, torque, 'k', 'LineWidth', 2)
hold on
%plot(RPM, thrust, 'g', 'LineWidth', 2);
%hold on
speed = [6400, 10080, 12070, 13730, 15100, 16320, 17350, 18350, 19210, 20080];
thrst = [0.062, 0.162, 0.236, 0.311, 0.374, 0.439, 0.490, 0.548, 0.611, 0.712] * 10;
%figure
plot(speed, thrst, 'r')
hold on
speed2 = [7220, 10790, 13030, 14720, 16180, 17150, 18640, 19270, 20270, 21060, 21840, 22590, 23210, 23920, 24560];
thrst2 = [0.076, 0.183, 0.283, 0.352, 0.426, 0.497, 0.560, 0.628, 0.692, 0.754, 0.812, 0.878, 0.936, 0.997, 1.024] * 10;
plot(speed2, thrst2, 'b')
