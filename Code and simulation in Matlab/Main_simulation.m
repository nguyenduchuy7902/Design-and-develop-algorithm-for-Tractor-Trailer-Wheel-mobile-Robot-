clc
clear

%% Simulation detail
stop_time = 10;
sampling_interval = 0.00001;
num_data = int32(stop_time / sampling_interval) + 1;
t = 0 : sampling_interval : stop_time;


%% System parameter

% Model parameter
m0 = 1; % 8<m0<12
m1 = 0.4; % 5<m1<8
M = m0 + m1; % 13<M<20
I1 = 0.005; % 3<I1<7
I0 = 0.002; % 2<I0<6
d = 0.15; % 0.3<d<0.7
a0 = 0.03; %0.1<a0<0.25
a1 = 0.03; %0.1<a0<0.25
r = 0.03; %0.08<r<0.12
b = 0.06; %0.1<b<0.25

I_theta1 = m1 * a1^2 + m0 * d^2 + I1;
I_theta0 = m0 * a0^2 + I0;

parameters = zeros(5,1);
parameters(1, 1) = r * I_theta1 / d^2;
parameters(2, 1) = r * a0 * m0;
parameters(3, 1) = (m0 + m1) * r;
parameters(4, 1) = r * a0 * m0 / b;
parameters(5, 1) = r * I_theta0 / b;

% Controller parameter
Lamda = 0.1*diag([1;1;1;1;1]);
k1 = 0.4;
k2 = 30;
Gamma = 5*diag([1;1]);
k = 100; 


%% Variable declaration
% State variable 
x = zeros(1, num_data); 
y = zeros(1, num_data);
theta_1 = zeros(1, num_data);
theta_0 = zeros(1, num_data);
u_1 = zeros(1, num_data);
u_2 = zeros(1, num_data);

% Desired trajectory

%{
a = 0.1;
alpha = 0.5;
R = 2;
eta = 6;
d_1 = 1; 

x_r = a * (R + cos(eta * alpha * t)) .* cos(alpha * t);
y_r = a * (R + cos(eta * alpha * t)) .* sin(alpha * t);
derivative_x_r = discrete_derivative(x_r, sampling_interval);
derivative_y_r = discrete_derivative(y_r, sampling_interval);
%}

u_1r = zeros(1, num_data) + 1.5;
%{
 u_1r = sqrt(derivative_x_r .^ 2 + derivative_y_r .^ 2);
%}

u_2r = zeros(1, num_data) + 1.2;

%{
u_2r = (d_1 * (discrete_derivative(discrete_derivative(u_1r, sampling_interval), sampling_interval) .* derivative_x_r ...
    + discrete_derivative(u_1r, sampling_interval) .* discrete_derivative(derivative_x_r, sampling_interval) ...
    - discrete_derivative(discrete_derivative(derivative_x_r, sampling_interval), sampling_interval) .* u_1r ...
    - discrete_derivative(derivative_x_r, sampling_interval) .* discrete_derivative(u_1r, sampling_interval)) .* derivative_y_r .* (u_1r .^ 2) ...
    - d_1 * (discrete_derivative(u_1r, sampling_interval) .* derivative_x_r ...
    - discrete_derivative(derivative_x_r, sampling_interval) .* u_1r) .* (discrete_derivative(derivative_y_r, sampling_interval) .* (u_1r .^ 2) ...
    + 2 * derivative_y_r .* u_1r .* discrete_derivative(u_1r, sampling_interval))) ...
    ./ ((derivative_y_r .^ 2) .* (u_1r .^ 4) + (d_1^2) * ((discrete_derivative(u_1r, sampling_interval) .* derivative_x_r - discrete_derivative(derivative_x_r, sampling_interval) .* (u_1r .^ 2))) .^ 2);
%}

x_r = zeros(1, num_data);
y_r = zeros(1, num_data);
theta_1r = zeros(1, num_data);
theta_0r = zeros(1, num_data);

derivative_x_r = zeros(1, num_data);
derivative_y_r = zeros(1, num_data);
derivative_theta_1r = zeros(1, num_data);
derivative_theta_0r = zeros(1, num_data);

x_r(1) = 2;
y_r(1) = 0;
theta_1r(1) = 0;
theta_0r(1) = 0;

derivative_x_r(1) = u_1r(1) * cos(theta_1r(1));
derivative_y_r(1) = u_1r(1) * sin(theta_1r(1));
derivative_theta_1r(1) = u_1r(1) / d * tan(theta_0r(1) - theta_1r(1));
derivative_theta_0r(1) = u_2r(1);

x_r(2) = derivative_x_r(1) * sampling_interval + x_r(1);
y_r(2) = derivative_y_r(1) * sampling_interval + y_r(1);
theta_1r(2) = derivative_theta_1r(1) * sampling_interval + theta_1r(1);
theta_0r(2) = derivative_theta_0r(1) * sampling_interval + theta_0r(1);

for i = 2 : (num_data)
    derivative_x_r(i) = u_1r(i) * cos(theta_1r(i));
    derivative_y_r(i) = u_1r(i) * sin(theta_1r(i));
    derivative_theta_1r(i) = u_1r(i) / d * tan(theta_0r(i) - theta_1r(i));
    derivative_theta_0r(i) = u_2r(i);
    
    if i == num_data
        break
    end
    
    x_r(i + 1) = derivative_x_r(i) * sampling_interval + x_r(i);
    y_r(i + 1) = derivative_y_r(i) * sampling_interval + y_r(i);
    theta_1r(i + 1) = derivative_theta_1r(i) * sampling_interval + theta_1r(i);
    theta_0r(i + 1) = derivative_theta_0r(i) * sampling_interval + theta_0r(i);
end

% Error
e_x = zeros(1, num_data);
e_y = zeros(1, num_data);
e_theta_1 = zeros(1, num_data);
e_theta_0 = zeros(1, num_data);

% Z_error
z_1 = zeros(1, num_data);
z_2 = zeros(1, num_data);
z_3 = zeros(1, num_data);
z_4 = zeros(1, num_data);

% Virtual control
u_1c = zeros(1, num_data);
u_2c = zeros(1, num_data);

% Torque tau
tau_1 = zeros(1, num_data);
tau_2 = zeros(1, num_data);

% Parameters estimation
parameters_est = zeros(5, num_data);


%% Inital calculation
% initial value
x(1) = 1.5;
y(1) = 0.5;
u_1(1) = 0;
u_2(1) = 0;
theta_1(1) = -0.1;
theta_0(1) = -0.2;
parameters_est(:, 1) = [0.02;0.02;0.3;0.03;0.01]';

% Errors
e_x(1) = cos(theta_1r(1)) * (x(1) - x_r(1)) + sin(theta_1r(1)) * (y(1) - y_r(1));
e_y(1) = - sin(theta_1r(1)) * (x(1) - x_r(1)) + cos(theta_1r(1)) * (y(1) - y_r(1));
e_theta_1(1) = theta_1(1) - theta_1r(1);
e_theta_0(1) = theta_0(1) - theta_0r(1);

% Z - errors
z_1(1) = e_x(1);
z_2(1) = e_y(1);
z_3(1) = tan(e_theta_1(1));
z_4(1) = (tan(theta_0(1) - theta_1(1)) - tan(theta_0r(1) - theta_1r(1)) * cos(e_theta_1(1))) / ( d * (cos(e_theta_1(1)))^3) + e_y(1);

beta_1 = (- tan(theta_0r(1) - theta_1r(1))) / (d^2 * (cos(e_theta_1(1)))^3 * (cos(theta_0(1) - theta_1(1)))^2) + sin(e_theta_1(1)) + ... 
        3 * ((tan(theta_0(1) - theta_1(1)))^2 * sin(e_theta_1(1))) / (d^2 * (cos(e_theta_1(1)))^4) - ...
        2 * ((tan(theta_0r(1) - theta_1r(1)))^2 * sin(e_theta_1(1))) / (d^2 * (cos(e_theta_1(1)))^3);
beta_2 = d^(-1) * (cos(e_theta_1(1)))^(-3) * (cos(theta_0(1) - theta_1(1)))^(-2);
f_2 = - derivative_theta_1r(1) * ((3 * tan(theta_0(1) - theta_1(1)) * sin(e_theta_1(1))) / (d * (cos(e_theta_1(1)))^4) + e_x(1) ...
    - (2 * tan(theta_0r(1) - theta_1r(1)) * sin(e_theta_1(1))) / (d * (cos(e_theta_1(1)))^3)) ...
    - (derivative_theta_0r(1) - derivative_theta_1r(1)) / (d * (cos(theta_0r(1) - theta_1r(1)))^2 * (cos(e_theta_1(1)))^2);

Phi = [cos(e_theta_1(1)) 0; beta_1 beta_2];
f = [- u_1r(1); f_2];
Omega_1 = z_3(1) * (z_4(1) + (tan(theta_0r(1) - theta_1r(1))) / d * (1 + z_3(1)^2)) + z_1(1);
Omega_2 = 1 / k * z_4(1);

% Calculation of virtual control
w_1c = - k2 * z_3(1) * (z_4(1) + (tan(theta_0r(1) - theta_1r(1))) / d * (1 + z_3(1)^2)) - k2 * z_1(1);
w_2c = k * (- z_3(1) * u_1r(1) - k1 * z_4(1));
Omega = [Omega_1; Omega_2];

omega_c = [w_1c; w_2c];
u_c = Phi \ (omega_c - f);
u = [u_1(1) ; u_2(1)];
u_1c(1) = u_c(1);
u_2c(1) = u_c(2);

% Calculation of control torque (tau)
Y = zeros(2, 5);

derivative_u_1c = 0;
derivative_u_2c = 0;

derivative_theta_1 = u_1(1) / d * tan(theta_0(1) - theta_1(1));
derivative_theta_0 = u_2(1);

Xi = (derivative_theta_0 - derivative_theta_1) * tan(theta_0(1) - theta_1(1)) * u_1c(1) / ((cos(theta_0(1) - theta_1(1)))^2) + ((tan(theta_0(1) - theta_1(1)))^2) * derivative_u_2c;

Y(1, 1) = Xi;
Y(1, 2) = derivative_theta_0 * u_2(1) / cos(theta_0(1) - theta_1(1));
Y(1, 3) = derivative_u_1c;
Y(2, 4) = derivative_theta_0 * u_1(1) / cos(theta_0(1) - theta_1(1));
Y(2, 5) = derivative_u_2c;

derivative_parameter_est = - Lamda \ Y' * (u - u_c);
parameters_est(:, 2) = parameters_est(:, 1) + derivative_parameter_est * sampling_interval;

tau = Y * parameters_est(:, 2) - Gamma * (u - u_c) - Phi' * Omega;
tau_1(2) = tau(1);
tau_2(2) = tau(2);

% 2nd state
% Velocity
u_1(2) = u_1(1);
u_2(2) = u_2(1);

% Coordinates
derivative_x = u_1(1) * cos(theta_1(1));
derivative_y = u_1(1) * sin(theta_1(1));
derivative_theta_1 = u_1(1) / d * tan(theta_0(1) - theta_1(1));
derivative_theta_0 = u_2(1);

x(2) = x(1) + derivative_x * sampling_interval;
y(2) = y(1) + derivative_y * sampling_interval;
theta_1(2) = theta_1(1) + derivative_theta_1 * sampling_interval;
theta_0(2) = theta_0(1) + derivative_theta_0 * sampling_interval;

% 3rd State
% Velocity
M_2 = zeros(2, 2);
M_2(1, 1) = parameters(3, 1) + parameters(1, 1) * ((tan(theta_0(1) - theta_1(1)))^2);
M_2(2, 2) = parameters(5, 1);

C_2 = zeros(2, 2);
derivative_theta_1 = u_1(2) / d * tan(theta_0(2) - theta_1(2));
derivative_theta_0 = u_2(2);

C_2(1, 1) = parameters(1, 1) * (derivative_theta_0 - derivative_theta_1) * tan(theta_0(2) - theta_1(2)) / ((cos(theta_0(2) - theta_1(2)))^2);
C_2(1, 2) = - parameters(2, 1) * derivative_theta_0 / cos(theta_0(2) - theta_1(2));
C_2(2, 1) = parameters(4, 1) * derivative_theta_0 / cos(theta_0(2) - theta_1(2));

derivative_u = M_2 \ (tau - C_2 * u);

u_1(3) = u_1(2) + derivative_u(1) * sampling_interval;
u_2(3) = u_2(2) + derivative_u(2) * sampling_interval;

% Coordinates
derivative_x = u_1(2) * cos(theta_1(2));
derivative_y = u_1(2) * sin(theta_1(2));
derivative_theta_1 = u_1(2) / d * tan(theta_0(2) - theta_1(2));
derivative_theta_0 = u_2(2);

x(3) = x(2) + derivative_x * sampling_interval;
y(3) = y(2) + derivative_y * sampling_interval;
theta_1(3) = theta_1(2) + derivative_theta_1 * sampling_interval;
theta_0(3) = theta_0(2) + derivative_theta_0 * sampling_interval;


%% Next state calculation

for i = 2 : num_data
    % Errors
    e_x(i) = cos(theta_1r(i)) * (x(i) - x_r(i)) + sin(theta_1r(i)) * (y(i) - y_r(i));
    e_y(i) = - sin(theta_1r(i)) * (x(i) - x_r(i)) + cos(theta_1r(i)) * (y(i) - y_r(i));
    e_theta_1(i) = theta_1(i) - theta_1r(i);
    e_theta_0(i) = theta_0(i) - theta_0r(i);

    % Z - errors
    z_1(i) = e_x(i);
    z_2(i) = e_y(i);
    z_3(i) = tan(e_theta_1(i));
    z_4(i) = (tan(theta_0(i) - theta_1(i)) - tan(theta_0r(i) - theta_1r(i)) * cos(e_theta_1(i))) / ( d * (cos(e_theta_1(i)))^3) + e_y(i);

    beta_1 = (- tan(theta_0r(i) - theta_1r(i))) / (d^2 * (cos(e_theta_1(i)))^3 * (cos(theta_0(i) - theta_1(i)))^2) + sin(e_theta_1(i)) + ... 
        3 * ((tan(theta_0(i) - theta_1(i)))^2 * sin(e_theta_1(i))) / (d^2 * (cos(e_theta_1(i)))^4) - ...
        2 * ((tan(theta_0r(i) - theta_1r(i)))^2 * sin(e_theta_1(i))) / (d^2 * (cos(e_theta_1(i)))^3);
    beta_2 = d^(-1) * (cos(e_theta_1(i)))^(-3) * (cos(theta_0(i) - theta_1(i)))^(-2);
    f_2 = - derivative_theta_1r(i) * ((3 * tan(theta_0(i) - theta_1(i)) * sin(e_theta_1(i))) / (d * (cos(e_theta_1(i)))^4) + e_x(i) ...
        - (2 * tan(theta_0r(i) - theta_1r(i)) * sin(e_theta_1(i))) / (d * (cos(e_theta_1(i)))^3)) ...
        - (derivative_theta_0r(i) - derivative_theta_1r(i)) / (d * (cos(theta_0r(i) - theta_1r(i)))^2 * (cos(e_theta_1(i)))^2);

    Phi = [cos(e_theta_1(i)) 0; beta_1 beta_2];
    f = [- u_1r(i); f_2];
    Omega_1 = z_3(i) * (z_4(i) + (tan(theta_0r(i) - theta_1r(i))) / d * (1 + z_3(i)^2)) + z_1(i);
    Omega_2 = 1 / k * z_4(i);

    % Calculation of virtual control
    w_1c = - k2 * z_3(i) * (z_4(i) + (tan(theta_0r(i) - theta_1r(i))) / d * (1 + z_3(i)^2)) - k2 * z_1(i);
    w_2c = k * (-z_3(i) * u_1r(i) - k1 * z_4(i));
    Omega = [Omega_1; Omega_2];

    omega_c = [w_1c; w_2c];
    u_c = Phi \ (omega_c - f);
    u = [u_1(i) ; u_2(i)];
    u_1c(i) = u_c(1);
    u_2c(i) = u_c(2);
    
    if i <= num_data - 1 % Stop excess calculation of torque
    % Calculation of control torque (tau)
    derivative_u_1c = (u_1c(i) - u_1c(i - 1)) / sampling_interval;
    derivative_u_2c = (u_2c(i) - u_2c(i - 1)) / sampling_interval;
    
    derivative_theta_1 = u_1(i) / d * tan(theta_0(i) - theta_1(i));
    derivative_theta_0 = u_2(i);
    
    Xi = (derivative_theta_0 - derivative_theta_1) * tan(theta_0(i) - theta_1(i)) * u_1c(i) / ((cos(theta_0(i) - theta_1(i)))^2) + ((tan(theta_0(i) - theta_1(i)))^2) * derivative_u_2c;
    
    Y(1, 1) = Xi;
    Y(1, 2) = derivative_theta_0 * u_2(i) / cos(theta_0(i) - theta_1(i));
    Y(1, 3) = derivative_u_1c;
    Y(2, 4) = derivative_theta_0 * u_1(i) / cos(theta_0(i) - theta_1(i));
    Y(2, 5) = derivative_u_2c;
    
    derivative_parameter_est = - Lamda \ Y' * (u - u_c);
    parameters_est(:, i + 1) = parameters_est(:, i) + derivative_parameter_est * sampling_interval;
    
    tau = Y * parameters_est(:, i + 1) - Gamma * (u - u_c) - Phi' * Omega;
    tau_1(i + 1) = tau(1);
    tau_2(i + 1) = tau(2);
    end
    
    if i <= num_data - 2
    % (i + 2)th State
    % Velocity
    M_2(1, 1) = parameters(3, 1) + parameters(1, 1) * ((tan(theta_0(i + 1) - theta_1(i + 1)))^2);
    M_2(2, 2) = parameters(5, 1);

    C_2(1, 1) = parameters(1, 1) * (derivative_theta_0 - derivative_theta_1) * tan(theta_0(i + 1) - theta_1(i + 1)) / (cos(theta_0(i + 1) - theta_1(i + 1)))^2;
    C_2(1, 2) = - parameters(2, 1) * derivative_theta_0 / cos(theta_0(i + 1) - theta_1(i + 1));
    C_2(2, 1) = parameters(4, 1) * derivative_theta_0 / cos(theta_0(i + 1) - theta_1(i + 1));

    derivative_u = M_2 \ (tau - C_2 * u);

    u_1(i + 2) = u_1(i + 1) + derivative_u(1) * sampling_interval;
    u_2(i + 2) = u_2(i + 1) + derivative_u(2) * sampling_interval;

    % Coodinate
    derivative_x = u_1(i + 1) * cos(theta_1(i + 1));
    derivative_y = u_1(i + 1) * sin(theta_1(i + 1));
    derivative_theta_1 = u_1(i + 1) / d * tan(theta_0(i + 1) - theta_1(i + 1));
    derivative_theta_0 = u_2(i + 1);

    x(i + 2) = x(i + 1) + derivative_x * sampling_interval;
    y(i + 2) = y(i + 1) + derivative_y * sampling_interval;
    theta_1(i + 2) = theta_1(i + 1) + derivative_theta_1 * sampling_interval;
    theta_0(i + 2) = theta_0(i + 1) + derivative_theta_0 * sampling_interval;
    end
end
