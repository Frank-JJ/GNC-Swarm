%% Drone dynamics setup
% Motors weigh 1kg
m_vec = [1, 1, 1, 1] %kg
m = sum(m_vec)

% Motors are at these coordinates from center of mass
diameter = 1;

l = [sqrt(2)/2,      sqrt(2)/2,    0
     -sqrt(2)/2,     sqrt(2)/2,    0
     -sqrt(2)/2,     -sqrt(2)/2,   0
     sqrt(2)/2,      -sqrt(2)/2,   0]'

l = l * diameter/2;

% Inertial tensor of quadcopter is then
I_xx = 0;
I_yy = 0;
I_zz = 0;
for i=1:4
    I_xx = I_xx + (l(2,i)^2)*m_vec(i);
    I_yy = I_yy + (l(1,i)^2)*m_vec(i);
    I_zz = I_zz + (l(1,i)^2 + l(2,i)^2)*m_vec(i);
end
I_C = diag([I_xx,I_yy,I_zz])

M_RB = [m*eye(3)     zeros(3)
        zeros(3)     I_C]

% Gravity
g = 9.82;
F_e_g = [0; 0; g]*m; % m/s

% PID parameters
thrust_max = 5*(m*g);
v_angle_in_max = 60;
angle_outer_max = 60;

out_P = 20;
out_I = 0;
out_D = 50;

in_thrust_P = 10;
in_thrust_I = 0;
in_thrust_D = 30;

in_P = 20;
in_I = 0;
in_D = 20;

% Swarm dynamics 2D with dynamic Laplacian
disp("----------------")
% Defining vertice vector V
%V = 1:4 %W/o fix-agent
V = 1:6

%Weights for Func_1 and Func_2
r_m = 2.2; % m  distance between drones
w_1 = 6;
w_2 = 1.05*sqrt(3*w_1^2)/abs(atan(-r_m^2)-atan(pi^2))
w = 0.6;
R_c = 3; % m communication distance between drones for collision avoidance

w_2_with_h = 1.05*(1/1.5)*sqrt(3*w_1^2)/abs(atan(-r_m^2)-atan(pi^2))