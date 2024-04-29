% Motors weigh 1kg
m_vec = [1,1,1,1]
m = sum(m_vec)

% Motors are at these coordinates from center of mass
l = [
    sqrt(2)/2,      sqrt(2)/2,   0
    -sqrt(2)/2,     sqrt(2)/2,   0
    -sqrt(2)/2,     -sqrt(2)/2,   0
    sqrt(2)/2,      -sqrt(2)/2,   0
]'

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


M_RB = [
    m*eye(3)     zeros(3)
    zeros(3)            I_C
]

% PID parameters
PID_inner_max = 100
PID_outer_max = 10

% Gravity
g = 9.82
F_e_g = [0; 0; g]*m % m/s