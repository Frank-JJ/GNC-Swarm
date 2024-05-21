% Motors weigh 1kg
m_vec = [1, 1, 1, 1]
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


%%---------------------------------------------
%Swarm dynamics
%%
disp("------------------")

% Defining vertice vector V
V = [1,2,3,4]

%Starting x-values for drones in swarm
x_0 = [0, 3, 2, 1]'; 

R_min = 1; %Dist between drones in m

%Define edges using R_min distance between vertices
E = {};
for i = V
    for j = V
        if not(i == j) && abs(x_0(i) - x_0(j)) <= R_min
            E{end+1} = [i,j];           
        end
    end
end
E

%Use found edges to find adjecency matrix
A = zeros(length(V));
for i = V
    for j = V
        for val = E
            if isequal(val, {[i,j]})
                A(i,j) = 1;
                break
            end
        end
    end
end
A

%Defining desired position p_d
x_d = [1,4,-2,7]'

% Defining neighbors N
N = {};
for i = V
    N(i) = {find(not(A(i,:)==0))};
end
N

% Defining graph G
G = {V,E}

% Defining degree matrix D
D = zeros(length(V));
for i = V
    for j = V
        if not(i==j)
            D(i,i) = D(i,i) + A(i,j);
        end
    end
end
D

% Defnining graph Laplacian L
L = D-A

% check row-sum is equal to zero
rowSums = sum(L,2)

% Define eigenvector of *1* v
v = ones(length(L),1)

% Test result of L*v=lambda*v, which should then give zero
shouldBeZero = L*v

% Test result of L*v=lambda*v, which should also then give zero
shouldBeZero = v'*L

% Then Perron can be defined
delta = max(diag(D))
str = "Epsilon can be (" + 0 + "," + 1/delta + "]";
disp(str)
epsilon = 1/delta*0.999 % This value should be (0,delta^-1)
%epsilon = 0.1
P = eye(length(L)) - epsilon*L
% Alternative Perron definition
%P = ((eye(length(D))+D)^-1) * (eye(length(A)) + A)

% Check Perron matrix is defined correctly
disp("A correct Perron matrix should give 1 when multiplied by egeinvector with ones")
P*ones(length(L),1)

% check row-sum is equal to one
rowSums = sum(P,2)

% check col-sum is equal to one
colSums = sum(P,1)

disp("Eigenvalues(d) and eigenvectors(v) of L")
[v_L,d_L] = eig(L)
disp("Eigenvalues(d) and eigenvectors(v) of P")
[v_P,d_P] = eig(P)
1-epsilon*diag(d_L)

1-diag(D)/delta

% Testing convergence

% Expected Decision (balanced)
alpha_expected = sum(x_0)/length(x_0)

figure
x = x_0;
x_hist = [x];
loops = 20;
for t=1:loops
    x = P*(x)
    x_hist = [x_hist,x];
end
plot(1:length(x_hist),x_hist',[0,loops],[alpha_expected,alpha_expected])
legends = cellstr(num2str(V', 'N=%-d'))
legend(legends)
title("Perron")

distances = zeros(length(x_d));
for i=1:length(x_d)
    for j=1:length(x_d)
        distances(i,j) = x_d(i) - x_d(j);
    end
end
distances

final_distances = zeros(length(x));
for i=1:length(x)
    for j=1:length(x)
        final_distances(i,j) = x(i) - x(j);
    end
end
final_distances

figure
x = x_0;
x_hist = [x];
loops = 1000;
delta_time = 0.005;
for t=1:loops
    x_dot = -L*(x);
    x = x + x_dot * delta_time;
    x_hist = [x_hist,x];
end
plot(1:length(x_hist),x_hist')
legends = cellstr(num2str(V', 'N=%-d'))
legend(legends)
title("Laplacian")

figure
x = x_0;
x_hist = [x];
loops = 1000;
delta_time = 0.005;
for t=1:loops
    x_dot = -L*(x-x_d);
    x = x + x_dot * delta_time;
    x_hist = [x_hist,x];
end
plot(1:length(x_hist),x_hist')
legends = cellstr(num2str(V', 'N=%-d'))
legend(legends)
title("Laplacian with x_d")

%%
% Compare simulink output with matlab output

distances

final_distances

sim_lastx = simout_x(end,:)';
sim_distances = zeros(length(x));
for i=1:length(x)
    for j=1:length(x)
        sim_distances(i,j) = sim_lastx(i) - sim_lastx(j);
    end
end
sim_distances