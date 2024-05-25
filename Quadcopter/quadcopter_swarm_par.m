%% Drone dynamics setup
% Motors weigh 1kg
m_vec = [1, 1, 1, 1] %kg
m = sum(m_vec)

% Motors are at these coordinates from center of mass
diameter = 1;

l = [
    sqrt(2)/2,      sqrt(2)/2,   0
    -sqrt(2)/2,     sqrt(2)/2,   0
    -sqrt(2)/2,     -sqrt(2)/2,   0
    sqrt(2)/2,      -sqrt(2)/2,   0
]'

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


M_RB = [
    m*eye(3)     zeros(3)
    zeros(3)          I_C
]

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

%% Swarm dynamics 1D
disp("------------------")

% Defining vertice vector V
%V = [1,2,3,4]
%V = 1:4 %W/o fix-agent
V = 1:4;

%Starting x-values for drones in swarm
%x_0 = [0, 1, 2, 3]'
%x_0 = [0, 3, 2, 1,-1,-2,-3,-3]'
p_0 = [0, 1, 2, 3, 0]'; %Fix-agent
p_0 = p_0(V)

%Defining desired displacements p_d
p_d = [1,4,-2,7,0]';
p_d = p_d(V)

%x_d = [1,4,-2,7,10,3,-4,6]'

%Desired fix-agent
fix = false;


R_min = 2; %Dist between drones in m

%Define edges using R_min distance between vertices
E = {};
for i = V
    for j = V
        if not(i == j) && norm(p_0(i,:) - p_0(j,:)) <= R_min
            E{end+1} = [i,j];
        end
    end
end
E

%Use found edges to find adjecency matrix
A = zeros(length(V));
if fix
for i = V
    if i == length(V) %Meaning this is the fix-agent
        A(i,j) = 0;
    else
        for j = V
            if j == length(V)
                A(i,j) = 1;
            end
            for val = E
                if isequal(val, {[i,j]})
                    A(i,j) = 1;
                    break
                end
            end
        end
    end
end
else
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
end

A

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
suppused_eigenvalues_for_P = 1-epsilon*diag(d_L)

suppused_eigenvalues_for_P = 1-diag(D)/delta

% Testing convergence

% Expected Decision (balanced)
alpha_expected = sum(p_0,1)/length(p_0)

figure
p = p_0;
x_hist = [p];
loops = 20;
for t=1:loops
    p = P*(p);
    x_hist = [x_hist,p];
end
plot(1:length(x_hist),x_hist',[1,loops+1],[alpha_expected(1),alpha_expected(1)])
legends = cellstr(num2str(V', 'i=%-d'));
legends{end+1} = "expected alpha";
legend(legends)
title("Perron x-axis")

expected_distances_x = zeros(length(p_d));
for i=1:length(p_d)
    for j=1:length(p_d)
        expected_distances_x(i,j) = p_d(i) - p_d(j);
    end
end
expected_distances_x

final_distances_P_x = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        final_distances_P_x(i,j) = p(i) - p(j);
    end
end
final_distances_P_x

figure
p = p_0;
x_hist = [p];
loops = 20;
delta_time = epsilon;
for t=1:loops
    p_dot = -L*(p);
    p = p + p_dot * delta_time;
    x_hist = [x_hist,p];
end
plot(1:length(x_hist),x_hist',[1,loops+1],[alpha_expected(1),alpha_expected(1)])
legends = cellstr(num2str(V', 'i=%-d'));
legends{end+1} = "expected alpha";
legend(legends);
title("Laplacian x-axis")

final_distances_L_x = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        final_distances_L_x(i,j) = p(i) - p(j);
    end
end
final_distances_L_x

figure
p = p_0;
x_hist = [p];
loops = 20;
for t=1:loops
    p_dot = -L*(p-p_d);
    p = p + p_dot * delta_time;
    x_hist = [x_hist,p];
end
plot(1:length(x_hist),x_hist',[1,length(x_hist)],[p_d,p_d])
legends = cellstr(num2str(V', 'i=%-d'));
legends = cat(1,legends,cellstr(num2str(V', 'expected_i=%-d')));
legend(legends)
title("Laplacian with p_d x-axis")

final_distances_L_w_desired_values_x = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        final_distances_L_w_desired_values_x(i,j) = p(i) - p(j);
    end
end
final_distances_L_w_desired_values_x

%%Compare simulink output with matlab output

expected_distances_x

final_distances_L_w_desired_values_x

sim_lastx = simout_x(end,:)';
sim_distances = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        sim_distances(i,j) = sim_lastx(i) - sim_lastx(j);
    end
end
sim_distances

%Is the fix-agent actually fixed? (yes)
agent_fixed = max(simout_x(:,end)) == min(simout_x(:,end)) 

%% Swarm dynamics 2D
disp("------------------")

% Defining vertice vector V
%V = [1,2,3,4]
%V = 1:4 %W/o fix-agent
V = 1:6

%Weights for Func_1 and Func_2
r_m = 2.2; % m  distance between drones
w_1 = 6;
w_2 = 1.05*sqrt(3*w_1^2)/abs(atan(-r_m^2)-atan(pi^2));
w = w_1/10;

%Starting x-values for drones in swarm
p_0 = [10,5,-8,-5,8,0;
       5,-5,5,-13,8,0]' %Fix-agent
p_0 = p_0(V,:)

%Defining desired displacements p_d
p_d = [1,4,-2,7, 9,0;
       0,0,0,0, 0,0]'
p_d = p_d(V,:)

%Desired fix-agent
fix = true;


R_min = 100; %Dist between drones in m

%Define edges using R_min distance between vertices
E = {};
for i = V
    for j = V
        if not(i == j) && norm(p_0(i,:) - p_0(j,:)) <= R_min
            E{end+1} = [i,j];           
        end
    end
end
E

%Use found edges to find adjecency matrix
A = zeros(length(V));
if fix
for i = V
    if i == length(V) %Meaning this is the fix-agent
        A(i,j) = 0;
    
    else
        for j = V
            if j == length(V)
                A(i,j) = 1;
            end
            for val = E
                if isequal(val, {[i,j]})
                    A(i,j) = 1;
                    break
                end
            end
        end
    end
end
else
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
end

A

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
suppused_eigenvalues_for_P = 1-epsilon*diag(d_L)

% Testing convergence

% Expected Decision (balanced)
alpha_expected = sum(p_0,1)/size(p_0,1)

figure
p = p_0(1:length(V),:)
x_hist = [p(:,1)];
y_hist = [p(:,2)];
loops = 20;
for t=1:loops
    p = P*(p);
    x_hist = [x_hist,p(:,1)];
    y_hist = [y_hist,p(:,2)];
end
plot(1:length(x_hist),x_hist, ...
    1:length(y_hist),y_hist, ...
    [1,loops+1],[alpha_expected(1),alpha_expected(1)])
legends = cellstr(num2str(V', 'ix=%-d'));
legends = cat(1,legends,cellstr(num2str(V', 'iy=%-d')));
legends{end+1} = "expected alpha";
legend(legends)
title("Perron 2D")

expected_distances_x = zeros(length(p_d));
expected_distances_y = zeros(length(p_d));
for i=1:length(p_d)
    for j=1:length(p_d)
        expected_distances_x(i,j) = p_d(i,1) - p_d(j,1);
        expected_distances_y(i,j) = p_d(i,2) - p_d(j,2);
    end
end
expected_distances_x
expected_distances_y

final_distances_P_x = zeros(length(p));
final_distances_P_y = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        final_distances_P_x(i,j) = p(i,1) - p(j,1);
        final_distances_P_y(i,j) = p(i,2) - p(j,2);
    end
end
final_distances_P_x
final_distances_P_y

figure
p = p_0;
x_hist = [p(:,1)];
y_hist = [p(:,2)];
loops = 20;
delta_time = epsilon;
for t=1:loops
    p_dot = -L*(p);
    p = p + p_dot * delta_time;
    x_hist = [x_hist,p(:,1)];
    y_hist = [y_hist,p(:,2)];
end
plot(1:length(x_hist),x_hist, ...
    1:length(y_hist),y_hist, ...
    [1,loops+1],[alpha_expected(1),alpha_expected(1)])
legends = cellstr(num2str(V', 'ix=%-d'));
legends = cat(1,legends,cellstr(num2str(V', 'iy=%-d')));
legends{end+1} = "expected alpha";
legend(legends)
title("Laplacian 2D")


final_distances_L_x = zeros(length(p));
final_distances_L_y = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        final_distances_L_x(i,j) = p(i,1) - p(j,1);
        final_distances_L_y(i,j) = p(i,2) - p(j,2);
    end
end
final_distances_L_x
final_distances_L_y

figure
p = p_0;
x_hist = [p(:,1)];
y_hist = [p(:,2)];
loops = 20;
delta_time = epsilon;
for t=1:loops
    p_dot = -L*(p-p_d);
    p = p + p_dot * delta_time;
    x_hist = [x_hist,p(:,1)];
    y_hist = [y_hist,p(:,2)];
end
plot(1:length(x_hist),x_hist' ...
    ,1:length(y_hist),y_hist' ...
    ,[1,length(x_hist)],[p_d(:,1),p_d(:,1)] ...
    ,[1,length(y_hist)],[p_d(:,2),p_d(:,2)])
legends = cellstr(num2str(V', 'ix=%-d'));
legends = cat(1,legends,cellstr(num2str(V', 'iy=%-d')));
legends = cat(1,legends,cellstr(num2str(V', 'expected_ix=%-d')));
legends = cat(1,legends,cellstr(num2str(V', 'expected_iy=%-d')));
legend(legends)
title("Laplacian with p_d 2D")

final_distances_L_w_desired_values_x = zeros(length(p));
final_distances_L_w_desired_values_y = zeros(length(p));
for i=1:length(p)
    for j=1:length(p)
        final_distances_L_w_desired_values_x(i,j) = p(i,1) - p(j,1);
        final_distances_L_w_desired_values_y(i,j) = p(i,2) - p(j,2);
    end
end
final_distances_L_w_desired_values_x
final_distances_L_w_desired_values_y

%% Compare simulink output with matlab output 2D

expected_distances_x
final_distances_L_w_desired_values_x

sim_last = simout_x(end-3:end,:)
sim_distances_x = zeros(length(V));
sim_distances_y = zeros(length(V));
for i=V
    for j=V
        sim_distances_x(i,j) = sim_last(i,1) - sim_last(j,1);
        sim_distances_y(i,j) = sim_last(i,2) - sim_last(j,2);
    end
end
sim_distances_x

expected_distances_y

final_distances_L_w_desired_values_y

sim_distances_y

%Is the fix-agent actually fixed?
agent_fixed = max(simout_x(:,end)) == min(simout_x(:,end)) 

%% Swarm dynamics 2D with dynamic Laplacian
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

%Starting x-values for drones in swarm
p_0 = [5,5,-8,5,8,0;
       10,-5,5,-13,8,0]' %Fix-agent
p_0 = p_0(V,:)

%Defining desired displacements p_d
p_d = [1,4,-2,7, 9,0;
       0,0,0,0, 0,0]'
p_d = p_d(V,:)

p = p_0
p_len = size(p,1);
drone_len = p_len-using_fixed_agent; % Without fixed agent

% Defining code for simulink usage
using_fixed_agent = 1
fully_connected = 0
recieving_agents = [1,1,0,1,1,1]
sending_agents = [1,1,1,1,1,1]
disabled_edges = zeros(p_len)
disabled_edges(1:4,6) = 0
k_nearest = 1
communication_distance = 12
balanced = false

% Define the edge list
(drone_len-1)*(drone_len)
% Define the edge list
E = zeros((drone_len-1)*(drone_len),2);
R = zeros(drone_len ,drone_len);
num_edges = 1;
if fully_connected
    for i=1:drone_len 
        for j = 1:drone_len 
            if not(i == j)
                E(num_edges,:) = [i,j];
                num_edges = num_edges + 1;
            end
        end
    end
elseif communication_distance > 0
    % Define edges using min distance between vertices
    % Get min distance from drone i to all other drones
    for i=1:drone_len 
        for j=1:drone_len 
            if not(i == j)
                R(i,j) = norm(p(i,:) - p(j,:));
            else
                R(i,j) = inf;
            end
        end
    end
    R_min = mink(R,k_nearest,1);
    for i=1:drone_len 
        num_conn = 0;
        for j = 1:drone_len 
            if num_conn >= k_nearest
                break
            end
            if not(i == j) && R(i,j) <= R_min(k_nearest,i) && R(i,j) <= communication_distance
                num_conn = num_conn + 1;
                E(num_edges,:) = [i,j];
                num_edges = num_edges + 1;
                if balanced
                    E(num_edges,:) = [j,i];
                    num_edges = num_edges + 1;
                end
            end
        end
    end
else
    % Define edges using min distance between vertices
    % Get min distance from drone i to all other drones
    for i=1:drone_len 
        for j=1:drone_len 
            if not(i == j)
                R(i,j) = norm(p(i,:) - p(j,:));
            else
                R(i,j) = inf;
            end
        end
    end
    R_min = mink(R,k_nearest,1);
    for i=1:drone_len 
        num_conn = 0;
        for j = 1:drone_len 
            if num_conn >= k_nearest
                break
            end
            if not(i == j) && R(i,j) <= R_min(k_nearest,i)
                num_conn = num_conn + 1;
                E(num_edges,:) = [i,j];
                num_edges = num_edges + 1;
                if balanced
                    E(num_edges,:) = [j,i];
                    num_edges = num_edges + 1;
                end
            end
        end
    end
end
E

% Define adjacancy matrix A based on these edges
A = zeros(p_len);
if using_fixed_agent
    for i=1:p_len
        if i == p_len %Meaning this is the fix-agent
            A(i,:) = 0;
        else
            for j = 1:p_len
                if j == p_len
                    A(i,j) = 1;
                end
                for edge = 1:(num_edges-1)
                    edge_vals = E(edge,:);
                    A(edge_vals(1),edge_vals(2)) = 1;
                end
            end
        end
    end
else
    for edge = 1:(num_edges-1)
        edge_vals = E(edge,:);
        A(edge_vals(1),edge_vals(2)) = 1;
    end
end
A

% Defining degree matrix D
D = zeros(p_len);
for i=1:p_len
    for j=1:p_len
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

% Test result of L*v=lambda*v, 
% which should then give zero as per definition (Murray page 6)
shouldBeZero = L*v

% Test result of L*v=lambda*v, 
% which should also then give zero if the graph is balanced (Murray page 7)
% It will not be balanced with the (fix agent)
shouldBeZero = v'*L

% Then Perron can be defined
delta = max(diag(D))
str = "Epsilon can be (" + 0 + "," + 1/delta + "]";
disp(str)
epsilon = 1/delta*0.999 % This value should be (0,delta^-1)
%epsilon = 0.1
P = eye(length(L)) - epsilon*L

% Testing convergence

% Expected Decision (balanced)
alpha_expected = sum(p_0,1)/size(p_0,1)

p = p_0;
x_hist = [p(:,1)];
y_hist = [p(:,2)];
loops = 20;
delta_time = epsilon;
for t=1:loops
    p_dot = -L*(p);
    p = p + p_dot * delta_time;
    x_hist = [x_hist,p(:,1)];
    y_hist = [y_hist,p(:,2)];
end
figure
plot(1:length(x_hist),x_hist, ...
    [1,loops+1],[alpha_expected(1),alpha_expected(1)])
legends = cellstr(num2str(V', 'ix=%-d'));
legends{end+1} = "expected alpha x";
legend(legends)
title("Laplacian 2D x")
figure
plot(1:length(y_hist),y_hist, ...
    [1,loops+1],[alpha_expected(2),alpha_expected(2)])
legends = cellstr(num2str(V', 'iy=%-d'));
legends{end+1} = "expected alpha y";
legend(legends)
title("Laplacian 2D y")

% Removing disconnected agents
% remove the ones that stopped recieving
for i=1:p_len
    if recieving_agents(i) == 0
        A(i,:) = 0;
    end
end
% remove the onse that stopped sending
for j=1:p_len
    if sending_agents(j) == 0
        A(:,j) = 0;
    end
end
% remove the edges that are not allowed
A = A.*(ones(p_len)-disabled_edges);
A

% Defining degree matrix D
D = zeros(p_len);
for i=1:p_len
    for j=1:p_len
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

% Test result of L*v=lambda*v, 
% which should then give zero as per definition (Murray page 6)
shouldBeZero = L*v

% Test result of L*v=lambda*v, 
% which should also then give zero if the graph is balanced (Murray page 7)
% It will not be balanced with the (fix agent)
shouldBeZero = v'*L

% Then Perron can be defined
delta = max(diag(D))
str = "Epsilon can be (" + 0 + "," + 1/delta + "]";
disp(str)
epsilon = 1/delta*0.999 % This value should be (0,delta^-1)
%epsilon = 0.1
P = eye(length(L)) - epsilon*L

% Testing convergence

% Expected Decision (balanced)
alpha_expected = sum(p_0,1)/size(p_0,1)

p = p_0;
x_hist = [p(:,1)];
y_hist = [p(:,2)];
loops = 20;
delta_time = epsilon;
for t=1:loops
    p_dot = -L*(p);
    p = p + p_dot * delta_time;
    x_hist = [x_hist,p(:,1)];
    y_hist = [y_hist,p(:,2)];
end
figure
plot(1:length(x_hist),x_hist, ...
    [1,loops+1],[alpha_expected(1),alpha_expected(1)])
legends = cellstr(num2str(V', 'ix=%-d'));
legends{end+1} = "expected alpha x";
legend(legends)
title("Laplacian 2D x removed drones")
figure
plot(1:length(y_hist),y_hist, ...
    [1,loops+1],[alpha_expected(2),alpha_expected(2)])
legends = cellstr(num2str(V', 'iy=%-d'));
legends{end+1} = "expected alpha y";
legend(legends)
title("Laplacian 2D y removed drones")

%% Collision avoidance with falied transmission
%inputs
p
w_2
r_m
R_c
recieving_agents = [1,1,1,1,1]
sending_agents = [1,1,1,1,1]

n_drones = size(p,1) -1
n_dim = size(p,2)

p_dot_2 = zeros(n_drones + 1,n_dim);
dist = zeros(n_drones + 1,1);


for i = 1:(n_drones)
    if not(recieving_agents(i))
        continue
    end
    for j = 1:(n_drones)
        if not(i == j) && sending_agents(j)
            d_ij = sqrt((p(i,1)-p(j,1))^2+(p(i,2)-p(j,2))^2);
            dist(i) = d_ij;
            if d_ij < R_c
                p_dot_2(i,:) = p_dot_2(i,:) + w_2*(atan2(d_ij^2-r_m^2,1) - atan2(d_ij^2+pi^2,1))*(p(i,:)-p(j,:))/d_ij; 
            end
        end
    end
end
p_dot_2