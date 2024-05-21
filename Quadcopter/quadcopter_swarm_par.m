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

% Defining vertice vector V
V = [1,2,3,4]

%Starting x-values for drones in swarm
x_0 = [-1, 2, 0, 1]'; 

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
            val
            if isequal(val, {[i,j]})
                A(i,j) = 1;
                break
            end
        end
    end
end
A

%Defining desired position p_d
x_d = [6,43,8,9]'

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

% Defnining graph Laplacian L
L = D-A

% check row-sum is equal to zero
rowSums = sum(L,2)

% Then Perron can be defined
delta = max(diag(D))
str = "Epsilon can be (" + 0 + "," + 1/delta + "]";
disp(str)
epsilon = 1/delta % This value should be (0,delta^-1)
P = eye(length(L)) - epsilon*L

% Check Perron matrix is defined correctly
disp("A correct Perron matrix should give 1 when multiplied by egeinvector with ones")
P*ones(length(L),1)

%%
% Testing convergence

% Expected Decision (balanced)
alpha_expected = sum(x_0)/length(x_0)

figure
x = x_0;
x_hist = [x];
loops = 10;
for t=1:loops
    x = P*(x-x_d);
    x_hist = [x_hist,x];
end
plot(1:length(x_hist),x_hist')
legends = cellstr(num2str(V', 'N=%-d'))
legend(legends)

figure
x = x_0;
x_hist = [x];
loops = 10;
for t=1:loops
    x_dot = -L*(x-x_d);
    x = x + x_dot * 0.1
    x_hist = [x_hist,x];
end
plot(1:length(x_hist),x_hist')
legends = cellstr(num2str(V', 'N=%-d'))
legend(legends)