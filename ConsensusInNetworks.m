%% I. CONSENSUS NETWORKS
% A test of the different notions given in chapter "A consensus networks"
% of "Consensus and Cooperation in Networked Multi-Agent Systems"
%% Definitions
% Define nodes
V = [1,2,3]

% Define edges
E={[2],[1,3],[1]}
ismember(1,E{3})

% Define graph
G = {V,E}
G{1}(1)
iscell(G{2})

%% Test row-sum of graph Laplacian
% Calculate graph Laplacian

L = zeros(length(V));

for i=1:length(L)
    for j=1:length(L)
        if ismember(j,G{2}{i})
            L(i,j)=-1;
        end
        if j==i
            L(i,j)=length(G{2}{i});
        end
    end
end

L

% check row-sum is equal to zero

rowSums = sum(L,2)

%% Test zero eigenvalue of graph Laplacian
% Calculate graph Laplacian

L = zeros(length(V));

for i=1:length(L)
    for j=1:length(L)
        if ismember(j,G{2}{i})
            L(i,j)=-1;
        end
        if j==i
            L(i,j)=length(G{2}{i});
        end
    end
end

L

% Define eigenvector of *1* v
v = ones(length(L),1)

% Test result of L*v=lambda*v, which should then give zero
L*v

% Define x* vector with alpha value instead of ones
alpha = 3
x_star = v*alpha

% Repeat test wih x* instead, which should also give zero
L*x_star

% Thus v and x* are both eigenvectors for the graph Laplacian L
%% II. INFORMATION CONSENSUS
% Testing the notions given in chapter II Information Consensus

% Defining vertice vector V
V = [1,2,3,4,5]

% Defining adjacancy matrix A
A = [
    0,1,1,1,1
    1,0,0,0,0
    1,0,0,0,0
    1,0,0,0,0
    1,0,0,0,0
    ]

%res=find(A(3,:)==1)

% Defining neighbors N
N = {};
for i=1:length(V)
    N(i) = {find(not(A(i,:)==0))};
end
N

% Defining edges E
E = {};
for i=1:length(V)
    for j=1:length(V)
        if not(A(i,j)==0)
            E{end+1} = [i,j];
        end
    end
end
E

% Defining graph G
G = {V,E}

% Defining degree matrix D
D = zeros(length(V));
for i=1:length(V)
    for j=1:length(V)
        if not(i==j)
            D(i,i) = D(i,i) + A(i,j);
        end
    end
end
D

%% Defnining graph Laplacian L
L = D-A

% check row-sum is equal to zero
rowSums = sum(L,2)

% Testing with an actual system
x = [1;2;3;4;5]

x_dot = -L*x

disp("sum of x_dot:")
sum(x_dot)

x_final = x_dot + x
disp("supposed alpha:")
alpha_cal = sum(x)/length(x)
disp("actual alpha:")
alpha_res = sum(x_final)/length(x_final)

plot([0;1],[x,x_final],[0,1],[alpha_cal,alpha_res])
legend('x1','x2','x3','x4','x5','aplha')


figure
x_dot_i = zeros(length(V),1);
x_hist = x;
x_dot_hist = x;
for t=1:10
    for i=1:length(V)
        % for j=N{i}
        %     x_dot_i(i) = A(i,j)*(x(j)-x(i));
        % end
        x_dot_i = -L*x;
    end
    x_dot_i;
    x = x + x_dot_i;
    x_dot_hist = [x_dot_hist,x_dot_i];
    x_hist = [x_hist,x];
end
plot(x_hist')
hold on
plot(x_dot_hist')


%% Defining alternative graph Laplacian for discrite time P, a so called
% Perron matrix
% Need to define Laplacian matrix first
L = D-A

% Then Perron can be defined
delta = max(diag(D))
str = "Epsilon can be (" + 0 + "," + 1/delta + "]";
disp(str)
epsilon = 1/delta % This value should be (0,delta^-1)
P = eye(length(L)) - epsilon*L

% Check Perron matrix is defined correctly
disp("A correct Perron matrix should give 1 when multiplied by egeinvector with ones")
P*ones(length(L),1)

% Testing with an actual system
x_0 = [1;2;3;4;5]

% Expected Decision (balanced)
alpha_expected = sum(x_0)/length(x_0)

% Testing convergence
figure
x = x_0;
x_hist = [x];
for t=1:10
    x = P*x;
    x_hist = [x_hist,x];
end
plot(1:length(x_hist),x_hist',[1,10],[alpha_expected,alpha_expected])
legends = cellstr(num2str(V', 'N=%-d'))
legends{end+1} = "alpha"
legend(legends)