r_m = 1;

r_m1 = 1;
r_m2 = 2;
r_m3 = 3;

w_2 = 11;

p1 = [1, 4, 6, 9, 0
      0, 5, 9, 9, 0]'

p2 = [1, 4, 0.4, 9, 0
      0, 5, 0, 9, 0]';

pdot = fun2(p1, w_2, r_m)
pdot2 = fun2(p2, w_2, r_m)

p_dot_plot1 = zeros(100000,1);
p_dot_plot2 = zeros(100000,1);
p_dot_plot3 = zeros(100000,1);

for i = -50000:50000
    p_dot_plot1(i+50001,:) = (atan2((i/1000)^2 - r_m1^2,1) - atan2((i/1000)^2 + pi^2,1));
    p_dot_plot2(i+50001,:) = (atan2((i/1000)^2 - r_m2^2,1) - atan2((i/1000)^2 + pi^2,1));
    p_dot_plot3(i+50001,:) = (atan2((i/1000)^2 - r_m3^2,1) - atan2((i/1000)^2 + pi^2,1));
end

plot(-50:0.001:50,p_dot_plot1(:,1), -50:0.001:50,p_dot_plot2(:,1), -50:0.001:50,p_dot_plot3(:,1))


function p_dot_2 = fun2(p, w_2, r_m)

n_drones = size(p,1) -1
n_dim = size(p,2)

p_dot_2 = zeros(n_drones + 1,n_dim);

for i = 1:(n_drones)
    for j = 1:(n_drones)
        if not(i == j)
            d_ij = sqrt((p(i,1)-p(j,1))^2+(p(i,2)-p(j,2))^2);
            p_dot_2(i,:) = p_dot_2(i,:) + w_2*(atan2(d_ij^2-r_m^2,1) - atan2(d_ij^2+pi^2,1))*(p(i,:)-p(j,:))/d_ij;
        end
    end
end

end


function p_dot_2 = fun3(p, w_2, r_m)

n_drones = size(p,1) -1;
n_dim = size(p,2);

p_dot_2 = zeros(n_drones + 1,n_dim);

for i = 1:(n_drones)
    for j = 1:(n_drones)
        if not(i == j)
            d_ij = sqrt((p(i)-p(j))^2+(p(i)-p(j))^2);
            p_dot_2(i,:) = p_dot_2(i,:) + w_2*(atan2(d_ij^2-r_m^2,1) - atan2(d_ij^2+pi^2,1))*(p(i,:)-p(j,:))/d_ij;
        end
    end
end

end