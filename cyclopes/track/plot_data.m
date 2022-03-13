% function plot_data(data)
x_1 = [];
x_2 = [];
for i = 1:size(data_1,2)
norm_x_1(i) = data_1(i).left_right.norm_x;
norm_x_2(i) = data_2(i).left_right.norm_x;

norm_r_1(i) = data_1(i).left_right.norm_residues;
norm_r_2(i) = data_2(i).left_right.norm_residues;

x_1 = [x_1 data_1(i).left_right.x];
x_2 = [x_2 data_2(i).left_right.x];

% norm_x_1(i) = data_1(i).left.norm_x;
% norm_x_2(i) = data_2(i).left.norm_x;

% norm_r_1(i) = data_1(i).left.norm_residues;
% norm_r_2(i) = data_2(i).left.norm_residues;

% x_1 = [x_1 data_1(i).left.x];
% x_2 = [x_2 data_2(i).left.x];
end

figure;
plot(norm_x_1);
hold on;
plot(norm_x_2);
legend(["prev_est", "curr_est"]);
title("norm X")

figure;
plot(norm_r_1);
hold on;
plot(norm_r_2);
legend(["1", "2"]);
title("norm residuals")

figure;
for j=1:8
    subplot(2,8,j)
    plot(x_1(j,:));
    subplot(2,8,j+8)
    plot(x_2(j,:));
end
% legend(["1", "2"]);
title("X")
