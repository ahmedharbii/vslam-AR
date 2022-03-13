% function plot_data(data)
x_1 = [];
x_2 = [];
x_3 = [];
data_1 = data_4;
for i = 1:size(data_1,2)
norm_x_1(i) = data_1(i).norm_x;
norm_r_1(i) = data_1(i).norm_residues;
x_1 = [x_1 data_1(i).x];
end

figure;
set(gca,'FontWeight','bold');
set(gca,'FontSize',15);
plot(norm_x_1,LineWidth=1);
ylim([0 10]);
title("norm x")

figure;
set(gca,'FontWeight','bold');
set(gca,'FontSize',15);
plot(norm_r_1,LineWidth=1);
title("norm residuals")

figure;
set(gca,'FontWeight','bold');
set(gca,'FontSize',15);
for j=1:8
    subplot(1,8,j)
    plot(x_1(j,:),LineWidth=1);
    if j == 1
        ylabel("Ref Jac");
    end
    title(sprintf(['x', num2str(j)]));
end
% legend(["1", "2"]);
% title("X")
