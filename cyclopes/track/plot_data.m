% function plot_data(data)
x_1 = [];
x_2 = [];
x_3 = [];
for i = 1:size(data_1,2)
norm_x_1(i) = data_1(i).norm_x;
norm_r_1(i) = data_1(i).norm_residues;
x_1 = [x_1 data_1(i).x];
end
for i = 1:size(data_2,2)
norm_x_2(i) = data_2(i).norm_x;
norm_r_2(i) = data_2(i).norm_residues;
x_2 = [x_2 data_2(i).x];
end
for i = 1:size(data_3,2)
norm_x_3(i) = data_3(i).norm_x;
norm_r_3(i) = data_3(i).norm_residues;
x_3 = [x_3 data_3(i).x];
end

figure;
set(gca,'FontWeight','bold');
set(gca,'FontSize',15);
plot(norm_x_1,LineWidth=1);
hold on;
plot(norm_x_2,LineWidth=1);
hold on;
plot(norm_x_3,LineWidth=1);
ylim([0 10]);
legend(["Ref Jac", "Curr Jac", "ESM"]);
title("norm x")

figure;
set(gca,'FontWeight','bold');
set(gca,'FontSize',15);
plot(norm_r_1,LineWidth=1);
hold on;
plot(norm_r_2,LineWidth=1);
hold on;
plot(norm_r_3,LineWidth=1);
legend(["Ref Jac", "Curr Jac", "ESM"]);
title("norm residuals")

figure;
set(gca,'FontWeight','bold');
set(gca,'FontSize',15);
for j=1:8
    subplot(3,8,j)
    plot(x_1(j,:),LineWidth=1);
    if j == 1
        ylabel("Ref Jac");
    end
    title(sprintf(['x', num2str(j)]));
    subplot(3,8,j+8)
    plot(x_2(j,:),LineWidth=1);
    if j == 1
        ylabel("Curr Jac");
    end
    title(sprintf(['x', num2str(j)]));
    subplot(3,8,j+2*8)
    plot(x_3(j,:),LineWidth=1);
    if j == 1
        ylabel("ESM");
    end
    title(sprintf(['x', num2str(j)]));
end
% legend(["1", "2"]);
% title("X")
