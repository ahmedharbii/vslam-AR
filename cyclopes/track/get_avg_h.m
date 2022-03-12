function h_avg = get_avg_h(H1,H2)
    x1_liri = logm(H1);
    x2_liri = logm(H2);
    x_avg = (x1_liri + x2_liri) / 2;
    h_avg = expm(x_avg);
end

