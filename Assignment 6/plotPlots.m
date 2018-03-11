function [] = plotPlots(t, F, title, yLabels)
% Plot my subplots for the longitudinal dynamics simulation
set(0, 'defaulttextinterpreter', 'latex');
figure
set(gcf, 'units', 'normalized', ...
    'pos', [0.1, 0.1, 0.8, 0.75])

subplot(2, 2, 1)
hold on; grid on; grid minor;
plot(t, F(:, 1), 'b-', 'linewidth', 1.1)
xlabel('Time, [s]')
ylabel(yLabels{1})
set(gca, 'ticklabelinterpreter', 'latex', ...
         'fontsize', 15)

subplot(2, 2, 2)
hold on; grid on; grid minor;
plot(t, F(:, 2), 'b-', 'linewidth', 1.1)
xlabel('Time, [s]')
ylabel(yLabels{2})
set(gca, 'ticklabelinterpreter', 'latex', ...
         'fontsize', 15)

subplot(2, 2, 3)
hold on; grid on; grid minor;
plot(t, F(:, 3), 'b-', 'linewidth', 1.1)
xlabel('Time, [s]')
ylabel(yLabels{3})
set(gca, 'ticklabelinterpreter', 'latex', ...
         'fontsize', 15)

subplot(2, 2, 4)
hold on; grid on; grid minor;
plot(t, F(:, 4), 'b-', 'linewidth', 1.1)
xlabel('Time, [s]')
ylabel(yLabels{4})
set(gca, 'ticklabelinterpreter', 'latex', ...
         'fontsize', 15)

[~, t] = suplabel(title, 't');
set(t, 'interpreter', 'latex', ...
        'fontsize', 15)

end