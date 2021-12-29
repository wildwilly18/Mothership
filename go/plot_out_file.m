%Plotting Script for the out.txt files from the Simulation.

data = importfile('out.txt');

close all

f = figure('Name', 'Flight Data', 'NumberTitle', 'off');
t = tiledlayout(3,3);
t.TileSpacing = 'compact';
t.Padding = 'compact';

%Tile one X Position 
nexttile
plot(data.Elapsed_Time, data.local_X, 'b', 'DisplayName', 'Local X')
hold on
plot(data.Elapsed_Time, data.R_target_X, 'r--', 'DisplayName', 'Target X')
plot(data.Elapsed_Time, data.R_target_X - data.local_X, 'g-', 'DisplayName', 'Calculated Error')
title('Rendesvouz X Error')
legend; grid minor; hold off

nexttile

plot(data.Elapsed_Time, data.local_Y, 'b', 'DisplayName', 'Local Y')
hold on
plot(data.Elapsed_Time, data.R_target_Y, 'r--', 'DisplayName', 'Target Y')
plot(data.Elapsed_Time, data.R_target_Y - data.local_Y, 'g-', 'DisplayName', 'Calculated Error')
title('Rendesvouz Y error')
legend; grid minor; hold off

nexttile

plot(data.Elapsed_Time, data.local_Z, 'b', 'DisplayName', 'Local Z')
hold on
plot(data.Elapsed_Time, data.R_target_Z, 'r--', 'DisplayName', 'Target Z')
plot(data.Elapsed_Time, data.R_target_Z - data.local_Z, 'g-', 'DisplayName', 'Calculated Error')
title('Rendesvouz Z error')
legend; grid minor; hold off

%Algorithm Counter Tiles
nexttile
plot(data.Elapsed_Time, data.Algo_counter, 'b-', 'DisplayName', 'R Algo Counter')
hold on
plot(data.Elapsed_Time, data.Vis_counter, 'g-', 'DisplayName', 'V Algo counter')
title('Algorithm Counters')
legend; grid minor; hold off

%Algorithm Enabled and Last
nexttile
plot(data.Elapsed_Time, data.visual_mode, 'g--', 'DisplayName','Visual Mode')
hold on
plot(data.Elapsed_Time, data.Algo_last, 'r--', 'DisplayName','R Algo Last')
plot(data.Elapsed_Time, data.Vis_last, 'b--', 'DisplayName', 'V Algo Last')
ylim([-0.2 1.2])
title('Mode and Last Mode Tracker')
legend; grid minor; hold off

%Integrator counters
nexttile
plot(data.Elapsed_Time, data.X_err_integrator,'b--', 'DisplayName', 'X Int')
hold on
plot(data.Elapsed_Time, data.Y_err_integrator, 'g--', 'DisplayName', 'Y Int')
title('X and Y Integrators')
legend; grid minor; hold off

%Mode
nexttile
plot(data.Elapsed_Time, data.Rendesvous_mode,'b--', 'DisplayName', 'Rendesvous Mode')
hold on
plot(data.Elapsed_Time, data.visual_mode, 'g--', 'DisplayName', 'Visual Mode')
title('Mode Enabled')
ylim([-0.2 1.2])
legend; grid minor; hold off

%Integrator counters
nexttile
plot(data.Elapsed_Time, data.Rendesvous_mode,'b--', 'DisplayName', 'Rendesvous Mode')
hold on
plot(data.Elapsed_Time, data.visual_mode, 'g--', 'DisplayName', 'Visual Mode')
title('Mode Enabled')
ylim([-0.2 1.2])
legend; grid minor; hold off

%Consecutives
nexttile
plot(data.Elapsed_Time, data.Vis_Consecutive,'b--', 'DisplayName', 'Rendesvous Mode')
hold on
plot(data.Elapsed_Time, data.Algo_consecutive, 'g--', 'DisplayName', 'Visual Mode')
title('Mode Enabled')
legend; grid minor; hold off
