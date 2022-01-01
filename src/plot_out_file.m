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
ylim([-0.5 0.5])
title('Rendesvouz X Error')
legend; grid minor; hold off

%Tile two Y Position
nexttile
plot(data.Elapsed_Time, data.local_Y, 'b', 'DisplayName', 'Local Y')
hold on
plot(data.Elapsed_Time, data.R_target_Y, 'r--', 'DisplayName', 'Target Y')
plot(data.Elapsed_Time, data.R_target_Y - data.local_Y, 'g-', 'DisplayName', 'Calculated Error')
ylim([-0.5 0.5])
title('Rendesvouz Y error')
legend; grid minor; hold off

%Tile three Z Position
nexttile
plot(data.Elapsed_Time, data.local_Z, 'b', 'DisplayName', 'Local Z')
hold on
plot(data.Elapsed_Time, data.R_target_Z, 'r--', 'DisplayName', 'Target Z')
plot(data.Elapsed_Time, data.R_target_Z - data.local_Z, 'g-', 'DisplayName', 'Calculated Error')
title('Rendesvouz Z error')
legend; grid minor; hold off

%Tile four X visual Error
nexttile
plot(data.Elapsed_Time, data.x_vis_err,'b--', 'DisplayName', 'X Set Point')
hold on
% plot(data.Elapsed_Time, data.visual_mode, 'g--', 'DisplayName', 'Visual Mode')
title('X Visual Error')
ylim([-0.5 0.5])
legend; grid minor; hold off

%Tile five Y visual Error
nexttile
plot(data.Elapsed_Time, data.y_vis_err,'b--', 'DisplayName', 'Y Set Point')
hold on
% plot(data.Elapsed_Time, data.visual_mode, 'g--', 'DisplayName', 'Visual Mode')
title('Y Visual Error')
ylim([-0.5 0.5])
legend; grid minor; hold off

%Tile six Z visual Error
nexttile
plot(data.Elapsed_Time, data.z_vis_err,'b--', 'DisplayName', 'Z Set Point')
hold on
% plot(data.Elapsed_Time, data.Algo_consecutive, 'g--', 'DisplayName', 'Visual Mode')
ylim([-0.5 0.5])
title('Z Visual Error')
legend; grid minor; hold off

%Tile seven X alg setpoint
nexttile
plot(data.Elapsed_Time, data.x_setpoint, 'b--', 'DisplayName', 'X setpoint')
hold on
plot(data.Elapsed_Time, data.V_target_x, 'g--', 'DisplayName', 'X Vis Target')
title('X Visual Error')
% ylim([-0.5 0.5])
legend; grid minor; hold off

%Tile eight Y setpoint
nexttile
plot(data.Elapsed_Time, data.y_setpoint,'b--', 'DisplayName', 'Y setpoint')
hold on
plot(data.Elapsed_Time, data.V_target_y, 'g--', 'DisplayName', 'Y Vis Target')
title('Y Visual Error')
% ylim([-0.5 0.5])
legend; grid minor; hold off

%Tile nine Z setpoint
nexttile
plot(data.Elapsed_Time, data.z_setpoint,'b--', 'DisplayName', 'Z setpoint')
hold on
plot(data.Elapsed_Time, data.V_target_z, 'g--', 'DisplayName', 'Z Vis Target')
% ylim([-0.5 0.5])
title('Z Visual Error')
legend; grid minor; hold off


f = figure('Name', 'Algo Data', 'NumberTitle', 'off');
t = tiledlayout(3,1);
t.TileSpacing = 'compact';
t.Padding = 'compact';

%Algorithm Counter Tiles
nexttile
plot(data.Elapsed_Time, data.Algo_counter, 'b-', 'DisplayName', 'R Algo counter')
hold on
plot(data.Elapsed_Time, data.Vis_counter, 'g-', 'DisplayName', 'V Algo counter')
plot(data.Elapsed_Time, data.Vis_app_cnt, 'r-', 'DisplayName', 'Vis App counter')
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
plot(data.Elapsed_Time, data.safe_radius,'b--', 'DisplayName', 'safe radius')
hold on
plot(data.Elapsed_Time, data.err_mag, 'g--', 'DisplayName', 'err mag')
plot(data.Elapsed_Time, data.fade_increment, 'r--', 'DisplayName', 'fade increment')
title('visual approach trackers')
legend; grid minor; hold off


