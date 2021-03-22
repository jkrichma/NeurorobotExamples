function plot_trajectory(trialNum)

tbl=readtable(['trial_metrics_', num2str(trialNum), '_early.txt']);
pos = table2array(tbl(:,6:7));
subplot(1,2,1)
scatter(pos(:,1),pos(:,2),'.')
axis([-0.5 0.5 -0.5 0.5])
axis square
title(['Trial ', num2str(trialNum), ' - Early'])
xlabel('X')
ylabel('Z')

tbl=readtable(['trial_metrics_', num2str(trialNum), '_late.txt']);
pos = table2array(tbl(:,6:7));
subplot(1,2,2)
scatter(pos(:,1),pos(:,2),'.')
axis([-0.5 0.5 -0.5 0.5])
axis square
title(['Trial ', num2str(trialNum), ' - Late'])
xlabel('X')
ylabel('Z')