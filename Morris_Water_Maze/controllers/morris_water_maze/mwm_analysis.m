% Jeff Krichmar 
% Description: Plots results from webots morris water maze simulation.
% Inputs (need to be loaded from the Webots trial)
%    t - the latencies from the 32 trials.
%    >> t = load('mwm_latency.txt');
%    pc - the locations of the place cells. 
%    >> pc = load('mwm_place.txt');
%    z - the weights of the actor.
%    >> z = load('mwm_z.txt');
% Output
%    time2platform - summation of 4 runs from each starting point
%
function time2platform = mwm_analysis (t, pc, z)

time2platform = zeros(1,size(t,1)/4);
inx = 0;
for i=1:4:size(t,1)
    inx = inx + 1;
    time2platform(inx) = t(i)+t(i+1)+t(i+2)+t(i+3);
end
    
subplot(1,2,1)
plot(time2platform, 'LineWidth', 2)
axis square;
title('Time to Find Platform', 'FontSize',16)
xlabel('Block','FontSize',14)
ylabel('Time Steps','FontSize',14)

subplot(1,2,2)
vectorPlot(z,pc);
axis square
title('Actor Weights','FontSize',16)

end
function vectorPlot (wgts, pc)

dirs = size(wgts,1);
inx = 1;
dx = zeros(sqrt(size(wgts,2)));
dy = zeros(sqrt(size(wgts,2)));

for i = 1:sqrt(size(wgts,2))
    for j = 1:sqrt(size(wgts,2))
        for k = 0:dirs-1
            dx(i,j) = dx(i,j) + wgts(k+1,inx) * cos(2*pi*(k/dirs)+pi);
            dy(i,j) = dy(i,j) + wgts(k+1,inx) * sin(2*pi*(k/dirs)+pi);
            x(i,j) = pc(inx,1);
            y(i,j) = pc(inx,2);
        end
        inx = inx + 1;
    end
end

quiver(x,y,dx,dy)
axis([-2 2 -2 2])
end
























