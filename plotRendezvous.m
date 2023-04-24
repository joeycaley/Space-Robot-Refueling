% plot time to intercept
figure(1)
plot(k,tElapsedTotal./3600,'b','LineWidth',1.5)
xlim([k(1) k(end)])
xlabel("k")
ylabel("Time (hr)")
title("Total Time to Reach Serviced Spacecraft")
grid on

% plot delta V required
figure(2)
plot(k,deltaVTotal,'b','LineWidth',1.5)
xlim([k(1) k(end)])
xlabel("k")
ylabel("\Deltav Required (m/s)")
title("Total \Deltav to Reach Serviced Spacecraft")
grid on

if animationCheck == 1
    % plot initial orbit change
    pause(.1)
    figure(3)
    pause(.1)
    for i = 1:length(thetaTrav)
        figure(3)
        scatter3(0,0,0,300,'filled')
        title("Plane Change Manuever")
        hold on
        plot3(xTrav(1:i),yTrav(1:i),zTrav(1:i),'b','LineWidth',1.5)
        hold off
        grid on
        pause(.05)
    end
    hold on
    grid on
    quiver3(xTrav(end),yTrav(end),zTrav(end),vx,vy,vz,'r','LineWidth',1.5)
    legend(["Earth", "Original Orbit", "Velocity After Plane Change"])
    xlim([-1e07 1e07])
    ylim([-1e07 1e07])
    zlim([-1e07 1e07])
    
    input("Press any key to continue\n")

    % plot phasing manuever
    pause(.1)
    figure(4)
    pause(.1)
    for i = 1:length(ft)
        figure(4)
        plot(xog(1:i),yog(1:i),'b','LineWidth',3)
        hold on
        title("Phasing Manuever")
        plot(xt(1:i),yt(1:i),'r','LineWidth',3)
        scatter(xog(i),yog(i),100,'b','filled')
        scatter(xt(i),yt(i),100,'r','filled')
        hold off
        grid on
        axis square
        legend(["Goal Spacecraft", "Servicing Spacecraft"])
        xlim([-1e07 1e07])
        ylim([-1e07 1e07])
        pause(.05)
    end
    

end