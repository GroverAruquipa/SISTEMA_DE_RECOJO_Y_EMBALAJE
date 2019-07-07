function [ traj ] = CalcTrajectory( r0,rGoal,m ) % calculo de la trayectoria
traj=[linspace(r0(1),rGoal(1),m);...% generar vector de espacio lineal
        linspace(r0(2),rGoal(2),m);...% generar vector de espacio lineal
        linspace(r0(3),rGoal(3),m)];% generar vector de espacio lineal

end

