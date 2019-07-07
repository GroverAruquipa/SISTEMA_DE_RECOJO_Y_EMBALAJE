function [] = Animation( angles,traj,param )
%dibuja la animacion
%cuidado trayectoria cinematica directa


%simulacion paso por paso
N=size(angles,2);% calculo de animacion
dt=0.1;

for i=1:N 
    tic
    
    PlotPosition(traj(:,i),angles(:,i),param); % dibujamos posicion
    
    toc
    pause(dt); % pausas diferencial de tiempo
end

end

