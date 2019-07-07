function [ angles ] = CalcTrajectoryAngles( traj,param )
m=size(traj,2);
angles=zeros(3,m);

for i=1:m
    angles(:,i)=InverseKinematics([traj(1,i),traj(2,i),traj(3,i)],param); % calculo de los angulos necesiarios con la trayectoria
end

end

