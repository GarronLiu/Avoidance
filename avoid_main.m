URG_Trans_Length=8000*ones(181,1)-rand(181,1);
%设定无人船通过激光雷达检测到有两堵墙
for i=30:50
    URG_Trans_Length(i)=2500;
end 
for i=110:140
    URG_Trans_Length(i)=5000;
end
Obj_Yaw=45;
%假定当前目标点在船体坐标系45°方位
[yawOrder,vOrder] = avoid_APFmethod(URG_Trans_Length,45);
hold on;
angle=30:50;
polarplot(angle/57.3,5000*ones(16,1),'b');%绘制障碍物A
hold on;
angle=110:140;
polarplot(angle/57.3,5000*ones(31,1),'b');%绘制障碍物B