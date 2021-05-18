function [yawOrder,vOrder] = avoid_APFmethod(URG_Trans_Length,Obj_Yaw)
%变量说明：
%URG_Trans_length   激光雷达传输回来的每个角度上距离障碍物的距离（mm）
%v 当前无人船航速；
maxRight=0;%定义激光雷达右边最大视野（deg），船体坐标系；
maxLeft=180;%定义激光雷达左边最大视野（deg），船体坐标系；  
n_count=length(URG_Trans_Length);
URG_Trans_length_Temp = zeros(n_count,1);
runFun = zeros(n_count,1);%定义通行函数值变量；
maxEvaltDis = 8000;%定义最大检测范围8米；
minEvaltDis=300;%定义最小检测范围30cm；
stopForceBack = maxEvaltDis*ones(n_count,1);%定义阻力倒数；
porSafeDis = 2000;%定义纵向安全距离2米；
ladSafeDis = 1000;%定义横向安全距离1米；
vmax=5;%定义最大限速(m/s)；
vmin=2;%定义最小限速（m/s)；
%对接收的数据进行初始的处理
for i=1:n_count
    if URG_Trans_Length(i)>maxEvaltDis
        URG_Trans_Length(i)=maxEvaltDis;
    elseif URG_Trans_Length(i)<porSafeDis
        URG_Trans_Length(i)=porSafeDis;
    end 
    URG_Trans_length_Temp(i)=URG_Trans_Length(i);
end 
%计算阻力的倒数
n=0;
for i=1:n_count
    stopForceBack(i)=URG_Trans_length_Temp(i)-porSafeDis;
end
for i=1:n_count
     stopForceBack_temp=URG_Trans_length_Temp(i)-porSafeDis;
     dertafi=asin(ladSafeDis/URG_Trans_length_Temp(i))*57.3;
     while dertafi>=1 %执行while循环使障碍物附近产生与障碍物同样的阻力效应，保障横向安全间距
        if i-dertafi>=0
            if stopForceBack_temp<=stopForceBack(i-floor(dertafi))
                stopForceBack(i-floor(dertafi))=stopForceBack_temp;
            end
        end 
        if i+dertafi<=n_count
            if stopForceBack_temp<=stopForceBack(i+floor(dertafi))
                stopForceBack(i+floor(dertafi))=stopForceBack_temp;
            end
        end 
        dertafi=dertafi-1;
        n=n+1;
     end
end
%计算各个角度的通行值
URG_Yaw = maxRight:1:maxLeft;
for i=1:n_count
    runFun(i)=abs(cos((URG_Yaw(i)-Obj_Yaw)/57.3))*stopForceBack(i);
end
runFun_max=max(runFun);
%决策输出
if runFun_max==0
   vOrder=0;
else
   yawOrder = find(runFun==runFun_max);
   vOrder = (vmax-vmin)*runFun_max/(maxEvaltDis-porSafeDis)+vmin;
end
if min(runFun)<0
runFun_min = min(runFun);
runFun=runFun-runFun_min;
polarplot(URG_Yaw/57.3,-runFun_min*ones(n_count,1),'r');
hold on;
end

polarplot(URG_Yaw/57.3,runFun,'k');



end

