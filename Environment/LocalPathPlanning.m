function [x,traj]=LocalPathPlanning(Ob,goal,x,Kinematic,evalParam,Rmax)
    % DWA参数输入,[u(vt,ot),traj(轨迹)]
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,Ob,Rmax);
    % 机器人移动到下一个时刻
    x=f(x,u);
end

function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
    % Dynamic Window [vmin,vmax,wmin,wmax]
    Vr=CalcDynamicWindow(x,model);
    % 评价函数的计算
    [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);
    if isempty(evalDB)
        disp('no path to goal!!');
        u=[0;0];return;
    end
    % 各评价函数正则化
    evalDB=NormalizeEval(evalDB,R,model(1));
    % 最终评价函数的计算
    % feval=[];
    feval=zeros(1,length(evalDB(:,1)));
    for id=1:length(evalDB(:,1))
    %     feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
        feval(id)=evalParam(1:3)*evalDB(id,3:5)';
    end
    % evalDB=[evalDB feval];
    [~,ind]=max(feval);% 最优评价函数
%     trajBest=trajDB(ind,:);
    u=evalDB(ind,1:2)';%
end

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
    evalDB=[];  trajDB=[];
    for vt=Vr(1):model(5):Vr(2)
        for ot=Vr(3):model(6):Vr(4)
            % 轨迹推测; 得到 xt: 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹
            [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);  %evalParam(4),前向模拟时间;
            if ~isempty(traj)
                % 各评价函数的计算
                heading=CalcHeadingEval(xt,goal);
                dist=CalcDistEval(xt,ob,R);
                vel=abs(vt);
                % 制动距离的计算
                stopDist=CalcBreakingDist(vel,model);
                if dist>stopDist  %
                    evalDB=[evalDB;[vt ot heading dist vel]];
                    trajDB=[trajDB;traj]; % traj = [x,y,yaw,u,w]'
                end
            end
        end
    end
end

function EvalDB=NormalizeEval(EvalDB,Rmax,velmax)
    % 评价函数正则化
    if sum(EvalDB(:,3))~=0
        EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
    end
    if sum(EvalDB(:,4))~=0
        EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
    end
    if sum(EvalDB(:,5))~=0
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
    end
%     EvalDB(:,3)=EvalDB(:,3)./180;
%     EvalDB(:,4)=(EvalDB(:,4)-1)./(Rmax-1);
%     EvalDB(:,5)=EvalDB(:,5)./velmax;
end

function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
    % 轨迹生成函数
    % evaldt：前向模拟时间; vt、ot当前速度和角速度;
    global dt;
    time=0;
    u=[vt;ot];% 输入值
    traj=x';% 机器人轨迹
    while time<=evaldt
        time=time+dt;% 时间更新
        x=f(x,u);% 运动更新
        if(x(1)>800||x(1)<0||x(2)>600||x(2)<0)
            traj=[];
            break;
        end
        traj=[traj x'];
    end
end

function stopDist=CalcBreakingDist(vel,model)
    % 根据运动学模型计算制动距离,这个制动距离并没有考虑旋转速度，不精确吧！！！
    global dt;
    stopDist=0;
    while vel>0
        stopDist=stopDist+vel*dt;% 制动距离的计算
        vel=vel-model(3)*dt;%
    end
end

function dist=CalcDistEval(state,Ob,R)
    % 障碍物距离评价函数
    x=state(2);    y=state(1);
    dist=R;    dlimit=R;
    xmin=max(1,round(x-dlimit));
    xmax=min(600,round(x+dlimit));
    ymin=max(1,round(y-dlimit));
    ymax=min(800,round(y+dlimit));
    for i=xmin:1:xmax
        for j=ymin:1:ymax
            if (Ob(i,j))
                dist=min(dist,sqrt((x-i)^2+(y-j)^2));
            end
        end
    end
%     AroundOb=Ob(xmin:xmax,ymin:ymax);
%     for dist=1:1:R
%         xmin=max(1,round(x-dist));
%         xmax=min(800,round(x+dist));
%         ymin=max(1,round(y-dist));
%         ymax=max(600,round(y+dist));
%         AroundOb=Ob(xmin:xmax,ymin:ymax);
%         if(sum(sum(AroundOb)))
%             break;
%         end
%     end
end

function heading=CalcHeadingEval(x,goal)
% heading的评价函数计算
    theta=mod(toDegree(x(3)),360);% 机器人朝向
    goalTheta=mod(toDegree(atan2(goal(2)-x(2),goal(1)-x(1))),360);% 目标点的方位
    mint=min(theta,goalTheta);
    maxt=max(theta,goalTheta);
    targetTheta=min(maxt-mint,mint+360-maxt);
%     if goalTheta>theta
%         targetTheta=goalTheta-theta;% [deg]
%     else
%         targetTheta=theta-goalTheta;% [deg]
%     end
    heading=180-targetTheta;
end

function Vr=CalcDynamicWindow(x,model)
    %
    global dt;
    % 车子速度的最大最小范围
    Vs=[0 model(1) -model(2) model(2)];
    % 根据当前速度以及加速度限制计算的动态窗口
    Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
    % 最终的Dynamic Window
    Vtmp=[Vs;Vd];
    Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
end

function x = f(x, u)
    % Motion Model
    % u = [vt; wt];当前时刻的速度、角速度
    global dt;
    F = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 0 0;0 0 0 0 0];
    B = [dt*cos(x(3)),0;dt*sin(x(3)),0;0,dt;1,0;0,1];
    x= F*x'+B*u;
    x=x';
end

function radian = toRadian(degree)
    % degree to radian
    radian = degree/180*pi;
end

function degree = toDegree(radian)
    % radian to degree
    degree = radian/pi*180;
end