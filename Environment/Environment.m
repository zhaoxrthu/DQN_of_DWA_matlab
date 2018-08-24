function [SurObnext,Reward,x,goal,state]=Environment(evalParam,Kinematic,x,goal,Ob,Const)
% 	Reward=0;
	PreDist=norm(x(1:2)-goal);
	[xnext,~]=LocalPathPlanning(Ob,goal,x,Kinematic,evalParam,10);
    CurDist=norm(x(1:2)-goal);
    Reward=-1/Const.ratio;
    if CurDist>PreDist
        Reward=10*Reward;
    end
    state=0;
    if(norm(xnext(4:5))<1/2*min(Kinematic(5:6)))
%  	    xnext(3)=(rand()-0.5)*2*pi;
        xnext(3)=mod(xnext(3)+pi,2*pi);
	    %disp('Changing the yaw randomly');
	    Reward=-1000/Const.ratio;
        state=-1;
    end
    if(norm(x(1:2)-goal)<10)      
%         disp(['Arrive Goal ',num2str(T),' !!']);
        Reward=3000/Const.ratio;
		%goal=GoalRandGen(xnext(1:2),Ob);
        %T=T+1;
        state=1;
    end
    [SurObnext,~]=GetCurSurOb(xnext,goal,Ob);
	x=xnext;
end