function [SurOb,goalpos]=GetCurSurOb(state,goal,Ob)
	% 得到机器人附近的障碍物信息和与目标点之间的相对位置
    R=100;
    SurObDou=255*ones(2*R+1,2*R+1,2);
    goalpos=[1,1];
    [m,n]=size(Ob);
    x=round(state(2));    y=round(state(1));
    xmin=max(1,x-R);    xmax=min(m,x+R);
    ymin=max(1,y-R);    ymax=min(n,y+R);
    goalx=goal(2);goaly=goal(1);
    if (goalx<=xmax&&goalx>=xmin&&goaly<=ymax&&goaly>=ymin)
        goalpos=[goalx-x+R+1,goaly-y+R+1];
    else
        arc=mod(atan2(goaly-y,goalx-x)-pi/2,2*pi);
        if(arc>=7*pi/4||arc<pi/4)
            detx=-round(R*tan(arc));
            goalpos=[R+1+detx,2*R+1];
        end
        if(arc>=pi/4&&arc<3*pi/4)
            dety=-round(R*(goaly-y)/(goalx-x));
            goalpos=[1,R+1+dety];
        end
        if(arc>=3*pi/4&&arc<5*pi/4)
            detx=-round(R*(goalx-x)/(goaly-y));
            goalpos=[R+1+detx,1];
        end
        if(arc>=5*pi/4&&arc<7*pi/4)
            dety=round(R*(goaly-y)/(goalx-x));
            goalpos=[2*R+1,R+1+dety];
        end
    end
    SurObDou(xmin-x+R+1:xmax-x+R+1,ymin-y+R+1:ymax-y+R+1,1)=Ob(xmin:xmax,ymin:ymax);
    SurObDou(:,:,2)=GetGoalGra(goalpos);
    SurOb=SurObDou(:,:,1)*0.3+SurObDou(:,:,2)*0.7;
    %SubOb(goalpos(2),goalpos(1),2)=0;
%     ShowSur(SurOb(:,:,2),goalpos,state);
end

function Gra=GetGoalGra(goalpos)
    Gra=zeros(201,201);
    for i=1:201
        for j=1:201
            cost=min(255,abs(i-goalpos(1))+abs(j-goalpos(2)));
            Gra(i,j)=uint8(cost);
        end
    end
end

function ShowSur(SurOb,goal,x)
%     figure(2);
    [m,~]=size(SurOb);
    m=(m-1)/2;
    ArrowLength=20;
    clf;
    imshow(uint8(SurOb));hold on;
    plot(goal(2),goal(1),'*r');
%     quiver(m,m,ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    drawnow;
end