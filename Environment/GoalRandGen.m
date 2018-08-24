function goal=GoalRandGen(pos,Ob)
    % 随机生成目标点坐标
    [m,n]=size(Ob);
    goal=ones(1,2);
    while(1)
        x=max(ceil(rand()*n)-1,1);
        y=max(ceil(rand()*m)-1,1);
        dist=norm([x-pos(1),y-pos(2)]);
        %bool=SurroundOb([x,y],Ob);
        if(dist>1/4*min(m,n)&&Ob(y,x)==0)
            goal=[x,y];
            return;
        end      
    end
end

function bool=SurroundOb(pos,Ob)
    [m,n]=size(Ob);
    xmin=max(1,pos(1)-3);
    xmax=min(n,pos(1)+4);
    ymin=max(1,pos(2)-3);
    ymax=min(m,pos(2)+4);
    for x=xmin:xmax
        for y=ymin:ymax
            if(Ob(x,y))
               bool=0;
               return;
            end
        end
    end
    bool=1;
end