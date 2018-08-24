function []=DQN()
    clear all;clc;
    addpath(genpath('Environment'));
    addpath(genpath('Agent'));
    % 加载环境信息，像素255表示障碍物情况
    Ob=imread('Ob.png');Ob=Ob(:,:,1);
    % 机器人运动学模型:最高速度[pixel/s],最高旋转速度[rad/s],加速度[pixel/ss],
    % 旋转加速度[rad/ss],速度分辨率[pixel/s],转速分辨率[rad/s]]
    Kinematic=[100,1.2,80,0.6,3,0.08];
    % 评价函数参数 [heading,dist,velocity,predictDT]
    evalParam=[ones(1,3),1.5];
    % 最大距离阈值
    stochRmax=10;
    % 时间间隔
    global dt;dt=0.2;
    % 训练轮数/每轮迭代点数
    E=500;M=20;
    % 初始化当前状态、初始化目标点
    % 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
    % 初始化训练网络
    path='Model/model_0717_1209.mat';
    x=InitialState(Ob);
    goal=GoalRandGen(x(1:2),Ob);
    [SurOb,~]=GetCurSurOb(x,goal,Ob);
    net=CreateNeuralNetneural(1,path,SurOb,zeros(36,1));

    % 状态情况存储
    StochQmean=zeros(E,1);TrainRmean=zeros(E,M+1);
    StochCount=zeros(E,1);TrainCount=zeros(E,M+1);
    StochFail=zeros(E,1);TrainFail=zeros(E,M+1);
    State.x=zeros(M,5);State.goal=zeros(M,2);
    % eps-greedy
    Const.eps=0.5;
    % 退火系数
    Const.anneal=0.95;
    % 缓存区大小和batch大小
    Const.BufferSize=50;    Const.BatchSize=10;
    net.opts.batchsize=Const.BatchSize;
    % DQN中的学习率和折扣率
    Const.alpha=0.8;    Const.gamma=0.95;
    Const.ratio=10000;

    % 初始化并储存每一轮的起始点/目标点信息
    for m=1:M
        x=InitialState(Ob);
        goal=GoalRandGen(x(1:2),Ob);
        State.x(m,:)=x;
        State.goal(m,:)=goal;
    end

    % 主训练程序
    for e=1:E
        tic
        % 退火
        Const.eps=Const.eps*Const.anneal;
        % 打乱起始点/目标点的顺序
        rank=randperm(M);
        % 首先用随机参数前进一次
        [StochQmean(e),StochCount(e),StochFail(e)]=StochParamGetQmean(State,Kinematic,rank,Ob,net,Const);
        % 开始学习
        [TrainRmean(e,:),TrainCount(e,:),TrainFail(e,:),net]=Train(State,Kinematic,rank,Ob,net,Const);
        % 打印并存储本次训练结果
        DisplayAndStore(net,StochQmean,StochCount,StochFail,TrainRmean,TrainCount,TrainFail,e,E,rank(1));
        toc
    end
    time=clock();timestr=cell(5);
    for j=2:5
       timestr{j}=['0',num2str(time(j))]; 
       timestr{j}=timestr{j}(end-1:end);
    end
    str=['Data_',timestr{2},timestr{3},'_',timestr{4},timestr{5}];
    save([str,'.mat']);
end

function [Qmean,CurCount,FailCount]=StochParamGetQmean(State,Kinematic,rank,Ob,net,Const)
    x=State.x(rank(1),:);
    goal=State.goal(rank(1),:);
    evalParam=[ones(1,3),1.5];
    evalParam(1:3)=[0.5,rand(),rand()]+0.5;
    [SurOb,~]=GetCurSurOb(x,goal,Ob);
    CurCount=0;FailCount=0;Q=0;
     while (1)
         CurCount=CurCount+1;
         % 得到Agent当前的输出
         [~,net,~]=AgentOutput(SurOb,net,0);
         Q=Q+1000*max(net.o);
         % 使用当前的参数，得到下一个状态
         [SurOb,~,x,goal,state]=Environment(evalParam,Kinematic,x,goal,Ob,Const); 
         if(state==1)
             break;
         end
         if(state==-1)
             FailCount=FailCount+1;
         end
     end
     Qmean=Q/CurCount;
end

function [TrainRmean,TrainCount,TrainFail,net]=Train(State,Kinematic,rank,Ob,net,Const)
    M=length(rank);
    SumCount=0;
    evalParam=[ones(1,3),1.5];
    TrainRmean=zeros(1,M+1);
    TrainCount=zeros(1,M+1);
    TrainFail=zeros(1,M+1);        
    Buffer=cell(1,Const.BufferSize);
    for m=1:M
        % 清空缓存区
        i=0;R=0;
        x=State.x(rank(m),:);
        goal=State.goal(rank(m),:);
        [SurOb,~]=GetCurSurOb(x,goal,Ob);
        while(1)
            i=i+1;
            SumCount=SumCount+1;
            % 得到Agent当前的输出
            [evalParam(1:3),net,num]=AgentOutput(SurOb,net,Const.eps);
            % 使用当前的参数，得到下一个状态
            [SurObnext,Reward,x,goal,state]=Environment(evalParam,Kinematic,x,goal,Ob,Const);
            R=R+Const.ratio*Reward;            
            % 将这一次操作的过程存入缓存区
            D.s=SurOb;D.a=num;D.r=Reward;D.snext=SurObnext;D.flag=state;
            Buffer{Const.BufferSize-mod(SumCount,Const.BufferSize)}=D;            
            % 从缓存中选择状态，迭代更新学习网络
            if (SumCount>Const.BufferSize && mod(i,5)==0)
                net=AgentLearn(Buffer,net,net.opts,Const);
            end
            SurOb=SurObnext;
            if(state==1)
                break;
            end
            if(state==-1)
                TrainFail(rank(m))=TrainFail(rank(m))+1;
            end                            
        end
        TrainRmean(rank(m))=R;
        TrainCount(rank(m))=i;        
    end
    TrainRmean(end)=mean(TrainRmean(1:end-1));
    TrainCount(end)=sum(TrainCount(1:end-1));
    TrainFail(end)=sum(TrainFail(1:end-1));
end

function []=DisplayAndStore(net,StochQmean,StochCount,StochFail,TrainRmean,TrainCount,TrainFail,e,E,num)
%     存储当前模型
    time=clock();
    timestr=cell(5);
    for j=2:5
       timestr{j}=['0',num2str(time(j))]; 
       timestr{j}=timestr{j}(end-1:end);
    end
    str=['model_',timestr{2},timestr{3},'_',timestr{4},timestr{5}];
    save(['Model/',str,'.mat'],'net');
    % 打印当前结果
    disp([num2str(e),'/',num2str(E),':   ']);
    disp(['StochCount/TrainCount:   ',num2str(StochCount(e)),'/',num2str(TrainCount(e,num))]);
    disp(['StochFail/TrainFail:   ',num2str(StochFail(e)),'/',num2str(TrainFail(e,num))]);
    disp(['StochQmean:',num2str(StochQmean(e)),'   ,','TrainRmean:',num2str(TrainRmean(e,end))]);
    % 画图
    figure(1);clf;
    xl=(1:e);
    subplot(2,1,1);
    plot(xl,StochQmean(1:e),'r'); legend('Average of Qmax');
    subplot(2,1,2);
    plot(xl,TrainRmean(1:e,end),'r'); legend('Average of R');
    drawnow;
    % 画图
    figure(2);clf;
    subplot(2,1,1);
    plot(xl,TrainCount(1:e,end),'r'); legend('Sum of TrainCount');
    subplot(2,1,2);
    plot(xl,TrainFail(1:e,end),'r'); legend('Sum of TrainFail');
    drawnow;
end