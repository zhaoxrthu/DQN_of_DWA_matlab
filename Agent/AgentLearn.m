function net=AgentLearn(Buffer,net,opts,Const)
%     figure(1);imshow(uint8(SurOb(:,:,1)+SurOb(:,:,2)));
%     figure(2);imshow(uint8(SurObnext(:,:,1)+SurObnext(:,:,2)));
    rank=randperm(Const.BufferSize);
    sa=size(Buffer{1}.s);
    Input=zeros(100,100,Const.BatchSize);
    InputNext=zeros(100,100,Const.BatchSize);
    for i=1:Const.BatchSize
       Input(:,:,i)=imresize(Buffer{rank(i)}.s,[100,100])./255; 
       InputNext(:,:,i)=imresize(Buffer{rank(i)}.snext,[100,100])./255; 
%        figure;imshow(uint8(255*Input(:,:,i)))
%        figure;imshow(uint8(255*InputNext(:,:,i)))
    end
    net=ForwardPro(net,InputNext);
    maxQ=max(net.o);
    net=ForwardPro(net,Input);
    Q=net.o;
    for i=1:Const.BatchSize
       D=Buffer{rank(i)};
       if D.flag
           Q(D.a,i)=D.r;
       else
           Q(D.a,i)=(1-Const.alpha)*Q(D.a,i)+...
           Const.alpha*(D.r+Const.gamma*maxQ(i));
           if (Q(D.a,i)>0.5||Q(D.a,i)<-0.5)
               disp('Over Range!!');
           end
       end
    end
%     Qpre(num,:)=(1-alpha)*Qpre(num,:)+alpha*(y);
    net = BackPro(net, Q);
end

% function [maxQ,maxcur]=GetNextQ(net,SurObnext)
%     input=imresize(SurObnext,[100,100]);
%     input=input-mean(mean(input));
%     net=ForwardPro(net,input);
%     output=net.o;
%     [~,maxcur]=max(output);
%     maxQ=net.o(maxcur,:);
% end