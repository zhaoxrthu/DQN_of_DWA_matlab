function [w,net,num]=AgentOutput(SurOb,net,eps)
    evaParam=zeros(1,3);
    if(rand()<eps)
        num=min(floor(rand()*36),120)+1;
    else
        input=imresize(SurOb,[100,100])./255;
        net=ForwardPro(net,input);
        output=net.o;
        [~,num]=max(output);         
%         clf;
%      imshow(uint8(255*input(:,:,1)));
%      drawnow;
    end

    w=GetEvaParam(num);
end

function w=GetEvaParam(num)
    x=ceil(num/6);
    y=mod(num,6)+1;
    table=[0.5:0.2:1.5];
    w=[1,table(x),table(y)];
end