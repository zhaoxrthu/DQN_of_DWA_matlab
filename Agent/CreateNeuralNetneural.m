function [net,opts]=CreateNeuralNetneural(mode,path,input,output)
	%初始�?mode=1)/加载(mode=2)训练网络
%     figure(1);imshow(input);
    input=imresize(input,[100,100]);
%     figure(2);imshow(input);
	if(mode==1)
%         input = double(reshape(input,28,28))/255;
%         output = double(reshape(output,1,28,10000))/255;
% 		rand('state',0);
		net.layers = {
		    struct('type', 'i') %input layer
		    struct('type', 'c', 'outputmaps', 12, 'kernelsize', 9) %convolution layer
		    struct('type', 's', 'scale', 4) %sub sampling layer
		    struct('type', 'c', 'outputmaps', 24, 'kernelsize', 9) %convolution layer
		    struct('type', 's', 'scale', 5) %subsampling layer
            %struct('type', 'c', 'kernelnums', 36, 'kernelsize', 5)
		};
		net = NetSetUp(net, input, output);
    end
    if(mode==2)
		load(path);
    end
    net.opts.alpha = 1;
	net.opts.batchsize = 50;
	net.opts.numepochs = 1;
end