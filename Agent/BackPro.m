function net = BackPro(net, y)
    n = numel(net.layers);
    %   error
    net.e = net.o - y;
    %  loss function
    net.L = 1/2* sum(net.e(:) .^ 2) / size(net.e, 2);
    %  backprop deltas
     net.od = net.e .* ((0.5+net.o) .* (0.5-net.o));
    %net.od = net.e .* (net.o .* (1 - net.o));   %  output delta
    net.fvd = (net.ffW' * net.od);              %  feature vector delta
    if strcmp(net.layers{n}.type, 'c')         %  only conv layers has sigm function
        net.fvd = net.fvd .* (net.fv .* (1 - net.fv));
    end
    %  reshape feature vector deltas into output map style
    sa = size(net.layers{n}.a{1});
    fvnum = sa(1) * sa(2);
    for j = 1 : numel(net.layers{n}.a)
        net.layers{n}.d{j} = reshape(net.fvd(((j - 1) * fvnum + 1) : j * fvnum, :), sa(1), sa(2), sa(3));
    end
    for l = (n - 1) : -1 : 1
        if strcmp(net.layers{l}.type, 'c')
            for j = 1 : numel(net.layers{l}.a)
                net.layers{l}.d{j} = net.layers{l}.a{j} .* (1 - net.layers{l}.a{j}) .* (expand(net.layers{l + 1}.d{j}, [net.layers{l + 1}.scale net.layers{l + 1}.scale 1]) / net.layers{l + 1}.scale ^ 2);
            end
        elseif strcmp(net.layers{l}.type, 's')
            for i = 1 : numel(net.layers{l}.a)
                z = zeros(size(net.layers{l}.a{1}));
                for j = 1 : numel(net.layers{l + 1}.a)
                     z = z + convn(net.layers{l + 1}.d{j}, rot180(net.layers{l + 1}.k{i}{j}), 'full');
                end
                net.layers{l}.d{i} = z;
            end
        end
    end
    %  calc gradients
    for l = 2 : n
        if strcmp(net.layers{l}.type, 'c')
            for j = 1 : numel(net.layers{l}.a)
                for i = 1 : numel(net.layers{l - 1}.a)
                    net.layers{l}.dk{i}{j} = convn(flipall(net.layers{l - 1}.a{i}), net.layers{l}.d{j}, 'valid') / size(net.layers{l}.d{j}, 3);
                end
                net.layers{l}.db{j} = sum(net.layers{l}.d{j}(:)) / size(net.layers{l}.d{j}, 3);
            end
        end
    end
    net.dffW = net.od * (net.fv)' / size(net.od, 2);
    net.dffb = mean(net.od, 2);

    for l = 2 : numel(net.layers)
        if strcmp(net.layers{l}.type, 'c')
            for j = 1 : numel(net.layers{l}.a)
                for ii = 1 : numel(net.layers{l - 1}.a)
                    net.layers{l}.k{ii}{j} = net.layers{l}.k{ii}{j} - net.opts.alpha * net.layers{l}.dk{ii}{j};
                end
                net.layers{l}.b{j} = net.layers{l}.b{j} - net.opts.alpha * net.layers{l}.db{j};
            end
        end
    end

    net.ffW = net.ffW - net.opts.alpha * net.dffW;
    net.ffb = net.ffb - net.opts.alpha * net.dffb;
end

function X = rot180(X)
    X = flipdim(flipdim(X, 1), 2);
end

function B = expand(A, S)
    if nargin < 2
        error('Size vector must be provided.  See help.');
    end
    SA = size(A);  % Get the size (and number of dimensions) of input.
    if length(SA) ~= length(S)
       error('Length of size vector must equal ndims(A).  See help.')
    elseif any(S ~= floor(S))
       error('The size vector must contain integers only.  See help.')
    end
    T = cell(length(SA), 1);
    for ii = length(SA) : -1 : 1
        H = zeros(SA(ii) * S(ii), 1);   %  One index vector into A for each dim.
        H(1 : S(ii) : SA(ii) * S(ii)) = 1;   %  Put ones in correct places.
        T{ii} = cumsum(H);   %  Cumsumming creates the correct order.
    end
    B = A(T{:});   %  Feed the indices into A.
end

function X=flipall(X)
    for i=1:ndims(X)
        X = flipdim(X,i);
    end
end