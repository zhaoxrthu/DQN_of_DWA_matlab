function state=InitialState(Ob)
	[m,n]=size(Ob);
	state=zeros(1,5);
	while(1)
        x=max(ceil(rand()*n)-1,1);
        y=max(ceil(rand()*m)-1,1);
        if(Ob(y,x)==0)
            state(1:2)=[x,y];
            return;
        end      
	end
end