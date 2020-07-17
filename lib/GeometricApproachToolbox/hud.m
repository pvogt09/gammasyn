function [Ac,Bc,Cc,Dc,err]=hud(A,B,C,H,D,G,Tc)
%HUD      Synthesis of a feedforward decoupling compensator.
%  [Ac,Bc,Cc,Dc,err]=hud(A,B,C,H,Tc) or
%  [Ac,Bc,Cc,Dc,err]=hud(A,B,C,H,D,G,Tc) provides a feedforward compensator
%  (Ac,Bc,Cc,Dc) that, for the continuous-time system (Tc=0)
%
%      dot x(t) = A x(t) + B u(t) + H h(t)
%          y(t) = C x(t) + D u(t) + G h(t)
%
%  or the discrete-time system (Tc~=0)
%
%        x(k+1) = A x(k) + B u(k) + H h(k)
%          y(k) = C x(k) + D u(k) + G h(k)
%
%  produces decoupling between input h and output y. The flag err is set
%  equal to 1 if the program ends with an error message.
%  The LTI system call: [sysc,err]=hud(sys,H,[G]) is also possible.

mm=' press any key to continue';
Ac=[]; Bc=[]; Cc=[]; Dc=[]; err=0; disc=[];
tol=eps*10^4;

if (nargin~=2)&&(nargin~=3)&&(nargin~=5)&&(nargin~=7)
	disp(' **** error in input arguments')
	if nargin>3, err=1; else, Bc=1; return, end
end

if (nargin==2)
	H=B; [A,B,C,D,Tc]=ssdata(A);
elseif (nargin==3)
	H=B; G=C; [A,B,C,D,Tc]=ssdata(A);
end

na=length(A); nb=size(B,2); mc=size(C,1); nh=size(H,2);

if nargin==2
	G=zeros(mc,nh);
end

if nargin==5
	Tc=D; D=zeros(mc,nb); G=zeros(mc,nh);
end

flg=0;
if (nargin==7)||(nargin==3)
	if size(ima([B;D],0))~=nb
		disp(' **** matrix [B;D] is not maximum rank')
		if nargin>3, err=1; else, Bc=1; return, end
	end
	if (norm(D,'fro')>eps)||(norm(G,'fro')>eps), flg=1; end
else
	if size(ima(B,0))~=nb
		disp(' **** matrix B is not maximum rank')
		if nargin>3, err=1; else, Bc=1; return, end
	end
end

if size(ima(C',0))~=mc
	disp(' **** matrix C is not maximum rank')
	if nargin>3, err=1; else, Bc=1; return, end
end

if flg
	A=[A zeros(na,mc); C zeros(mc,mc)];
	B=[B;D]; H=[H;G];
	C=[zeros(mc,na) eye(mc)];
end

V=vstar(A,B,C);

if size(ima([V,B,H],0),2) > size(ima([V,B],0),2)
	disp(' **** H is not contained in Vstar+B')
	if nargin>3, err=1; else, Bc=1; end
end

z=gazero(A,sums(B,H),C);

if Tc==0, disc=0; else, disc=1; end,

Vm=ints(V,sstar(A,sums(B,H),C));
if ~any(Vm), Vm=[]; end

z=gazero(A,B,ortco(Vm)');

if disc
	ii=find(abs(z)>1+tol);
else
	ii=find(real(z)>0+tol);
end

if ~isempty(ii)
	disp(' **** warning: Vm is not internally stabilizable')
	if nargin>3, err=1; else, Bc=1; end
end

[na,nv]=size(Vm);

if nv~=0
	XU=pinv([Vm B])*A*Vm;
	KK=ker([Vm B]);
	if ~any(KK), KK=[]; end
	W=XU(1:nv,:);
	if isempty(KK)
		L=XU(nv+1:nv+nb,:);
		Rv=[];
		V1=Vm;
	else
		Rv=rvstar(A,B,C);
		ncol=size(Rv,2);
		V1=ima([Rv Vm],0);
		XU=pinv([V1 B])*A*V1;
		AA=XU(1:ncol,1:ncol);
		KK=ker([V1 B]);
		BB=KK(1:ncol,:);
		disp(' **** the system is not left-invertible')
		disp(' ')
		fprintf(' you have to define %2i',ncol)
		fprintf(' eigenvalue(s)\n')
		disp(' ')
		kk=0; pa=[];
		while kk<ncol
			pp=input(' enter a stable value for a free zero : ');
			if ~isempty(pp), pa=[pa;pp]; kk=kk+1; end
		end
		disp(' ')
		FF=-place(AA,BB,pa);
		XU(:,1:ncol)=XU(:,1:ncol)+KK*FF;
		W=XU(1:nv,:);
		L=XU(nv+1:nv+nb,:);
	end
	alfabeta=pinv([V1 B])*H;
	alfa=alfabeta(1:nv,:);
	beta=alfabeta(nv+1:nv+nb,:);
	Ac=W; Cc=-L; Bc=alfa; Dc=-beta;
else
	Dc=-pinv(B)*H;
	Ac=0; Bc=zeros(1,nh); Cc=zeros(nb,1);
end

if nargin<=3
	Ac=ss(Ac,Bc,Cc,Dc,Tc); Bc=err; Cc=[]; Dc=[];
end

% --- last line of hud ---