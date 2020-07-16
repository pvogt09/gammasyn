%CHECKREG m-file to check the function regtr.

clear

vv=version;
if strcmp(vv(1:3),'5.3')
	set(0,'DefaultFigureTool','none')
end

delf

disp(' ')
disp('   enter the name of your data file; files certainly available are :')
disp('   continuous time: ex_c1')
disp('   discrete time  : ex_d1')
disp(' ')
xxx=0;
while xxx==0
	name=input('   name : ','s');
	if isempty(name), return, end
		err=0; eval(name,'err=1;')
	if err==1
		disp(['   **** file ',name,' is not available']), disp(' ')
	else
		xxx=1;
	end
	if isempty(name), return, end
end

infor=input('   do you want information on the design? (1) : ');
if isempty(infor), infor=0; end

nJ=input('   enter the order of the internal model (default 1) : ');
if isempty(nJ), nJ=1; end
if dis==0
	if nJ==1
		J=0;
	else
		J=diag(zeros(1,nJ),0)+diag(ones(1,nJ-1),1);
	end
else
	if nJ==1
		J=1;
	else
		J=diag(ones(1,nJ),0)+diag(ones(1,nJ-1),1);
	end
end

[L,M,N,K] = regtr(A,B,C,J,P1,P2,infor);

cln=input('   do you want to clean the regulator matrix N ? (1) : ');
if isempty(cln), cln=0; end

if cln==1
%
%  cleaning the regulator matrix
%
%  computing the schur form
%
tol=norm(N,'fro')*10^(-6);
%
nr=length(N);
[U,N1]=schur(N);
[U,N1]=rsf2csf(U,N1);
d1=diag(N1);
if dis==0
	rd1=real(d1); %%%%
else
	rd1=abs(d1)-1; %%%%
end
ii=sign(abs(rd1)-tol*ones(nr,1)); in=length(find(ii<0));
[U,Temp]=schord(U,N1,ii);
if in==0
	Xm=zeros(nr,1);
else
	Tm=U(:,1:in); Xm=ima([real(Tm),imag(Tm)]);
end
U=[Xm,ortco(Xm)];
M=U\M;
L=L*U;
N=U\N*U;
%
%  end of cleaning
%
end

A1=A; B1=B; C1=C;
clf
nA1=size(A1,1); mC1=size(C1,1); nB1=size(B1,2); D1=zeros(mC1,nB1);
ordf(3)
if dis==0
	step(A1,B1,C1,D1)
else
	dstep(A1,B1,C1,D1)
end
if ~strcmp(vv(1:3),'5.3')
	uimenu(gcf,'Label','Grid on/off','Callback','grid')
	uimenu(gcf,'Label','Zoom on/off','Callback','zoom')
end
title('open-loop step response')
figure(gcf), pause
nN=size(N,1);
E1=-C1;
A1pl=A1+B1*K*E1;
A3pl=B1*L;
A4pl=-M*E1;
A2pl=N;
Apl=[A1pl A3pl; A4pl A2pl];
Bpl=[zeros(nA1,mC1);M];
Cpl=[E1 zeros(mC1,nN)];
Dpl=zeros(mC1,mC1);
%
figure, ordf(6)
if dis==0
	step(Apl,Bpl,Cpl,Dpl)
else
	dstep(Apl,Bpl,Cpl,Dpl)
end
if ~strcmp(vv(1:3),'5.3')
	uimenu(gcf,'Label','Grid on/off','Callback','grid')
	uimenu(gcf,'Label','Zoom on/off','Callback','zoom')
end
title('closed-loop step response')
figure(gcf), pause
%ordf
if strcmp(vv(1:3),'5.3')
	set(0,'DefaultFigureTool','auto')
end
disp(' ')
% --- last line of checkreg ---