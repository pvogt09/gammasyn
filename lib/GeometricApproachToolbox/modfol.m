% MODFO H2-optimal feedforward or feedback model following.
% System and model are *.mat files with matrices A,B,C,D and sampling time Tc.
% (Tc=0 in the continuous-time case).

% sysv : LTI original system
% sys1 : LTI modified system
% sys2 : LTI model
% sysc : LTI regulator
% ssst : LTI input-to-error system

clear, close all

vv=version;
if (strcmp(vv(1:3),'5.3'))||(strcmp(vv(1),'6'))||(strcmp(vv(1),'7'))
	set(0,'DefaultFigureTool','none')
end

% Loading a chosen example

flg=0;
while flg==0
	disp(' ')
	disp('   (esmilano, esmilanod)')
	nex=input('   enter the name of the system file : ','s');
	if ~isempty(nex)
		err=0; eval(['load ',nex],'err=1;')
		if err==1
			disp('**** file not available'), return
		else
			flg=1;
		end
	end
end

na=length(A); nb=size(B,2); mc=size(C,1);

if Tc==0, disc=0; else, disc=1; end

fprintf('\n   information on the original controlled system (sysv)\n')

sysv=ss(A,B,C,D,Tc); [STBL,MF,SSC,RHO]=sysinfo1(sysv); pause

if (STBL==1)&&(MF==1)&&(SSC==1)
	fprintf('\n   the controlled system is stable, minimum-phase')
	fprintf('\n   and steady-state controllable')
	fprintf('\n   tracking is perfect and observer is not necessary')
	opsobv=1; sys1=sysv; xxxx=1;
elseif (STBL==0)||(MF==0)||(SSC==0)
	opsobv=[];
	while isempty(opsobv)
		if STBL==0
			fprintf('\n   the system is not stable')
		end
		if MF==0
			fprintf('\n   the system is not minimum-phase')
		end
		if SSC==0
			fprintf('\n   the system is not xteady-state controlleble')
		end
		fprintf('\n   in this case we need a state observer')
		fprintf('\n                            2 - reduced-order observer')
		fprintf('\n                            3 - full-order observer')
	opsobv=input('\n   enter your choice : ');
	end
end

if opsobv==1
	A1=A; B1=B; C1=C; D1=D; syspo=ss(A,B,eye(na),0,Tc); C1def=C;
elseif opsobv==2
	[Ao,Bo1,Bo2,Co,Do1,Do2]=redobs(A,B,C,D);
elseif opsobv==3
	[Ao,Bo1,Bo2,Co,Do1,Do2]=fullobs(A,B,C,D);
end

if (opsobv==2)||(opsobv==3)

	% in (AAo,BBo,CCo,DDo) there is the system plus the observer
	% the output is the estimated state

	no=length(Ao);
	AAo=[A zeros(na,no); Bo2*C Ao];
	BBo=[B; Bo1+Bo2*D];
	CCo=[Do2*C Co];
	DDo=Do1+Do2*D;

	statesys=ss(A,B,eye(na),zeros(na,nb),Tc);
	stateobs=ss(AAo,BBo,CCo,DDo,Tc);
	step(statesys,20), title('the state of the system'), shg, pause
	figure
	step(stateobs,20), title('the estimated state'), shg, pause
	close all

	xxxx=2;
	if STBL==0
		fprintf('\n the controlled system is not stable')
		xxxx=input('\n do you want to set a stabilizing feedback from the observer ? (1) : ');
		if isempty(xxxx), xxxx=2; end
	else
		fprintf('\n the controlled system is stable')
		xxxx=input('\n do you want to set a different set of poles ? (1) : ');
		if isempty(xxxx), xxxx=2; end
	end

	if xxxx==1
		fprintf(' you have to define %2i',na)
		fprintf(' stable eigenvalue(s)\n')
		kk=0;
		while kk<na
			pp=input(' enter a column vector : ');
			if ~isempty(pp)
				kk=length(pp);
			end
		end
		pp=pp(1:na);
		KK=-place(A,B,pp);
		AAF=AAo+BBo*KK*CCo;
	else
		AAF=AAo;
	end

	A1=AAF; B1=BBo; C1=C*CCo; D1=zeros(mc,nb);

	syspo=ss(AAF,BBo,CCo,DDo,Tc); %%%%%%%%

	%A1=A; B1=B; C1=C; D1=D;

	%sys1v=ss(A1,B1,C1,D1,Tc);

	%fprintf('\n   information on the modified controlled system plus observer (sys1)\n')

	%[STBL,MF,SSC,RHO]=sysinfo1(sys1v); pause

	%[At,Bt,Ct,Dt]=ssdata(sys1v);

	%sys1vm=minreal(sys1v); % [A1,B1,C1,D1]=ssdata(sys1);

	%fprintf('\n   information on the minimal realization of the previous one')
	%fprintf('\n   (this may be used for synthesis if minimum-phase)\n')

	%[STBL,MF,SSC,RHO]=sysinfo1(sys1vm); pause

	fprintf('\n   direct derivation of the reference controlled system')
	%fprintf('\n   (this is used for synthesis if minimum-phase)\n')

	if xxxx==1
		sys1=ss((A+B*KK),B,C,D,Tc);
	else
		sys1=sysv;
	end

	[STBL,MF,SSC,RHO]=sysinfo1(sys1); pause

	[A1,B1,C1,D1]=ssdata(sys1); C1def=C1;

	%return

	if (MF==0)||(SSC==0) %%%%%%

		fprintf('\n   since the system is not minimum-phase or not steady-state controllable,')
		fprintf('\n   we consider its stoorvogel equivalent')
		disp(' '), disp(' '), pause

		sys1=stoor(sys1); [A1,B1,C1,D1]=ssdata(sys1); C1def=C1; %%%%%%

		fprintf('\n   information on the stoorvogel equivalent of sys1 (sys1s)\n')

		[STBL,MF,SSC,RHO]=sysinfo1(sys1); pause

	end

end

% ---------------------------------------------------------------------

flg=0; Tc1=Tc;
while flg==0
	disp(' ')
	disp('   (model32n, model32nd)')
	nex=input('   enter the name of the model file  : ','s');
	if ~isempty(nex)
		err=0; eval(['load ',nex],'err=1;')
		if err==1
			disp('   **** file not available'), return
		else
			flg=1;
		end
	end
end

if Tc1~=Tc, disp('   sampling time error'), return, end

A2=A; B2=B; C2=C; D2=D;

fprintf('\n   information on the model (sys2)\n')

sys2=ss(A2,B2,C2,D2,Tc); sysinfo(sys2), pause

%disp(' '), zpk(sys2), pause

na1=length(A1); nb1=size(B1,2); mc1=size(C1,1);
na2=length(A2); nb2=size(B2,2); mc2=size(C2,1);

if size(ima([B1;D1],0))~=nb1
	disp(' ')
	disp('   **** warning: matrix [B1;D1] is not maximum rank'), %return
end

if size(ima([B2;D2],0))~=nb2
	disp(' ')
	disp('   **** warning: matrix [B2;D2] is not maximum rank'), %return
end

if size(ima(C1',0))~=mc1
	disp(' ')
	disp('   **** warning: matrix C1 is not maximum rank'), %return
end

if size(ima(C2',0))~=mc2
	disp(' ')
	disp('   **** warning: matrix C2 is not maximum rank'), %return
end

disp(' ')
feedb=input('   feedback? (1) : ');
if isempty(feedb), feedb=0; end
if feedb~=0, feedb=1; end
disp(' ')

fdthrplant=0;
[n1,n2]=size(D1);
if ~(all((all(D1==zeros(n1,n2)))'))
	disp('   **** warning: feedthrough matrix in the plant'),
	disp(' ')
	fdthrplant=1; % if feedb, return, end
end

fdthrmodel=0;
[n1,n2]=size(D2);
if ~(all((all(D2==zeros(n1,n2)))'))
	disp(' ')
	disp('   **** warning: feedthrough matrix in the model'),
	disp(' ')
	fdthrmodel=1; % if feedb, return, end
end

Vstar1=vstar(A1,B1,C1,D1);
Sstar1=sstar(A1,B1,C1,D1);
RVstar1=ints(Vstar1,Sstar1);
[nrow1,ncol1]=size(RVstar1);
leftinv1=0;
if RVstar1==zeros(nrow1,ncol1)
	leftinv1=1;
end
if ~leftinv1
	disp('   **** warning: the plant is not left-invertible')
	disp(' ')
end

rightinv1=0;
sumsize1=size(sums(Vstar1,Sstar1),2);
if (((na1~=1)&&(sumsize1==na1))||((na1==1)&&(any(sums(Vstar1,Sstar1)))))
	rightinv1=1;
end
if ~rightinv1
	disp('   **** warning: the plant is not right-invertible')
	disp(' ')
else
	rd1=reldeg(A1,B1,C1,D1);
	disp(['   The relative degree of the plant is : ',int2str(rd1)])
	disp(' ')
end

Vstar2=vstar(A2,B2,C2,D2);
Sstar2=sstar(A2,B2,C2,D2);
RVstar2=ints(Vstar2,Sstar2);
[nrow2,ncol2]=size(RVstar2);
leftinv2=0;
if RVstar2==zeros(nrow2,ncol2)
	leftinv2=1;
end
if ~leftinv2
	disp('   **** warning: the model is not left-invertible')
	disp(' ')
end
rightinv2=0;
sumsize2=size(sums(Vstar2,Sstar2),2);
if (((na2~=1)&&(sumsize2==na2))||((na2==1)&&(any(sums(Vstar2,Sstar2)))))
	rightinv2=1;
end
if ~rightinv2
	disp('   **** warning: the model is not right-invertible')
	disp(' ')
	rd2=10^16;
else
	rd2=rhomin(A2,B2,C2,D2);
	disp(['   The minimum delay of the model is : ',int2str(rd2)])
	disp(' ')
end

disp(' '), pause

if feedb
	if nb2~=mc2
		error('GEOAPPR:input', '   **** the model is not square')
	end
	A2old=A2;
	A2=A2+B2*C2;
end

Ahat=[A1 zeros(na1,na2); zeros(na2,na1) A2];
Bhat=[B1;zeros(na2,nb1)];
Hhat=[zeros(na1,nb2);B2];
Chat=[C1 -C2];
Dhat=D1;
Ghat=-D2;

syshat=ss(Ahat,Bhat,Chat,Dhat,Tc);
[sysc,err]=hud(syshat,Hhat,Ghat);
%[Ac,Bc,Cc,Dc,err]=hud(Ahat,Bhat,Chat,Hhat,Dhat,Ghat,Tc);
%[Ac,Bc,Cc,Dc,err]=hud(Ahat,Bhat,Chat,Hhat,Tc);

%if err, disp('   error in hud'), return, end

[Ac,Bc,Cc,Dc,Tc]=ssdata(sysc);

cln=1;
if cln==1
	N=Ac; M=Bc; L=Cc; K=Dc;
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
	rd1=abs(d1)-1; %%%%
	ii=sign(abs(rd1)-tol*ones(nr,1)); in=length(find(ii<0));
	[U,Temp]=schord(U,N1,ii);
	if in==0
		Xm=zeros(nr,1);
	else
		Tm=U(:,1:in); Xm=ima([real(Tm),imag(Tm)]);
	end
	U=ima([Xm,eye(size(Xm,1))],0);
	M=U\M;
	L=L*U;
	N=U\N*U;
	%
	%  end of cleaning
	%
	Ac=N; Bc=M; Cc=L; Dc=K;
end

sysc=ss(Ac,Bc,Cc,Dc,Tc);
sysc=minreal(sysc);

fprintf('\n   information on the regulator (sysc)\n')

sysinfo(sysc), pause

disp(' ')

nac=length(Ac); nbc=size(Bc,2); mcc=size(Cc,1);

%A1=A1old; B1=B1old; C1=C1old; D1=D1old;
%na1=length(A1); nb1=size(B1,2); mc1=size(C1,1);

%return

disp(' ')
disp(' ---------------------------------------------------------------')

fprintf('\n   checking the overall control system\n\n'), pause

if feedb
	A2=A2old;
end

flg=0;
while flg==0
	ttt=input('\n   enter an optional simulation time : ');
	flg=1;
	if isempty(ttt)
	if disc, ttt=100; else, ttt=20; end
	end
end
disp(' ')

figure
%
step(sys1,ttt)
if xxxx==1
	title(' step response of the original controlled system')
else
	title(' step response of the controlled system with observer and feedback')
end
shg, pause
figure
step(sys2,ttt)
title('step response of the model')
shg, pause
[A,B,C,D]=ssdata(sysv);
sysvv=syspo*sysc;
if feedb
	fee=ss([],[],[],C1def,Tc);
	sysdef=feedback(sysvv,fee);
	[AA,BB,CC,DD]=ssdata(sysdef);
	syscom=ss(AA,BB,C*CC,0,Tc);
	%onesxx=ss([],[],[],eye(mc),Tc);
	%shreg=sysc*(onesxx-fee*sysdef);
	figure
	step(syscom,ttt)
	title('step response of the overall feedback system')
	shg, pause
	if size(sums(vstar(sysv),sstar(sysv)),2)==na1
		syserror=syscom-sys2;
		figure
		step(syserror,ttt)
		title('tracking error')
		shg, pause
		H2norm=abs(norm(syserror));
		fprintf('\n   l2-norm of the tracking error : %.4g\n',H2norm), disp(' ')
	end
else
	[AA,BB,CC,DD]=ssdata(sysvv);
	syscom=ss(AA,BB,C*CC,0,Tc);
	shreg=sysc;
	figure
	step(syscom,ttt)
	title('step response of the overall feedfrorward system')
	shg, pause
		if size(sums(vstar(sysv),sstar(sysv)),2)==na1
		syserror=syscom-sys2;
		%syserror=pinv(dcgain(syscom))*syscom-sys2;
		%syserror=dcgain(syscom)'*syscom-sys2;
		figure
		step(syserror,ttt)
		title('tracking error')
		shg, pause
		H2norm=abs(norm(syserror));
		fprintf('\n   l2-norm of the tracking error : %.4g\n',H2norm), disp(' ')
		end
end

% --- end of modfol ---