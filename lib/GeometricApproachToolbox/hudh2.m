function [sysc,syst,mcost]=hudh2(sysp,H)
%HUDH2    Synthesis of a minimal output H2-norm feedforward compensator.
%  [sysc,syst]=hudh2(sysp,H) provides a ffedforward compensator that
%  minimizes the H2 norm with respect to a disturbance h that enters
%  the plant sysp through a second input matrix H.
%  The overall system syst is also provided as a second output.

[A,B,C,D,Tc]=ssdata(sysp);
n=length(A); nb=size(B,2); mc=size(C,1); nh=size(H,2);
if size(H,1)~=n, error('GEOAPPR:input', 'the row number of H is not correct'), end

if Tc==0
	[X,L,K]=extcare(A,B,C,D); mcost=H'*X*H;
	Ac=A-B*K; Bc=H; Cc=-K; Dc=zeros(nb,nh);
	sysc=ss(Ac,Bc,Cc,Dc);
	sysc=minreal(sysc); %%%%
	[Ac,Bc,Cc,Dc]=ssdata(sysc);
%  [T,L]=eig(Ac); iT=inv(T);
%  Ac=cleanma(iT*Ac*T); Bc=cleanma(iT*Bc); Cc=cleanma(Cc*T);
	sysc=ss(Ac,Bc,Cc,Dc,Tc);
	At=[A -B*K; zeros(n) A-B*K]; Bt=[H;H];
%  Ct=[C-D*K zeros(mc,n)]; Dt=zeros(mc,nh);
	Ct=[C -D*K]; Dt=zeros(mc,nh);
	syst=ss(At,Bt,Ct,Dt);
else
	A1=[A H; zeros(nh,n+nh)]; B1=[B;zeros(nh,nb)]; C1=[C zeros(mc,nh)]; D1=D;
	H1=[zeros(n,nh); eye(nh,nh)];
	[X1,L1,K1]=extdare(A1,B1,C1,D1);
	K=K1(:,1:n); K2=K1(:,n+1:n+nh);
	Ac=A-B*K; Bc=H-B*K2; Cc=-K; Dc=-K2; mcost=sqrt(trace(H1'*X1*H1));
	sysc=ss(Ac,Bc,Cc,Dc,Tc);
	sysc=minreal(sysc); %%%%
	[Ac,Bc,Cc,Dc]=ssdata(sysc);
%  [T,L]=eig(Ac); iT=inv(T);
%  Ac=cleanma(iT*Ac*T); Bc=cleanma(iT*Bc); Cc=cleanma(Cc*T);
	sysc=ss(Ac,Bc,Cc,Dc,Tc);
	At=[A -B*K; zeros(n) A-B*K]; Bt=[H+B*Dc;H+B*Dc];
%  Ct=[C-D*K zeros(mc,n)]; Dt=D*Dc;
	Ct=[C -D*K]; Dt=D*Dc;
	syst=ss(At,Bt,Ct,Dt,Tc);
end
% --- end of hudh2 ---