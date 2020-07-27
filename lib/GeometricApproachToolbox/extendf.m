function [Am,Bm,Cm,Dm,Fs,Us]=extendf(A,B,C,D)
%EXTENDF  Squaring down for non left-invertible systems.
%   [Am,Bm,Cm,Dm,Fs,Us]=extendf(A,B,C,D) computes Fs and Us such that the
%   quadruple (Am,Bm,Cm,Dm) is left-invertible. This quadruple is defined by
%   Am=A+B*Fs; Bm=B*Us; Cm=C+D*Fs; Dm=D*Us.
%   When a state feedback matrix Fm referred to (Am,Bm,Cm,Dm) has been
%   derived, the solution referred to (A,B,C,D) is recovered by using
%   F=Us*Fm+Fs.

na=length(A); nb=size(B,2); mc=size(C,1);
flg=0; if norm(D,'fro')<eps, flg=1; end
if flg==1
	Abar1=A; Bbar1=B; Cbar1=C;
else
	Abar1=[A, zeros(na,mc); C, zeros(mc)];
	Bbar1=[B;D];
	Cbar1=[zeros(mc,na),eye(mc)];
end
%Aold=A; Bold=B; Cold=C; Dold=D;
VV=vstar(Abar1,Bbar1,Cbar1);
SS=sstar(Abar1,Bbar1,Cbar1);
RV=ints(VV,SS); if ~any(RV), RV=[]; end
leftin=0;
if size(RV,2)>0
	Fs=effesta(Abar1,Bbar1,RV);
	Fs=Fs(:,1:na);
	Am=A+B*Fs;
	Cm=C+D*Fs;
	Us=ortco(invt(Bbar1,VV));
	Bm=B*Us;
	Dm=D*Us;
else
	Fs=zeros(nb,na);
	Us=eye(nb);
	Am=A; Bm=B; Cm=C; Dm=D;
end
% --- last line of extendf ---