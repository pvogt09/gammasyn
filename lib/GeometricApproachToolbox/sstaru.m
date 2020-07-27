function [u,xu] = sstaru(A,B,E,x,ro)
%SSTARU   Control sequence on Sstar = minS(A,ker(E),im(B)).
%  [u,xu] = sstaru(A,B,E,x,ro) is a control sequence to reach the state x
%  belonging to the ro-th step computational subspace of Sstar in ro steps.

%  D.P. 11-28-99

[S,rr]=miinco(A,ker(E),B); romax=length(rr);

if nargin==5
	if ro > romax
		error('GEOAPPR:input', ' **** ro greater than the number of steps of Sstar in SSTARU')
	end
elseif nargin==4
	ro=romax;
elseif nargin < 4
	error('GEOAPPR:input', ' **** not enough input arguments in SSTARU')
end

[n,nb]=size(B); nx=size(x,2);
QC=[]; nQC=[];
nv=0; ni=[];
X1=ima(B,0); C=ker(E);
Q=X1; QC=X1; nQC=size(X1,2);
[mq,nq]=size(Q);

for kk=1:ro-1
	ni=[ni,nq];
	nv=nq;
	Q=ima([X1 A*ints(Q,C)],0);
	nq=size(Q,2);
	if nq > nv
		QC=[QC,Q]; nQC=[nQC,size(Q,2)];
	end
end

if size(ima([Q,x]),2) > size(ima(Q),2)
	warning('GEOAPPR:input', ' **** x is not in Sro')
end

ll=length(nQC); nl=nQC(ll); nQC=nQC(1:ll-1);
ll1=size(QC,2); Sn=QC(:,ll1-nl+1:ll1); QC=QC(:,1:ll1-nl);
alfac=pinv(Sn)*x;
u=[]; xu=Sn*alfac; Sn_1C=Sn;

for kk=1:ro-1
	ll=length(nQC); nl=nQC(ll); nQC=nQC(1:ll-1);
	ll1=size(QC,2); Sn_1=QC(:,ll1-nl+1:ll1); QC=QC(:,1:ll1-nl);
	Sn_1C=ints(Sn_1,C); if (size(Sn_1C,2)==1)&&(~any(Sn_1C)), Sn_1C=[]; end
	UU=pinv([B A*Sn_1C])*Sn*alfac;
	u=[UU(1:nb,:),u];
	alfac=UU(nb+1:size(UU,1),:);
	xu=[Sn_1C*alfac,xu];
	Sn=Sn_1C;
end

u=[pinv(B)*Sn_1C*alfac,u]; xu=[zeros(n,nx),xu];

% --- last line of sstaru ---