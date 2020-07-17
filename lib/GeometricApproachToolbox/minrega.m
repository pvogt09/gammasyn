function sysm=minrega(sysf,tol)
%MINREGA  Minimal realization with the geometric approach routines.
%  sysm=minrega(sys[,tol]) provides as sysm the minimal realization of sys.

[A,B,C,D,Tc]=ssdata(sysf);
if nargin==1, tol=norm(A,'fro')*eps*10^6; end
[Ak,Bk,Ck,Dk,n1,n2,n3,n4]=kalmcd(A,B,C,D,tol);
Am=Ak(n1+1:n1+n2,n1+1:n1+n2);
Bm=Bk(n1+1:n1+n2,:);
Cm=Ck(:,n1+1:n1+n2);
Dm=Dk;
redust=length(A)-length(Am);
fprintf('%2i',redust), fprintf(' state(s) removed\n')
sysm=ss(Am,Bm,Cm,Dm,Tc);

% --- end of minrega ---