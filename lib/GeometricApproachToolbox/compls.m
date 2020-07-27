function Jc=compls(A,J)

%COMPLS   Complementation of an invariant.
%  Jc=COMPLS(A,J) gives as Jc an orthonormal basis for the complement of the A-invariant J.

%  G. Marro 4-20-10

n=length(A); nJ=size(J,2);
JJ=mininv(A,J); JJ=sums(JJ,J);
if (size(JJ,2)~=nJ)
	disp(' *** J is not and A-invariant'), return
end
T=ima([J,ortco(J)],0);
A1=T\A*T;
A11=A1(1:nJ,1:nJ);
A22=A1(nJ+1:n,nJ+1:n);
A12=A1(1:nJ,nJ+1:n);
X=lyap(A11,-A22,A12);
T2=T(:,nJ+1:n);
Jc=ima(J*X+T2);

% --- end of compls ---