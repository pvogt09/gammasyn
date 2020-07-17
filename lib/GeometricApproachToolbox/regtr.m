function [L,M,N,K] = regtr(A1,B1,C1,J,P1,P2,infor)
%REGTR    Regulator design with the geometric approach tools.
%   [L,M,N,K] = regtr(A,B,C,J,P1,P2,[infor])
%   A,B,C the matrices of the plant
%   J the Jordan block of the internal model
%   P1 the eigenvalues to be assigned by state feedback (nP1=nA)
%   P2 the eigenvalues to be assigned by output injection (nP2=nA+mE*nJ)
%   infor: if present and =1 some information on design is displayed.

%   Basile and Marro 4-20-90 - Version for Matlab 5 (1998)

xx=' press any key to continue';
nA1=size(A1,1); nB1=size(B1,2);
E1=-C1;
mE1=size(E1,1); nJ=size(J,1);
if nargin==6, infor=0; end
if length(P1)<nA1
	error('GEOAPPR:input', ' **** not enough eigenvalues in P1 (regtr)')
end
if length(P2)<nA1+mE1*nJ
	error('GEOAPPR:input', ' **** not enough eigenvalues in P2 (regtr)')
end
P1=P1(1:nA1); P2=P2(1:nA1+mE1*nJ);
if infor
	disp(' '), disp(' the matrices of the system')
	A1, disp(xx), pause
	B1, disp(xx), pause
	C1, disp(xx), pause
	disp(' '), disp(' the matrix of the internal model')
	J, disp(xx), pause
	disp(' '), disp(' the eigenvalues to be assigned')
	P1, disp(xx), pause
	P2, disp(xx), pause
end
A2=zeros(mE1*nJ,mE1*nJ);
for kk=1:mE1
	A2((kk-1)*nJ+1:kk*nJ,(kk-1)*nJ+1:kk*nJ)=J;
end
nA2=size(A2,1);
A3=zeros(nA1,nA2);
A4=zeros(nA2,nA1);
A=[A1 A3; A4 A2];
nA= size(A,1);
B2 = zeros(nA2,nB1);
B=[B1;B2]; nB=nB1;
E2=zeros(mE1,nA2);
for kk=1:mE1
	E2(kk,(kk-1)*nJ+1)=1;
end
E=[E1 E2]; mE=mE1; kerE=ker(E);
if infor
	disp(' '), disp(' the matrices of the system with the internal model')
	A, disp(xx), pause
	B, disp(xx), pause
	E, disp(xx), pause
	disp(' '), disp('   the poles of the plant')
	p_plant=eig(A1);
	p_plant, disp(xx), pause
	disp(' ')
	disp('   the invariant zeros of the plant : must be different from the')
	disp('   eigenvalues of the exosystem to guarantee complementability')
	z_plant=gazero(A1,B1,C1);
	z_plant, disp(xx), pause
	disp(' '), disp('   the kernel of E')
	kerE, disp(xx), pause
end
Vm=mainco(A,B,kerE);
if infor
	disp(' ')
	disp('   the maximal (A,B)-controlled invariant contained in kerE')
	Vm, disp(xx), pause
end
Pl=[eye(nA1); zeros(nA2,nA1)];
if infor
	disp(' '), disp('   the extended plant')
	Pl, disp(xx), pause
	disp(' ')
	disp('   check if Vm complements the extended plant')
	disp(' ')
	disp('   Vm + Pl (must be the whole space)')
	Vm_plus_Pl=sums(Vm,Pl);
	Vm_plus_Pl, disp(xx), pause
	disp(' ')
	disp('   Vm ints Pl')
	Vm_ints_Pl=ints(Vm,Pl);
	Vm_ints_Pl, disp(xx), pause
end
tol=norm(A,'fro')*eps*10^4;
Rp=ints(Vm,Pl);
[t,nvi]=size(Rp);
no=norm(Rp,'fro');
if (nvi==1)&&(no<tol)
	nvi=0;
end
if nvi~=0
	V=compl(A,B,E,Pl);
	if infor
		disp(' ')
		disp('   since Vm is greater than a complement the plant, we construct')
		disp('   from it a complement of the plant')
	end
else
	V=Vm;
	if infor
		disp(' ')
		disp('   since Vm is a complement the plant, we can use it as a resolvent')
	end
end
if infor
	disp('   the resolvent')
	V, disp(xx), pause
	disp(' ')
	disp('   checks about V')
	disp(' ')
	disp('   V + Pl (must be the whole space)')
	V_plus_Pl=sums(V,Pl);
	V_plus_Pl, disp(xx), pause
	disp(' ')
	disp('   V ints Pl (must be the origin)')
	V_ints_Pl=ints(V,Pl);
	V_ints_Pl, disp(xx), pause
	disp(' ')
	disp('   E * V (must be zero)')
	E_V=E*V;
	E_V, disp(xx), pause
end
%disp(' ')
if infor
	disp(' ')
	disp('   pole assignment for the plant and derivation of an F such')
	disp('   that (A+B*F)*V is contained in V with internal eig = P1')
	disp(' ')
end
F=stabf(A,B,V,Pl,P1);
if infor
	disp(' '), disp('   matrix F')
	F, disp(xx), pause
	disp(' ')
	disp('   maximal A-invariant contained in kerE (must be the origin)')
	Jm_kerE=maxinv(A,kerE);
	Jm_kerE, disp(xx), pause
	disp(' ')
	disp('   pole assigment for the observer and derivation of a G such')
	disp('   that eig(A+GC) = P2')
	disp(' ')
end
G=-place(A',E',P2)'; % G %%%%
%G1 = (place1(A',E',P2))'; %%%% G1
if infor
	disp(' '), disp('   matrix G')
	G, disp(xx), pause
end
Ahat=[A B*F; -G*E A+B*F+G*E];
Ehat=[E zeros(mE,nA)];
A1plant=Ahat(1:nA1,1:nA1);
A3plant=Ahat(1:nA1,(nA+1):(nA+nA));
A4plant=Ahat((nA+1):(nA+nA),1:nA1);
A2plant=Ahat((nA+1):(nA+nA),(nA+1):(nA+nA));
Aplant=[A1plant A3plant; A4plant A2plant];
Bplant=[Ahat(1:nA1,(nA1+1):nA) ; Ahat((nA+1):(nA+nA),(nA1+1):nA)];
Cplant=[E1 zeros(mE,nA)];
Dplant=E2;
if infor
	disp(' ')
	disp('   eigenvalues of Aplant (must be the assigned ones)')
	eig_Aplant=eig(Aplant);
	eig_Aplant, disp(xx), pause
	disp(' ')
	disp('   computation of the invariant via the Sylvester equation')
	Xin=lyap(Aplant,-Ahat((nA1+1):nA,(nA1+1):nA),Bplant);
	X1=Xin(1:nA1,:);
	X2=Xin((nA1+1):(nA1+nA1),:);
	X3=Xin((nA1+nA1+1):(nA1+nA),:);
	Wsyl=[X1;eye(nA2);X2;X3];
	Wsyl, disp(xx), pause
	disp(' ')
	disp('   Ehat*Wsyl (must be zero)')
	Ehat_Wsylv=Ehat*Wsyl;
	Ehat_Wsylv, disp(xx), pause
	disp(' ')
	disp('   internal eigenvalues of Wsyl (must include those of the exosystem)')
	[P,Q]=stabi(Ahat,Wsyl);
	eig_P=eig(P);
	eig_P, disp(xx), pause
	disp(' ')
	disp('   external eigenvalues of Wsyl (must be the assigned ones)')
	eig_Q=eig(Q);
	eig_Q, disp(xx), pause
	disp(' ')
	disp('   maximal Ahat-invariant contained in kerEhat')
	kerEhat=ker(Ehat);
	W=maxinv(Ahat,kerEhat);
	W, disp(xx), pause
	disp(' ')
	disp('   internal eigenvalues of W (must include those of the exosystem)')
	[P,Q]=stabi(Ahat,W);
	eig_P=eig(P);
	eig_P, disp(xx), pause
	disp(' ')
	disp('   external eigenvalues of W (must be the assigned ones)')
	eig_Q=eig(Q);
	eig_Q, disp(xx), pause
	disp(' ')
	disp('   check of congruence')
	disp(' ')
	disp('   Wsyl + W (must have the same dimension as Wsyl and W)')
	Wsylv_plus_W=sums(Wsyl,W);
	Wsylv_plus_W, disp(xx), pause
	disp(' ')
	disp('   Wsyl ints W (must have the same dimension as Wsylv and W)')
	Wsylv_ints_W=ints(Wsyl,W);
	Wsylv_ints_W, disp(xx), pause
end
M=G; N=A+B*F+G*E; L=F; K=zeros(nB,mE);
% --- last line of regtr ---

function F = stabf(A,B,V,Pl,P)
%STABF    State feedback matrix in the regulator synthesis.
%  Let imV be an (A,B)-controlled invariant satisfying sums(imV,imPl) = X,
%  ints(imV,imPl) = 0, and suppose that (A1,B1) is stabilizable (A1 is formed
%  with the first n1 rows and columns of A, B1 with the first n1 rows of B,
%  where n1 is the number of columns of Pl).
%  F = stabf(A,B,V,Pl,P) returns a state feedback matrix such that (A+BF)imV
%  is contained in imV and A1+B1F1 has all its free eigenvalues assigned.

%  The routine implements Algorithm 6.2-2 of "Controlled and Conditioned
%  Invariants in Linear System Theory".

%       Basile and Marro 4-20-90

na=size(A,1);
mb=size(B,2);
% Checks and messages
mv=size(V,2);
my=size(mainco(A,B,V),2);
if my~=mv
	error('GEOAPPR:input', 'V is not a controlled invariant in STABF');
end
[t,my]=size(ima(B,1));
if my~=mb
	error('GEOAPPR:input', 'B is not full rank in STABF');
end
T=ima(B,1);
n1=size(T,2);
T=ima([T Pl],0);
n2=size(T,2);
T=ima([T V],0);
n3=size(T,2);
if n3~=na
	error('GEOAPPR:input', 'loss of rank in STABF');
end
T(:,1:n1)=B;
T(:,(n2+1):n3)=V;
At=T\A*T;
Bt=T\B;
F1t=-place(At(1:n2,1:n2),Bt(1:n2,:),P);
F2t=-At(1:n1,(n2+1):na);
Ft=[F1t F2t];
F=Ft/T;
% --- last line of stabf ---

function V = compl(A,B,E,Pl)
%COMPL    Complementation in the regulator synthesis.
%  Let imVm be the maximal (A,B)-controlled invariant contained in kerE such
%  that sums(imVm,imPl) = X.
%  V = compl(A,B,E,Pl) provides an orthonormal V such that
%  sums(imV,imPl) = X, ints(imV,imPl) = {0}, imV contained in kerE.
%  The routine implements Algorithm 6.2-1 of "Controlled and Conditioned
%  Invariants in Linear System Theory".

%       Basile and Marro 4-20-90 - revised 5-22-92

na=size(A,1);
mb=size(B,2);
tol = norm(A,'fro')*eps*10^4;
kerE=ortco(E');
Vm=mainco(A,B,kerE);
ne=size(sums(Vm,Pl),2);
if ne < na
	error('GEOAPPR:input', 'complementation impossible in COMPL');
end
T=ints(Vm,Pl);
ne=size(T,2);
no=norm(T,'fro');
if (ne==1)&&(no<tol)
	disp('   complementation not necessary in compl');
	V=Vm;
	return
end
mic=miinco(A,kerE,B);
Rv=ints(Vm,mic);
T=ima(Rv,0);
n1=size(T,2);
no=norm(T,'fro');
if (n1==1)&&(no<tol)
	n1=0;
end
Rp=ints(Vm,Pl);
if n1==0
	T=ima(Rp,0);
	Dmic=mic;
	Dmicv=Dmic;
	nz1=size(Dmic,2);
else
	Tx=T;
	T=ima([Tx Rp],0);
	Dmicv=ima([Tx mic],0);
	nz1=size(Dmicv,2);
	Dmic=Dmicv(:,(n1+1):nz1);
end
n2=size(T,2);
no=norm(T,'fro');
if (n2==1)&&(no<tol)
	n2=0;
end
if n2==0
	T=ima(Vm,0);
	Dpla=ima(Pl,0);
else
	Tx=T;
	T=ima([Tx Vm],0);
	Dpla=ima([Tx Dmic Pl],0);
	nz2=size(Dpla,2);
	Dpla=Dpla(:,(n2+nz1+1):nz2);
end
n3=size(T,2);
T=ima([T eye(na)],0);
[t,nz1]=size(Dmic);
T(:,(n3+1):(n3+nz1))=Dmic(:,1:nz1);
n4=n3+nz1;
[t,nz2]=size(Dpla);
T(:,(n4+1):(n4+nz2))=Dpla(:,1:nz2);
At=T\A*T;
Bt=T\B;
if n1~=0
	F1=rand(mb,n1);
	At(1:n1,1:n1) = At(1:n1,1:n1)+Bt(1:n1,:)*F1;
end
if n2==0
	V=ima(T(:,(n2+1):n3),1);
else
	A22=At(1:n2,1:n2);
	A33=At((n2+1):n3,(n2+1):n3);
	A23=At(1:n2,(n2+1):n3);
	X=lyap(A22,-A33,A23);
	V=ima(T(:,1:n2)*X+T(:,(n2+1):n3),1);
end
%[t,nv] = size(V);
%V = V*inv(V((na-nv+1):na,:));
% --- last line of compl ---

function F=place1(A,B,P)
%PLACE1   Eigenvalue assignment.
%  F = PLACE1(A,B,P) returns as F the state feedback matrix such that the
%  h assignable eigenvalues of A+B*F are those specified in the first h
%  rows of the column vector P.
%  F = PLACE1(A,B) assigns these eigenvalues in interactive mode.
%  To enter complex eigenvalues use symbol j for the imaginary unit.

%  Basile and Marro 4-20-90

j=sqrt(-1);
[ma,na]=size(A);
[mb,nb]=size(B);
T=mininv(A,B);
[m,n]=size(T);
nargs = nargin;
if n ~= 0
	if nargs == 2
	disp(   'Enter vector P of eigenvalues to be located: number of components')
		n
		ni=1;
		P=zeros(n,1);
		while (ni <= n)
			P(ni,1)=input(   'Enter an eigenvalue: ')
			ni=ni+1;
		end
	else
		if length(P) < n
			error('GEOAPPR:input', 'not enough eigenvalues transmitted to PLACE1');
		end
		P=P(1:n,:);
	end
	disp('   thinking')
	disp(' ')
	if (n < na)
		T=ima([T ortco(T)],0);
		A1=T\A*T;
		B1=T\B;
		A1=A1(1:n,1:n);
		B1=B1(1:n,:);
		F1=place(A1,B1,P);
		F1=[F1 zeros(nb,(na-n))];
		F=-F1/T;
	else
		F=place(A,B,P);
		F=-F;
	end
else
	disp('   no eigenvalues can be assigned in PLACE1')
end
% --- last line of place1 ---