function sysinfo(A,B,C,D,fl)
%SYSINFO  Detects and displays the main properties of an LTI system.
% Possible calls: sysinfo(sys) or sysinfo(A,B,C,D[,1]).
% The five-arguments call refers to a discrete-time system.


if (nargin==4)||(nargin==5)
	if (nargin==4)||((nargin==5)&&(fl==0)), disc=0; else, disc=1; end
elseif nargin==1
	[A,B,C,D,disc]=ssdata(A); if disc~=0, disc=1; end
else
	error('GEOAPPR:input', '*** illegal number of input arguments in syspro');
end

disp(' ')
if disc
	fprintf('   The system is discrete-time\n')
else
	fprintf('   The system is continuous-time\n')
end

na=length(A); nb=size(B,2); mc=size(C,1);
if na==0, return, end

if nb==mc
	fprintf('   The system is square: nb=mc=%.4g\n',nb)
else
	fprintf('   The system is not square: nb=%.4g, mc=%.4g\n',nb,mc)
end

RR=mininv(A,B); if ~any(RR), RR=[]; end
if (size(RR,2)~=na)||isempty(RR)
	fprintf('   The system is non-completely controllable\n')
else
	fprintf('   The system is completely controllable\n')
end
OO=maxinv(A,ker(C)); if ~any(OO), OO=[]; end
if isempty(OO)
	fprintf('   The system is completely observable\n')
else
	fprintf('   The system is non-completely observable\n')
end

p=eig(A);
z=gazero(A,B,C,D); nz=length(z);

if isempty(z)
	fprintf('   The system is minimum-phase\n')
elseif ~disc
	if any(real(p)>=zeros(na,1))
		fprintf('   The system is not stable\n')
	else
		if size(ima(-C*inv(A)*B+D),2)<mc
			fprintf('   The system is stable but not steady-state controllable\n')
		else
			fprintf('   The system is stable and steady-state controllable\n')
		end
	end
	if any(real(z)>=zeros(nz,1))
		fprintf('   The system is not minimum-phase\n')
	else
		fprintf('   The system is minimum-phase\n')
	end
elseif disc
	if any(abs(p)>=ones(na,1))
		fprintf('   The system is not stable\n')
	else
		if size(ima(C/(eye(na)-A)*B+D),2)<mc
			fprintf('   The system is stable but not steady-state controllable\n')
		else
			fprintf('   The system is stable and steady-state controllable\n')
		end
	end
	if any(abs(z)>=ones(nz,1))
		fprintf('   The system is not minimum-phase\n')
	else
		fprintf('   The system is minimum-phase\n')
	end
end

V=vstar(A,B,C,D); S=sstar(A,B,C,D);
RV=ints(V,S); RC=sums(V,S);

if any(RV)
	fprintf('   The system is not left-invertible\n')
else
	fprintf('   The system is left-invertible\n')
end
if size(RC,2)<na
	fprintf('   The system is not right-invertible\n')
else
	fprintf('   The system is right-invertible\n')
end

rho=reldeg(A,B,C,D);
if isempty(rho)
	fprintf('   *** The relative degree is not computable\n')
else
	fprintf('   The system has relative degree rho=%.4g\n',rho)
end

fprintf('   The system has the following poles:\n')
tol=10^(-8);
for kk=1:na
	rr=real(p(kk)); ii=imag(p(kk));
	fprintf('                ')
	if abs(rr)<=tol && abs(ii)<=tol, fprintf(' 0')
	elseif rr>tol, fprintf(' %.4g',rr), else, fprintf('%.4g',rr), end
	if abs(ii)>tol
	if ii >= 0, si=' + '; else, si=' - '; end
	if (abs(rr)<=tol)&&(strcmp(si, ' + ')), si=[]; end
	fprintf([si,'%.4gi'],abs(ii))
	end
	fprintf('\n')
end

if isempty(z)
	fprintf('   The system has no invariant zeros\n')
else
	fprintf('   The system has the following invariant zeros:\n')
	for kk=1:nz
		rr=real(z(kk)); ii=imag(z(kk));
		fprintf('                ')
		if abs(rr)<=tol && abs(ii)<=tol, fprintf(' 0')
		elseif rr>tol, fprintf(' %.4g',rr), else, fprintf('%.4g',rr), end
		if abs(ii)>tol
		if ii >= 0, si=' + '; else, si=' - '; end
		if (abs(rr)<=tol) && strcmp(si, ' + '), si=[]; end
		fprintf([si,'%.4gi'],abs(ii))
		end
		fprintf('\n')
	end
end

% --- end of sysinfo ---