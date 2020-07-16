function F = effe(A,B,X,C,D)
%EFFE     State feedback matrix for a controlled invariant.
%  F = effe(A,B,X) is a state-to-input feedback matrix such that the
%  (A,imB)-controlled invariant imX is an (A+B*F)-invariant.
%  F = effe(A,B,X,C,D) is a state-to-input feedback matrix such that
%  the (A,imB)-controlled invariant imX is an (A+B*F)-invariant
%  contained in ker(C+D*F).

%   Marro 4-20-01

% Checks and messages
[mx,nx] = size(X);
if ~any(X), nx=0; end
VV=mainco(A,B,X);
[my,ny] = size(VV);
if ~any(VV), ny=0; end
if ny ~= nx
	warning('GEOAPPR:input', 'X is not a controlled invariant in EFFE')
end
[mb,nb] = size(B);
V=X; nv=nx;
if nargin==3
	[my,ny] = size(ima(B,1));
	if ny ~= nb
		warning('GEOAPPR:input', 'B is not full rank in EFFE')
	end
	XU=pinv([V B])*A*V;
	U=XU(nv+1:nv+nb,:);
	%F=-U*inv(V'*V)*V';
	F=-U*pinv(V);
elseif nargin==5
	[my,ny] = size(ima([B;D],1));
	if ny ~= nb
		warning('GEOAPPR:input', '[B;D] is not full rank in EFFE')
	end
	mc=size(C,1);
	XU=pinv([V B; zeros(mc,nv) D])*[A; C]*V;
	U=XU(nv+1:nv+nb,:);
	F=-U*pinv(V);
end
% --- last line of effe ----