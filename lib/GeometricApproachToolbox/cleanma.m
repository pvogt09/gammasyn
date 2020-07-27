function B = cleanma(A,tol)
%CLEANMA  Cleans a matrix.
%  B=cleanma(A) sets to zero the less significant elements of A. The
%  value of tolerance is 10^(-14)*norm(A,'fro').
%  B=cleanma(A,tol) makes it possible to set the tolerance.

%  Basile and Marro 4-20-90

if isempty(A), B = A; return, end
[na,ma] = size(A);
if nargin==1, tol = 10^(-14)*norm(A,'fro'); end
for k = 1:ma
	for h = 1:na
		vv = A(h,k);
%   if real(vv)>=0, vv=vv+tol; else, vv=vv-tol; end
		if abs(vv) < tol, vv=0; end
		B(h,k) = vv;
	end
end
% --- last line of cleanma ---