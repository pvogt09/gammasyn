function M=fattmat(MM)
%FATTMAT  Positive semi-definite matrix factorization.
%   M=FATTMAT(MM) calculates as M a (rectangular) full rank matrix
%   such that MM=M'*M. MM must be positive semi-definite.

if any(real(eig(MM))<-norm(MM,'fro')*eps*10^(6))
	warning('GEOAPPR:input', ' **** non semi-definite positive matrix in FATTMAT')
end
[U,S,V]=svd(MM);
tol=eps*10^6*norm(MM,'fro');
ii=find(diag(S)>tol);
M=U*sqrt(S);
M=real(M(:,ii)');
if isempty(M), M=zeros(1,size(MM,1)); end
% --- last line of fattmat ---