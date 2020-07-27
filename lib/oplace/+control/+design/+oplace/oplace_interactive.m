function [K] = oplace_interactive(sys, ew, options, K0, S, P, w3, W4)
	% K = OPLACE(sys,ew,K0,S,P,w3,W4)
	% Computes the constant feedback gain K of the output feedback u = -Ky = -KCx
	% by pole or eigenvalue assignment.
	%
	% Sie haben folgende Möglichkeiten:
	%
	% K = POLEASSIGN(sys,ew)
	%   sys			 	- LTI system
	%   ew(n,1)       - vector with n predefined closed loop eigenvalues
	%
	% K = POLEASSIGN(sys,ew,K0)
	%   sys			 	- LTI system
	%   ew(n,1)       - vector with n predefined closed loop eigenvalues
	%   K0(p,q)       - initial feedback matrix for optimization
	%   All elements of K are considered unconstrained and used for solving
	%   the underlying poleassignment problem.
	%
	% K = POLEASSIGN(sys,ew,K0,S)
	%   S(p,q)        - matrix of structural constraints
	%   Only the elements of K0 with S(i,j)=0 are considered unconstrained
	%   and used for solving the underlying poleassignment problem.
	%   The remaining elements of K0 are kept constant.
	%
	% K = POLEASSIGN(sys,ew,K0,S,P,w3)
	%   P(p,x)        - matrix with predefined parameter vectors (the i-th column
	%                   corresponds to the i-th eigenvalue in ew).
	%   w3(x,1)       - Gewichtungsfaktoren für die einzelnen Parameter-
	%                   vektoren
	%
	% K = POLEASSIGN(sys,ew,K0,S,P,w3,W4)
	%   W4(p,q)       - weighting matrix for the individual elements of K.
	%                   used to minimize the weighted norm of K.
	if nargin >= 3
		if isstruct(options)
			options.interactive = true;
		end
	end
	switch nargin
		case 0
			K = control.design.oplace.oplace();
		case 1
			K = control.design.oplace.oplace(sys);
		case 2
			K = control.design.oplace.oplace(sys, ew);
		case 3
			K = control.design.oplace.oplace(sys, ew, options);
		case 4
			K = control.design.oplace.oplace(sys, ew, options, K0);
		case 5
			K = control.design.oplace.oplace(sys, ew, options, K0, S);
		case 6
			K = control.design.oplace.oplace(sys, ew, options, K0, S, P);
		case 7
			K = control.design.oplace.oplace(sys, ew, options, K0, S, P, w3);
		case 8
			K = control.design.oplace.oplace(sys, ew, options, K0, S, P, w3, W4);
		otherwise
			error('oplace:arguments', 'Too many input arguments for function oplace.');
	end
end