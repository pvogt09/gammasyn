function [K, J_opt, eigenvalues_cl] = oplace(sys, ew, options, K0, S, P, w3, W4, w1)
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
	%
	%
	% K = POLEASSIGN(sys,ew,K0,S,P,w3,W4,w1)
	%   w1(n,1)       - weighting vector for the individual eigenvalues.

	%--- checking the input arguments
	if nargin < 2
		disp(' ');
		disp('------------------------------------------------');
		disp('Usage: K = OPLACE(sys,ew,[K0],[S],[P],[w3],[W4])');
		disp('------------------------------------------------');
		disp(' ');
		error('oplace:arguments', 'The function OPLACE needs at least 2 input arguments!');
	end
	switch nargin
		case 2
			K0 = []; S = []; P = []; w3 = []; W4 = []; w1 = [];
			options = [];
		case 3
			K0 = []; S = []; P = []; w3 = []; W4 = []; w1 = [];
		case 4
			S = []; P = []; w3 = []; W4 = []; w1 = [];
		case 5
			P = []; w3 = []; W4 = []; w1 = [];
		case 6
			W4 = [];
			w1 = [];
			[~, idx] = size(P);
			w3 = ones(idx, 1);
			fprintf('\nAll parameter vectors have a weight of 1!\n');
		case 7
			W4 = [];
			w1 = [];
		case 8
			w1 = [];
	end
	if isempty(options)
		options = struct(...
			'interactive',	false...
		);
	end
	if ~isstruct(options)
		error('oplace:arguments', 'Options must be of type ''struct''.');
	end
	if ~all(isfield(options, {'interactive'}))
		error('oplace:arguments', 'Option structure is missing fields.');
	end

	%Nichtnullelemente von S definiert auf 1 setzen
	if ~isempty(S)
		S = ~S;
		S = ~S;
	end

	%--- Dimensionen kontrollieren und in npq abspeichern
	[npq, ew, w3, K0] = dimensions(sys, ew, K0, S, P, w3, W4);
	% Nicht definierte Parametervektoren zufaellig erzeugen
	if npq(6) < npq(1)
		Pr = rand(npq(2), npq(1) - npq(6));
		P = [P, Pr];
	end
	%--- Vorgegebene Eigenwerte (und Parametervektoren) sortieren
	[ew, P, index] = sortewpv(ew, P, npq);
	if ~isempty(w1) && ndims(ew) == ndims(w1) && all(size(ew) == size(w1))
		w1 = w1(index, :);
	end
	A = sys.a;
	B = sys.b;
	C = sys.c;
	%--- Abfrage zu den EW und ggf. Abfragen zu erforderlichen Stützstellen
	[P, ew, st, ews, w1temp, w2, npq, index] = eigenvals(ew, A, P, npq, options.interactive);
	if isempty(w1)
		w1 = w1temp;
	else
		if ndims(w1) ~= ndims(index) || any(size(w1) < size(index))
			error('oplace:arguments', 'Eigenvalue weight must have dimension %dX%d.', size(index, 1), size(index, 2));
		end
		w1 = w1(index(1:size(w1temp, 1)), :);
	end
	%--- Berechnungskonstanten ermitteln
	if npq(4) > 0			% es könnten auch nur Stützstellen definiert werden
		[G_lam, H, ew, P, npq] = const_j1_j3(ew, A, B, C, P, npq);
	else
		[G_lam] = []; H = [];
		if npq(6) > 0
			fprintf('\nNone of the predefined %d parameter vectors can be used!\n', npq(6));
			npq(6) = 0;
		end
	end
	if npq(5) > 0 		% wenn Stützstellen definiert sind
		[G_xi, p_xi] = const_j2(st, ew, ews, A, B, C, npq);
	else
		G_xi = []; p_xi = [];
	end
	if ~isempty(W4)		% wenn die einzelnen Elemente gewichtet werden
		W4 = abs(W4);
		if ~isempty(S)
			%W4 = const_j4(W4,S,npq);
			W4 = W4.*(~S);
		end
	end

	% Entscheidung, wie die Matrix bestimmt werden kann:
	% a) komplett analytisch,
	% b) komplett numerisch oder
	% c) eine Kombination aus a) und b)
	%warning off;
	art = 'a';
	border = '-----------------------------------------------------';
	[anaok, ew, P, H, npq] = testana(isempty(S), isempty(W4), ew, P, H, npq);
	if anaok		% Fall a)
		K = analytic(A, B, C, ew, P, H, G_lam, G_xi, p_xi, w1, w2, anaok, npq);
		disp(K);
		eigenvalues = eig(A - B*K*C);
		eigenvalues = sort(eigenvalues);
		fprintf('\nClosed-loop eigenvalues with the above K:\n');
		disp(border);
		for ii = 1:npq(1)
			fprintf('%d.\t%s\n', ii, num2str(eigenvalues(ii)));
		end
		disp(border);
		if nargout >= 2
			c = zeros(4, 1);
			if npq(4)
				c(1) = criterion_1(K, w1, G_lam, npq, ew);
			end
			if npq(5)
				c(2) = criterion_2(K, w2, G_xi, p_xi, npq);
			end
			if npq(6)
				c(3) = criterion_3(K, w3, P, H, npq);
			end
			if ~isempty(W4)
				c(4) = criterion_4(K, W4, []);
			end
		end
	else
		if npq(2)*npq(3) - npq(2)*npq(6) >= npq(1) - npq(6) && isempty(S) && npq(6)
			if options.interactive
				while art ~= 'y' && art ~= 'n'
					question = sprintf('\nShall the predefined %d parametervectors be exactly approximated (y/n)?', npq(6));
					art = inp_from_keyb(question, 2);
				end
			else
				art = 'n';
			end
		else
			art = 'n';
		end
	end
	switch art
		case 'n'		% Fall b)
			K = K0;
			inp = 'y';
			%options = set_options;
			while strcmpi(inp, 'y')
				[c, optflag] = startval(K, npq, ew, w1, G_lam, w2, G_xi, p_xi, w3, P, H, W4, [], options.interactive);
				J = objective_function(S, npq, ew, w1, G_lam, w2, G_xi, p_xi, w3, P, H, W4, [], c, optflag);
				K = control.design.oplace.optimization.minsearch(J, K, options);
				disp(K);
				eigenvalues = eig(A - B*K*C);
				eigenvalues = sort(eigenvalues);
				fprintf('\nClosed-loop eigenvalues with the above K:\n');
				disp(border);
				for ii = 1:npq(1)
					fprintf('%d.\t%s\n', ii, num2str(eigenvalues(ii)));
				end
				disp(border);
				c = zeros(4, 1);
				if npq(4)
					c(1) = criterion_1(K, w1, G_lam, npq, ew);
				end
				if npq(5)
					c(2) = criterion_2(K, w2, G_xi, p_xi, npq);
				end
				if npq(6)
					c(3) = criterion_3(K, w3, P, H, npq);
				end
				if ~isempty(W4)
					c(4) = criterion_4(K, W4, []);
				end
				fprintf('\nValues of the individual criterions with the above K:\n');
				disp(border);
				fprintf('J(1) :  %d\n', c(1));
				fprintf('J(2) :  %d\n', c(2));
				fprintf('J(3) :  %d\n', c(3));
				fprintf('J(4) :  %d\n', c(4));
				disp(border);
				if options.interactive
					inp = 'a';
					while (inp ~= 'y' && inp ~= 'n') || isempty(inp)
						inp = inp_from_keyb('\nRestart optimization (y/n)?', 2);
					end
				else
					inp = 'n';
				end
			end
		case 'y'		% Fall c)
			[Ko, T1] = para_exact(P, H, npq);
			[Ks, npq_s, w1_s, ew_s, G_lam_s, G_xi_s, rho, w2_s] = transsys(A, B, C, K0, Ko, T1, npq, ew);
			inp = 'y';
			%options = set_options;
			while strcmpi(inp, 'y')
				[c, optflag] = startval(Ks, npq_s, ew_s, w1_s, G_lam_s, w2_s, G_xi_s, rho, [], [], [], W4, T1);
				J = objective_function([], npq_s, ew_s, w1_s, G_lam_s, w2_s, G_xi_s, rho, [], [], [], W4, T1, c, optflag);
				Ks = control.design.oplace.optimization.minsearch(J, Ks, options);
				K = -Ko + Ks*T1';
				eigenvalues = eig(A - B*K*C);
				eigenvalues = sort(eigenvalues);
				fprintf('\nClosed-loop eigenvalues with the above K:\n');
				disp(border);
				for ii = 1:npq(1)
					fprintf('%d.\t%s\n', ii, num2str(eigenvalues(ii)));
				end
				disp(border);
					c = zeros(4, 1);
				if npq(4)
					c(1) = criterion_1(Ks, w1_s, G_lam_s, npq_s, ew_s);
				end
				if npq(5)
					c(2) = criterion_2(Ks, w2_s, G_xi_s, rho, npq_s);
				end
				if ~isempty(W4)
					c(4) = criterion_4(Ks, W4, T1);
				end
				fprintf('\nValues of the individual criterions with the above K:\n');
				disp(border);
				fprintf('J(1) :  %d\n', c(1));
				fprintf('J(2) :  %d\n', c(2));
				fprintf('J(3) :  %d\n', c(3));
				fprintf('J(4) :  %d\n', c(4));
				disp(border);
				if options.interactive
					inp = 'a';
					while (inp ~= 'y' && inp ~= 'n') || isempty(inp)
						inp = inp_from_keyb('\nRestart optimization (y/n)?', 2);
					end
				else
					inp = 'n';
				end
			end
	end
	if nargout >= 2
		J_opt = sum(c);
	end
	if nargout >= 3
		eigenvalues_cl = eigenvalues;
	end
end

function [J] = objective_function(S, npq, ew, w1, G_lam, w2, G_xi, p_xi, w3, P, H, W4, T1, c, optflag)
	function [f, G, Rcode] = objective(K)
		[f, G, Rcode] = critgrad(K, S, npq, ew, w1, G_lam, w2, G_xi, p_xi, w3, P, H, W4, T1, c, optflag);
	end
	J = @objective;
end