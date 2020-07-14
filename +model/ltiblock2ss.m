function [ltiss] = ltiblock2ss(lti)
	%LTIBLOCK2SS convert ltiblock.tf or ltiblock.pid to ltiblock.ss
	%	Input:
	%		lti:	system to convert
	%	Output:
	%		ltiss:	system as ltiblock.ss
	if isnumeric(lti)
		if ndims(lti) ~= 2 %#ok<ISMAT> compatibility with Octave
			error('model:ltiblock:input', 'Gain matrix must have 2 dimensions.');
		end
		if any(imag(lti) ~= 0)
			error('model:ltiblock:input', 'Gain matrix must not be complex.');
		end
		if any(isnan(lti) | isinf(lti))
			error('model:ltiblock:input', 'Gain matrix must not be NaN or infinite.');
		end
		ltiss = ltiblock.ss('', 0, size(lti, 1), size(lti, 2), 0);
		ltiss.d.Value = lti;
		ltiss.d.Minimum = -Inf(size(lti));
		ltiss.d.Maximum = Inf(size(lti));
		ltiss.d.Free = false(size(lti));
		ltiss.d.Scale = ones(size(lti));
		return;
	end
	% state space
	if isa(lti, 'ltiblock.ss')
		ltiss = lti;
		return;
	end
	if isa(lti, 'tunableSS')
		A = lti.A;
		B = lti.B;
		C = lti.C;
		D = lti.D;
		ltiss = ltiblock.ss(lti.Name, size(A.Value, 1), size(C.Value, 1), size(B.Value, 2), lti.Ts);
		ltiss.a.Value = A.Value;
		ltiss.a.Minimum = A.Minimum;
		ltiss.a.Maximum = A.Maximum;
		ltiss.a.Free = A.Free;
		ltiss.a.Scale = A.Scale;
		ltiss.b.Value = B.Value;
		ltiss.b.Minimum = B.Minimum;
		ltiss.b.Maximum = B.Maximum;
		ltiss.b.Free = B.Free;
		ltiss.b.Scale = B.Scale;
		ltiss.c.Value = C.Value;
		ltiss.c.Minimum = C.Minimum;
		ltiss.c.Maximum = C.Maximum;
		ltiss.c.Free = C.Free;
		ltiss.c.Scale = C.Scale;
		ltiss.d.Value = D.Value;
		ltiss.d.Minimum = D.Minimum;
		ltiss.d.Maximum = D.Maximum;
		ltiss.d.Free = D.Free;
		ltiss.d.Scale = D.Scale;
		return;
	end
	% static gain
	if isa(lti, 'ltiblock.gain') || isa(lti, 'tunableGain')
		ltiss = ltiblock.ss(lti.Name, 0, size(lti, 1), size(lti, 2), lti.Ts);
		ltiss.d.Value = lti.Gain.Value;
		ltiss.d.Minimum = lti.Gain.Minimum;
		ltiss.d.Maximum = lti.Gain.Maximum;
		ltiss.d.Free = lti.Gain.Free;
		ltiss.d.Scale = lti.Gain.Scale;
		return;
	end
	% transfer function
	isltitf = isa(lti, 'ltiblock.tf');
	istf = isa(lti, 'tunableTF');
	if isltitf || istf
		if isltitf
			num = lti.num;
			den = lti.den;
		else
			num = lti.Numerator;
			den = lti.Denominator;
		end
		if iscell(lti.den)
			system_order = max(cellfun(@length, den, 'UniformOutput', true)) - 1;
		else
			system_order = max(length(den.Value)) - 1;
		end
		ltiss = ltiblock.ss(lti.Name, system_order, size(lti, 1), size(lti, 2), lti.Ts);
		ltiss.a.Value(1:end - 1, 1:end) = [
			zeros(system_order - 1, 1),	eye(system_order - 1, system_order - 1)
		];
		ltiss.a.Free(1:end - 1, 1:end) = false;
		ltiss.a.Maximum(1:end - 1, 1:end) = [
			zeros(system_order - 1, 1),	eye(system_order - 1, system_order - 1)
		];
		ltiss.a.Minimum(1:end - 1, 1:end) = [
			zeros(system_order - 1, 1),	eye(system_order - 1, system_order - 1)
		];
		ltiss.a.Scale(1:end - 1, 1:end) = 1;
		ltiss.a.Value(end, :) = -fliplr(den.Value(2:end));
		ltiss.a.Maximum(end, :) = -fliplr(den.Minimum(2:end));
		ltiss.a.Minimum(end, :) = -fliplr(den.Maximum(2:end));
		ltiss.a.Free(end, :) = den.Free(2:end);
		ltiss.a.Scale(end, :) = fliplr(den.Scale(2:end));

		ltiss.b.Value = double(1:system_order == system_order)';
		ltiss.b.Maximum = double(1:system_order == system_order)';
		ltiss.b.Minimum = double(1:system_order == system_order)';
		ltiss.b.Free = false;

		if size(lti.num.Value, 2) < system_order + 1
			% no throughput (see tf2ss)
			ltiss.d.Value = 0;
			ltiss.d.Maximum = 0;
			ltiss.d.Minimum = 0;
			ltiss.d.Free = false;
			ltiss.d.Scale = 1;

			ltiss.c.Value = [
				fliplr(num.Value), zeros(1, system_order - size(num.Value, 2))
			];
			ltiss.c.Minimum = [
				fliplr(num.Minimum), zeros(1, system_order - size(num.Value, 2))
			];
			ltiss.c.Maximum = [
				fliplr(num.Maximum), zeros(1, system_order - size(num.Value, 2))
			];
			ltiss.c.Free = [
				fliplr(num.Free), false(1, system_order - size(num.Value, 2))
			];
			ltiss.c.Scale = [
				fliplr(num.Value), ones(1, system_order - size(num.Value, 2))
			];
		else
			% throughput (see tf2ss)
			if num.Free(1) && any(num.Free(2:end))
				error('model:ltiblock:input', 'Cannot convert tf to ss: Tunable parameters must not be multiplied.');
			end
			ltiss.d.Value = num.Value(1);
			ltiss.d.Maximum = num.Maximum(1);
			ltiss.d.Minimum = num.Minimum(1);
			ltiss.d.Free = num.Free(1);
			ltiss.d.Scale = num.Scale(1);

			ltiss.c.Value = [
				fliplr(num.Value(2:end)), zeros(1, system_order - size(num.Value, 2))
			] - num.Value(1, 1)*den.Value(1, 2:end);
			ltiss.c.Minimum = [
				fliplr(num.Minimum(2:end)), zeros(1, system_order - size(num.Value, 2))
			] - num.Maximum(1, 1)*den.Maximum(1, 2:end);
			ltiss.c.Minimum(isnan(ltiss.c.Minimum)) = -Inf;
			ltiss.c.Maximum = [
				fliplr(num.Maximum(2:end)), zeros(1, system_order - size(num.Value, 2))
			] - num.Minimum(1, 1)*den.Minimum(1, 2:end);
			ltiss.c.Maximum(isnan(ltiss.c.Maximum)) = Inf;
			ltiss.c.Free = [
				fliplr(num.Free(2:end)), false(1, system_order - size(num.Value, 2))
			] | (num.Free(1, 1) | den.Free(1, 2:end));
			ltiss.c.Scale = [
				fliplr(num.Value(2:end)), ones(1, system_order - size(num.Value, 2))
			];
		end
		return;
	end
	% pid block
	if isa(lti, 'ltiblock.pid') || isa(lti, 'tunablePID')
		T = lti.Ts;
		K_P = lti.Kp;
		K_I = lti.Ki;
		K_D = lti.Kd;
		T_F = lti.Tf;
		IFormula = lti.IFormula;
		DFormula = lti.DFormula;
		if T > 0
			%ltiss = ltiblock.ss(lti.Name, 2, 1, 1, T);
			A_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			B_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			C_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			D_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			A_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			B_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			C_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			D_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			if K_I.Free || (~K_I.Free && K_I.Value ~= 0)
				if strcmpi(IFormula, 'ForwardEuler')
					A_I.Value = 1;
					A_I.Free = false;
					A_I.Minimum = 1;
					A_I.Maximum = 1;
					A_I.Scale = 1;

					B_I.Value = 1;
					B_I.Free = false;
					B_I.Minimum = 1;
					B_I.Maximum = 1;
					B_I.Scale = 1;

					C_I.Value = K_I.Value*T;
					C_I.Free = K_I.Free;
					C_I.Minimum = K_I.Minimum*T;
					C_I.Maximum = K_I.Maximum*T;
					C_I.Scale = K_I.Scale*T;

					D_I.Value = 0;
					D_I.Free = false;
					D_I.Minimum = 0;
					D_I.Maximum = 0;
					D_I.Scale = 1;
				elseif strcmpi(IFormula, 'BackwardEuler')
					A_I.Value = 1;
					A_I.Free = false;
					A_I.Minimum = 1;
					A_I.Maximum = 1;
					A_I.Scale = 1;

					B_I.Value = 1;
					B_I.Free = false;
					B_I.Minimum = 1;
					B_I.Maximum = 1;
					B_I.Scale = 1;

					C_I.Value = K_I.Value*T;
					C_I.Free = K_I.Free;
					C_I.Minimum = K_I.Minimum*T;
					C_I.Maximum = K_I.Maximum*T;
					C_I.Scale = K_I.Scale*T;

					D_I.Value = K_I.Value*T;
					D_I.Free = K_I.Free;
					D_I.Minimum = K_I.Minimum*T;
					D_I.Maximum = K_I.Maximum*T;
					D_I.Scale = K_I.Scale*T;
				elseif any(strcmpi(IFormula, {'Trapezoid', 'Trapezoidal'}))
					A_I.Value = 1;
					A_I.Free = false;
					A_I.Minimum = 1;
					A_I.Maximum = 1;
					A_I.Scale = 1;

					B_I.Value = 2;
					B_I.Free = false;
					B_I.Minimum = 2;
					B_I.Maximum = 2;
					B_I.Scale = 1;

					C_I.Value = K_I.Value*T/2;
					C_I.Free = K_I.Free;
					C_I.Minimum = K_I.Minimum*T/2;
					C_I.Maximum = K_I.Maximum*T/2;
					C_I.Scale = K_I.Scale*T/2;

					D_I.Value = K_I.Value*T/2;
					D_I.Free = K_I.Free;
					D_I.Minimum = K_I.Minimum*T/2;
					D_I.Maximum = K_I.Maximum*T/2;
					D_I.Scale = K_I.Scale*T/2;
				else
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for integral part of pid controller.', IFormula);
				end
			end
			if K_D.Free || (~K_D.Free && K_D.Value ~= 0)
				if strcmpi(DFormula, 'ForwardEuler')
					A_D.Value = 1 - T/T_F.Value;
					A_D.Free = T_F.Free;
					A_D.Minimum = 1 - T/T_F.Minimum;
					A_D.Maximum = 1 - T/T_F.Maximum;
					A_D.Scale = T/T_F.Scale;

					B_D.Value = T/T_F.Value;
					B_D.Free = T_F.Free;
					B_D.Minimum = T/T_F.Maximum;
					B_D.Maximum = T/T_F.Minimum;
					B_D.Scale = T/T_F.Scale;

					C_D.Value = K_D.Value/T_F.Value;
					C_D.Free = K_D.Free | T_F.Free;
					C_D.Minimum = K_D.Minimum/T_F.Maximum;
					C_D.Maximum = K_D.Maximum/T_F.Minimum;
					C_D.Scale = K_D.Scale/T_F.Scale;

					D_D.Value = K_D.Value/T_F.Value;
					D_D.Free = K_D.Free | T_F.Free;
					D_D.Minimum = K_D.Minimum/T_F.Maximum;
					D_D.Maximum = K_D.Maximum/T_F.Minimum;
					D_D.Scale = K_D.Scale/T_F.Scale;
				elseif strcmpi(DFormula, 'BackwardEuler')
					A_D.Value = T_F.Value/(T_F.Value + T);
					A_D.Free = T_F.Free;
					A_D.Minimum = 0;
					A_D.Maximum = Inf;
					A_D.Scale = 1;

					B_D.Value = T_F.Value/(T_F.Value + T) - 1;
					B_D.Free = T_F.Free;
					B_D.Minimum = -1;
					B_D.Maximum = Inf;
					B_D.Scale = 1;

					C_D.Value = K_D.Value/(T_F.Value + T);
					C_D.Free = K_D.Free | T_F.Free;
					C_D.Minimum = -Inf;
					C_D.Maximum = Inf;
					C_D.Scale = K_D.Scale/T_F.Scale;

					D_D.Value = K_D.Value/(T_F.Value + T);
					D_D.Free = K_D.Free | T_F.Free;
					D_D.Minimum = -Inf;
					D_D.Maximum = Inf;
					D_D.Scale = K_D.Scale/T_F.Scale;
				elseif any(strcmpi(DFormula, {'Trapezoid', 'Trapezoidal'}))
					A_D.Value = -(T/2 - T_F.Value)/(T_F.Value + T/2);
					A_D.Free = T_F.Free;
					A_D.Minimum = -Inf;
					A_D.Maximum = Inf;
					A_D.Scale = 1;

					B_D.Value = 1 + (T/2 - T_F.Value)/(T_F.Value + T/2);
					B_D.Free = T_F.Free;
					B_D.Minimum = -Inf;
					B_D.Maximum = Inf;
					B_D.Scale = 1;

					C_D.Value = -K_D.Value/(T_F.Value + T/2);
					C_D.Free = K_D.Free | T_F.Free;
					C_D.Minimum = -Inf;
					C_D.Maximum = Inf;
					C_D.Scale = K_D.Scale/T_F.Scale;

					D_D.Value = K_D.Value/(T_F.Value + T/2);
					D_D.Free = K_D.Free | T_F.Free;
					D_D.Minimum = -Inf;
					D_D.Maximum = Inf;
					D_D.Scale = K_D.Scale/T_F.Scale;
				else
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for derivative part of pid controller.', IFormula);
				end
			end
			A = struct(...
				'Value',	blkdiag(A_I.Value, A_D.Value),...
				'Minimum',	blkdiag(A_I.Minimum, A_D.Minimum),...
				'Maximum',	blkdiag(A_I.Maximum, A_D.Maximum),...
				'Free',		blkdiag(A_I.Free, A_D.Free),...
				'Scale',	blkdiag(A_I.Scale, A_D.Scale)...
			);
			B = struct(...
				'Value',	[
					B_I.Value;
					B_D.Value
				],...
				'Minimum',	[
					B_I.Minimum;
					B_D.Minimum
				],...
				'Maximum',	[
					B_I.Maximum;
					B_D.Maximum
				],...
				'Free',		[
					B_I.Free;
					B_D.Free
				],...
				'Scale',	[
					B_I.Scale;
					B_D.Scale
				]...
			);
			C = struct(...
				'Value',	[
					C_I.Value,		C_D.Value
				],...
				'Minimum',	[
					C_I.Minimum,	C_D.Minimum
				],...
				'Maximum',	[
					C_I.Maximum,	C_D.Maximum
				],...
				'Free',		[
					C_I.Free,		C_D.Free
				],...
				'Scale',	[
					C_I.Scale,		C_D.Scale
				]...
			);
			D = struct(...
				'Value',	K_P.Value,...
				'Minimum',	K_P.Minimum,...
				'Maximum',	K_P.Maximum,...
				'Free',		K_P.Free,...
				'Scale',	K_P.Scale...
			);
			if ~isempty(D_I.Value)
				D.Value = D.Value + D_I.Value;
				D.Minimum = D.Minimum + D_I.Minimum;
				D.Maximum = D.Maximum + D_I.Maximum;
				D.Free = D.Free | D_I.Free;
				D.Scale = D.Scale + D_I.Scale;
			end
			if ~isempty(D_D.Value)
				D.Value = D.Value + D_D.Value;
				D.Minimum = D.Minimum + D_D.Minimum;
				D.Maximum = D.Maximum + D_D.Maximum;
				D.Free = D.Free | D_D.Free;
				D.Scale = D.Scale + D_D.Scale;
			end
			ltiss = ltiblock.ss(lti.Name, size(A.Value, 1), size(B.Value, 2), size(C.Value, 1), lti.Ts);
			ltiss.a.Value = A.Value;
			ltiss.a.Minimum = A.Minimum;
			ltiss.a.Maximum = A.Maximum;
			ltiss.a.Free = A.Free;
			ltiss.a.Scale = A.Scale;
			ltiss.b.Value = B.Value;
			ltiss.b.Minimum = B.Minimum;
			ltiss.b.Maximum = B.Maximum;
			ltiss.b.Free = B.Free;
			ltiss.b.Scale = B.Scale;
			ltiss.c.Value = C.Value;
			ltiss.c.Minimum = C.Minimum;
			ltiss.c.Maximum = C.Maximum;
			ltiss.c.Free = C.Free;
			ltiss.c.Scale = C.Scale;
			ltiss.d.Value = D.Value;
			ltiss.d.Minimum = D.Minimum;
			ltiss.d.Maximum = D.Maximum;
			ltiss.d.Free = D.Free;
			ltiss.d.Scale = D.Scale;
		else
			hasI = K_I.Free || (~K_I.Free && K_I.Value ~= 0);
			hasD = K_D.Free || (~K_D.Free && K_D.Value ~= 0);
			ltiss = ltiblock.ss(lti.Name, hasI + hasD, 1, 1);
			if hasI && hasD
				ltiss.a.Value = [
					0,	0;
					0,	-1/T_F.Value
				];
				ltiss.a.Free = [
					false,	false;
					false,	T_F.Free
				];
				ltiss.a.Minimum = [
					0,	0;
					0,	-1/T_F.Minimum
				];
				ltiss.a.Maximum = [
					0,	0;
					0,	-1/T_F.Maximum
				];
				ltiss.a.Scale = [
					1,	1;
					1,	1/T_F.Scale
				];

				ltiss.b.Value = [
					K_I.Value;
					1/T_F.Value
				];
				ltiss.b.Free = [
					K_I.Free;
					T_F.Free
				];
				ltiss.b.Minimum = [
					K_I.Minimum;
					1/T_F.Maximum
				];
				ltiss.b.Maximum = [
					K_I.Maximum;
					1/T_F.Minimum
				];
				ltiss.b.Scale = [
					K_I.Scale;
					1/T_F.Scale
				];

				ltiss.c.Value = [
					1,	-K_D.Value/T_F.Value
				];
				ltiss.c.Free = [
					false,	K_D.Free | T_F.Free
				];
				ltiss.c.Minimum = [
					1,	K_D.Minimum/T_F.Maximum
				];
				ltiss.c.Maximum = [
					1,	K_D.Maximum/T_F.Minimum
				];
				ltiss.c.Scale = [
					1,	K_D.Scale/T_F.Scale
				];
			elseif hasI
				ltiss.a.Value = 0;
				ltiss.a.Free = false;
				ltiss.a.Minimum = 0;
				ltiss.a.Maximum = 0;
				ltiss.a.Scale = 1;

				ltiss.b.Value = K_I.Value;
				ltiss.b.Free = K_I.Free;
				ltiss.b.Minimum = K_I.Minimum;
				ltiss.b.Maximum = K_I.Maximum;
				ltiss.b.Scale = K_I.Scale;

				ltiss.c.Value = 1;
				ltiss.c.Free = false;
				ltiss.c.Minimum = 1;
				ltiss.c.Maximum = 1;
				ltiss.c.Scale = 1;
			elseif hasD
				ltiss.a.Value = -1/T_F.Value;
				ltiss.a.Free = T_F.Free;
				ltiss.a.Minimum = -1/T_F.Minimum;
				ltiss.a.Maximum = -1/T_F.Maximum;
				ltiss.a.Scale = 1/T_F.Scale;

				ltiss.b.Value = 1/T_F.Value;
				ltiss.b.Free = T_F.Free;
				ltiss.b.Minimum = 1/T_F.Maximum;
				ltiss.b.Maximum = 1/T_F.Minimum;
				ltiss.b.Scale = 1/T_F.Scale;

				ltiss.c.Value = -K_D.Value/T_F.Value;
				ltiss.c.Free = K_D.Free | T_F.Free;
				ltiss.c.Minimum = K_D.Minimum/T_F.Maximum;
				ltiss.c.Maximum = K_D.Maximum/T_F.Minimum;
				ltiss.c.Scale = K_D.Scale/T_F.Scale;
			else
				ltiss.a.Value = [];
				ltiss.a.Free = [];
				ltiss.a.Minimum = [];
				ltiss.a.Maximum = [];
				ltiss.a.Scale = [];

				ltiss.b.Value = zeros(0, 1);
				ltiss.b.Free = false(0, 1);
				ltiss.b.Minimum = -Inf(0, 1);
				ltiss.b.Maximum = Inf(0, 1);
				ltiss.b.Scale = ones(0, 1);

				ltiss.c.Value = zeros(1, 0);
				ltiss.c.Free = false(1, 0);
				ltiss.c.Minimum = -Inf(1, 0);
				ltiss.c.Maximum = Inf(1, 0);
				ltiss.c.Scale = ones(1, 0);
			end

			if hasD
				ltiss.d.Value = K_P.Value + K_D.Value/T_F.Value;
				ltiss.d.Free = K_P.Free | K_D.Free | T_F.Free;
				ltiss.d.Minimum = K_P.Minimum + K_D.Minimum/T_F.Maximum;
				ltiss.d.Maximum = K_P.Maximum + K_D.Maximum/T_F.Minimum;
				ltiss.d.Scale = K_P.Scale + K_D.Scale/T_F.Scale;
			else
				ltiss.d.Value = K_P.Value;
				ltiss.d.Free = K_P.Free;
				ltiss.d.Minimum = K_P.Minimum;
				ltiss.d.Maximum = K_P.Maximum;
				ltiss.d.Scale = K_P.Scale;
			end
		end
		return;
	end
	% pid2 block
	if isa(lti, 'ltiblock.pid2') || isa(lti, 'tunablePID2')
		T = lti.Ts;
		K_P = lti.Kp;
		K_I = lti.Ki;
		K_D = lti.Kd;
		T_F = lti.Tf;
		B_R = lti.b;
		C_R = lti.c;
		IFormula = lti.IFormula;
		DFormula = lti.DFormula;
		if T > 0
			A_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			B_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			C_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			D_I = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			A_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			B_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			C_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			D_D = struct(...
				'Value',	[],...
				'Minimum',	[],...
				'Maximum',	[],...
				'Free',		[],...
				'Scale',	[]...
			);
			if K_I.Free || (~K_I.Free && K_I.Value ~= 0)
				if strcmpi(IFormula, 'ForwardEuler')
					A_I.Value = 1;
					A_I.Free = false;
					A_I.Minimum = 1;
					A_I.Maximum = 1;
					A_I.Scale = 1;
					
					B_I.Value = [
						1,		-1
					];
					B_I.Free = [
						false,  false
					];
					B_I.Minimum = [
						1,		-1
					];
					B_I.Maximum = [
						1,		-1
					];
					B_I.Scale = [
						1,		1
					];
					
					C_I.Value = K_I.Value*T;
					C_I.Free = K_I.Free;
					C_I.Minimum = K_I.Minimum*T;
					C_I.Maximum = K_I.Maximum*T;
					C_I.Scale = K_I.Scale*T;
					
					D_I.Value = 0;
					D_I.Free = false;
					D_I.Minimum = 0;
					D_I.Maximum = 0;
					D_I.Scale = 1;
				elseif strcmpi(IFormula, 'BackwardEuler')
					A_I.Value = 1;
					A_I.Free = false;
					A_I.Minimum = 1;
					A_I.Maximum = 1;
					A_I.Scale = 1;
					
					B_I.Value = [
						1,		-1
					];
					B_I.Free = [
						false,	false
					];
					B_I.Minimum = [
						1,		-1
					];
					B_I.Maximum = [
						1,		-1
					];
					B_I.Scale = [
						1,		1
					];
					
					C_I.Value = K_I.Value*T;
					C_I.Free = K_I.Free;
					C_I.Minimum = K_I.Minimum*T;
					C_I.Maximum = K_I.Maximum*T;
					C_I.Scale = K_I.Scale*T;
					
					D_I.Value = [
						K_I.Value*T,	-K_I.Value*T
					];
					D_I.Free = [
						K_I.Free,		K_I.Free
					];
					D_I.Minimum = [
						K_I.Minimum*T,	-K_I.Maximum*T
					];
					D_I.Maximum = [
						K_I.Maximum*T,	-K_I.Minimum*T
					];
					D_I.Scale = [
						K_I.Scale*T,	K_I.Scale*T
					];
				elseif any(strcmpi(IFormula, {'Trapezoid', 'Trapezoidal'}))
					A_I.Value = 1;
					A_I.Free = false;
					A_I.Minimum = 1;
					A_I.Maximum = 1;
					A_I.Scale = 1;
					
					B_I.Value = [
						2,	-2
					];
					B_I.Free = [
						false,	false
					];
					B_I.Minimum = [
						2,	-2
					];
					B_I.Maximum = [
						2,	-2
					];
					B_I.Scale = [
						1,	1
					];
					
					C_I.Value = K_I.Value*T/2;
					C_I.Free = K_I.Free;
					C_I.Minimum = K_I.Minimum*T/2;
					C_I.Maximum = K_I.Maximum*T/2;
					C_I.Scale = K_I.Scale*T/2;
					
					D_I.Value = [
						K_I.Value*T/2,		-K_I.Value*T/2
					];
					D_I.Free = [
						K_I.Free,			K_I.Free
					];
					D_I.Minimum = [
						K_I.Minimum*T/2,	-K_I.Maximum*T/2
					];
					D_I.Maximum = [
						K_I.Maximum*T/2,	-K_I.Minimum*T/2
					];
					D_I.Scale = [
						K_I.Scale*T/2,		K_I.Scale*T/2
					];
				else
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for integral part of pid2 controller.', IFormula);
				end
			end
			if K_D.Free || (~K_D.Free && K_D.Value ~= 0)
				if strcmpi(DFormula, 'ForwardEuler')
					A_D.Value = 1 - T/T_F.Value;
					A_D.Free = T_F.Free;
					A_D.Minimum = 1 - T/T_F.Minimum;
					A_D.Maximum = 1 - T/T_F.Maximum;
					A_D.Scale = T/T_F.Scale;
					
					B_D.Value = [
						C_R.Value*T/T_F.Value,		-T/T_F.Value
					];
					B_D.Free = [
						C_R.Free | T_F.Free,		T_F.Free
					];
					B_D.Maximum = [
						C_R.Maximum*T/T_F.Minimum,	-T/T_F.Maximum
					];
					B_D.Minimum = [
						C_R.Minimum*T/T_F.Maximum,	-T/T_F.Minimum
					];
					B_D.Scale = [
						C_R.Scale*T/T_F.Scale,		T/T_F.Scale
					];
					
					C_D.Value = K_D.Value/T_F.Value;
					C_D.Free = K_D.Free | T_F.Free;
					C_D.Minimum = K_D.Minimum/T_F.Maximum;
					C_D.Maximum = K_D.Maximum/T_F.Minimum;
					C_D.Scale = K_D.Scale/T_F.Scale;
					
					D_D.Value = [
						C_R.Value*K_D.Value/T_F.Value,			-K_D.Value/T_F.Value
					];
					D_D.Free = [
						C_R.Free | K_D.Value | T_F.Free,		K_D.Value | T_F.Free
					];
					D_D.Maximum = [
						C_R.Maximum*K_D.Maximum/T_F.Minimum,	K_D.Minimum/T_F.Maximum
					];
					D_D.Minimum = [
						C_R.Minimum*K_D.Value/T_F.Maximum,		K_D.Maximum/T_F.Minimum
					];
					D_D.Scale = [
						C_R.Scale*K_D.Value/T_F.Scale,			K_D.Value/T_F.Scale
					];
				elseif strcmpi(DFormula, 'BackwardEuler')
					A_D.Value = T_F.Value/(T_F.Value + T);
					A_D.Free = T_F.Free;
					A_D.Minimum = 0;
					A_D.Maximum = Inf;
					A_D.Scale = 1;
					
					B_D.Value = [
						C_R.Value*T_F.Value/(T_F.Value + T) - C_R.Value,	1 - T_F.Value/(T_F.Value + T)
					];
					B_D.Free = [
						C_R.Free | T_F.Free,						        T_F.Free
					];
					B_D.Minimum = [
						-C_R.Maximum,								        -Inf
					];
					B_D.Maximum = [
						C_R.Maximum*Inf,							         1
					];
					B_D.Scale = [
						C_R.Scale,									         1
					];
					
					C_D.Value = K_D.Value/(T_F.Value + T);
					C_D.Free = K_D.Free | T_F.Free;
					C_D.Minimum = -Inf;
					C_D.Maximum = Inf;
					C_D.Scale = K_D.Scale/T_F.Scale;
					
					D_D.Value = [
						C_R.Value*K_D.Value/(T_F.Value + T),	-K_D.Value/(T_F.Value + T)
					];
					D_D.Free = [
						C_R.Free | K_D.Free | T_F.Free,			K_D.Free | T_F.Free
					];
					D_D.Minimum = [
						-C_R.Maximum*Inf,						-Inf
					];
					D_D.Maximum = [
						C_R.Maximum*Inf,						Inf
					];
					D_D.Scale = [
						C_R.Scale*K_D.Scale/T_F.Scale,			K_D.Scale/T_F.Scale
					];
				elseif any(strcmpi(DFormula, {'Trapezoid', 'Trapezoidal'}))
					A_D.Value = -(T/2 - T_F.Value)/(T_F.Value + T/2);
					A_D.Free = T_F.Free;
					A_D.Minimum = -Inf;
					A_D.Maximum = Inf;
					A_D.Scale = 1;
					
					B_D.Value = [
						C_R.Value + C_R.Value*(T/2 - T_F.Value)/(T_F.Value + T/2),	-1 - (T/2 - T_F.Value)/(T_F.Value + T/2)
					];
					B_D.Free = [
						C_R.Free | T_F.Free,										T_F.Free
					];
					B_D.Minimum = [
						-C_R.Maximum*Inf,											-Inf
					];
					B_D.Maximum = [
						C_R.Maximum*Inf,											Inf
					];
					B_D.Scale = [
						C_R.Scale,													1
					];
					
					C_D.Value = -K_D.Value/(T_F.Value + T/2);
					C_D.Free = K_D.Free | T_F.Free;
					C_D.Minimum = -Inf;
					C_D.Maximum = Inf;
					C_D.Scale = K_D.Scale/T_F.Scale;
					
					D_D.Value = [
						C_R.Value*K_D.Value/(T_F.Value + T/2),	-K_D.Value/(T_F.Value + T/2)
					];
					D_D.Free = [
						C_R.Free | K_D.Free | T_F.Free,			K_D.Free | T_F.Free
					];
					D_D.Minimum = [
						-C_R.Maximum*Inf,						-Inf
					];
					D_D.Maximum = [
						C_R.Maximum*Inf,						Inf
					];
					D_D.Scale = [
						C_R.Scale*K_D.Scale/T_F.Scale,			K_D.Scale/T_F.Scale
					];
				else
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for derivative part of pid2 controller.', IFormula);
				end
			end
			A = struct(...
				'Value',	blkdiag(A_I.Value, A_D.Value),...
				'Minimum',	blkdiag(A_I.Minimum, A_D.Minimum),...
				'Maximum',	blkdiag(A_I.Maximum, A_D.Maximum),...
				'Free',		blkdiag(A_I.Free, A_D.Free),...
				'Scale',	blkdiag(A_I.Scale, A_D.Scale)...
			);
			B = struct(...
				'Value',	[
					B_I.Value;
					B_D.Value
				],...
				'Minimum',	[
					B_I.Minimum;
					B_D.Minimum
				],...
				'Maximum',	[
					B_I.Maximum;
					B_D.Maximum
				],...
				'Free',		[
					B_I.Free;
					B_D.Free
				],...
				'Scale',	[
					B_I.Scale;
					B_D.Scale
				]...
			);
			C = struct(...
				'Value',	[
					C_I.Value,		C_D.Value
				],...
				'Minimum',	[
					C_I.Minimum,	C_D.Minimum
				],...
				'Maximum',	[
					C_I.Maximum,	C_D.Maximum
				],...
				'Free',		[
					C_I.Free,		C_D.Free
				],...
				'Scale',	[
					C_I.Scale,		C_D.Scale
				]...
			);
			D = struct(...
				'Value',	[
					B_R.Value*K_P.Value,		-K_P.Value
				],...
				'Minimum',	[
					B_R.Minimum*K_P.Minimum,	-K_P.Maximum
				],...
				'Maximum',	[
					B_R.Maximum*K_P.Maximum,	-K_P.Minimum
				],...
				'Free',		[
					B_R.Free | K_P.Free,		K_P.Free
				],...
				'Scale',	[
					B_R.Scale*K_P.Scale,		K_P.Scale
				]...
			);
			if ~isempty(D_I.Value)
				D.Value = D.Value + D_I.Value;
				D.Minimum = D.Minimum + D_I.Minimum;
				D.Maximum = D.Maximum + D_I.Maximum;
				D.Free = D.Free | D_I.Free;
				D.Scale = D.Scale + D_I.Scale;
			end
			if ~isempty(D_D.Value)
				D.Value = D.Value + D_D.Value;
				D.Minimum = D.Minimum + D_D.Minimum;
				D.Maximum = D.Maximum + D_D.Maximum;
				D.Free = D.Free | D_D.Free;
				D.Scale = D.Scale + D_D.Scale;
			end
			ltiss = ltiblock.ss(lti.Name, size(A.Value, 1), size(C.Value, 1), size(B.Value, 2), lti.Ts);
			ltiss.a.Value = A.Value;
			ltiss.a.Minimum = A.Minimum;
			ltiss.a.Maximum = A.Maximum;
			ltiss.a.Free = A.Free;
			ltiss.a.Scale = A.Scale;
			ltiss.b.Value = B.Value;
			ltiss.b.Minimum = B.Minimum;
			ltiss.b.Maximum = B.Maximum;
			ltiss.b.Free = B.Free;
			ltiss.b.Scale = B.Scale;
			ltiss.c.Value = C.Value;
			ltiss.c.Minimum = C.Minimum;
			ltiss.c.Maximum = C.Maximum;
			ltiss.c.Free = C.Free;
			ltiss.c.Scale = C.Scale;
			ltiss.d.Value = D.Value;
			ltiss.d.Minimum = D.Minimum;
			ltiss.d.Maximum = D.Maximum;
			ltiss.d.Free = D.Free;
			ltiss.d.Scale = D.Scale;
		else
			hasI = K_I.Free || (~K_I.Free && K_I.Value ~= 0);
			hasD = K_D.Free || (~K_D.Free && K_D.Value ~= 0);
			ltiss = ltiblock.ss(lti.Name, hasI + hasD, 1, 2);
			if hasI && hasD
				ltiss.a.Value = [
					0,	0;
					0,	-1/T_F.Value
				];
				ltiss.a.Free = [
					false,	false;
					false,	T_F.Free
				];
				ltiss.a.Minimum = [
					0,	0;
					0,	-1/T_F.Minimum
				];
				ltiss.a.Maximum = [
					0,	0;
					0,	-1/T_F.Maximum
				];
				ltiss.a.Scale = [
					1,	1;
					1,	1/T_F.Scale
				];
				
				ltiss.b.Value = [
					K_I.Value,					-K_I.Value;
					C_R.Value/T_F.Value,		-1/T_F.Value
				];
				ltiss.b.Free = [
					K_I.Free,					K_I.Free;
					C_R.Free | T_F.Free,		T_F.Free
				];
				ltiss.b.Minimum = [
					K_I.Minimum,				-K_I.Maximum;
					C_R.Minimum/T_F.Maximum,	-1/T_F.Minimum
				];
				ltiss.b.Maximum = [
					K_I.Maximum,				-K_I.Minimum;
					C_R.Maximum/T_F.Minimum,	-1/T_F.Maximum
				];
				ltiss.b.Scale = [
					K_I.Scale,					K_I.Scale;
					C_R.Scale/T_F.Scale,		1/T_F.Scale
				];
				
				ltiss.c.Value = [
					1,	-K_D.Value/T_F.Value
				];
				ltiss.c.Free = [
					false,	K_D.Free | T_F.Free
				];
				ltiss.c.Minimum = [
					1,	K_D.Minimum/T_F.Maximum
				];
				ltiss.c.Maximum = [
					1,	K_D.Maximum/T_F.Minimum
				];
				ltiss.c.Scale = [
					1,	K_D.Scale/T_F.Scale
				];
			elseif hasI
				ltiss.a.Value = 0;
				ltiss.a.Free = false;
				ltiss.a.Minimum = 0;
				ltiss.a.Maximum = 0;
				ltiss.a.Scale = 1;
				
				ltiss.b.Value = [
					K_I.Value,		-K_I.Value
				];
				ltiss.b.Free = [
					K_I.Free,		K_I.Free
				];
				ltiss.b.Minimum = [
					K_I.Minimum,	-K_I.Maximum
				];
				ltiss.b.Maximum = [
					K_I.Maximum,	-K_I.Minimum
				];
				ltiss.b.Scale = [
					K_I.Scale,		K_I.Scale
				];
				
				ltiss.c.Value = 1;
				ltiss.c.Free = false;
				ltiss.c.Minimum = 1;
				ltiss.c.Maximum = 1;
				ltiss.c.Scale = 1;
			elseif hasD
				ltiss.a.Value = -1/T_F.Value;
				ltiss.a.Free = T_F.Free;
				ltiss.a.Minimum = -1/T_F.Minimum;
				ltiss.a.Maximum = -1/T_F.Maximum;
				ltiss.a.Scale = 1/T_F.Scale;
				
				ltiss.b.Value = [
					C_R.Value/T_F.Value,		-1/T_F.Value
				];
				ltiss.b.Free = [
					C_R.Free | T_F.Free,		T_F.Free
				];
				ltiss.b.Minimum = [
					C_R.Minimum/T_F.Maximum,	-1/T_F.Minimum
				];
				ltiss.b.Maximum = [
					C_R.Maximum/T_F.Minimum,	-1/T_F.Maximum
				];
				ltiss.b.Scale = [
					C_R.Scale/T_F.Scale,		1/T_F.Scale
				];
				
				ltiss.c.Value = -K_D.Value/T_F.Value;
				ltiss.c.Free = K_D.Free | T_F.Free;
				ltiss.c.Minimum = K_D.Minimum/T_F.Maximum;
				ltiss.c.Maximum = K_D.Maximum/T_F.Minimum;
				ltiss.c.Scale = K_D.Scale/T_F.Scale;
			else
				ltiss.a.Value = [];
				ltiss.a.Free = [];
				ltiss.a.Minimum = [];
				ltiss.a.Maximum = [];
				ltiss.a.Scale = [];
				
				ltiss.b.Value = zeros(0, 2);
				ltiss.b.Free = false(0, 2);
				ltiss.b.Minimum = -Inf(0, 2);
				ltiss.b.Maximum = Inf(0, 2);
				ltiss.b.Scale = ones(0, 2);
				
				ltiss.c.Value = zeros(1, 0);
				ltiss.c.Free = false(1, 0);
				ltiss.c.Minimum = -Inf(1, 0);
				ltiss.c.Maximum = Inf(1, 0);
				ltiss.c.Scale = ones(1, 0);
			end
			
			if hasD
				ltiss.d.Value = [
					B_R.Value*K_P.Value + C_R.Value*K_D.Value/T_F.Value,			-K_P.Value - K_D.Value/T_F.Value
				];
				ltiss.d.Free = [
					B_R.Free | K_P.Free | C_R.Free | K_D.Free | T_F.Free,			K_P.Free | K_D.Free | T_F.Free
				];
				ltiss.d.Minimum = [
					B_R.Minimum*K_P.Minimum + C_R.Minimum*K_D.Minimum/T_F.Maximum,	-K_P.Maximum + K_D.Minimum/T_F.Maximum
				];
				ltiss.d.Maximum = [
					B_R.Maximum*K_P.Maximum + C_R.Maximum*K_D.Maximum/T_F.Minimum,	-K_P.Minimum + K_D.Maximum/T_F.Minimum
				];
				ltiss.d.Scale = [
					B_R.Scale*K_P.Scale + C_R.Scale*K_D.Scale/T_F.Scale,			K_P.Scale + K_D.Scale/T_F.Scale
				];
			else
				ltiss.d.Value = [
					B_R.Value*K_P.Value,		-K_P.Value
				];
				ltiss.d.Free = [
					B_R.Free | K_P.Free,		K_P.Free
				];
				ltiss.d.Minimum = [
					B_R.Minimum*K_P.Minimum,	-K_P.Maximum
				];
				ltiss.d.Maximum = [
					B_R.Maximum*K_P.Maximum,	-K_P.Minimum
				];
				ltiss.d.Scale = [
					B_R.Scale*K_P.Scale,		K_P.Scale
				];
			end
		end
		return;
	end
	% no valid ltiblock
	error('model:ltiblock:input', 'System must be supplied as ltiblock, not as ''%s''.', class(lti));
end