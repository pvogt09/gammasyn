function [lti] = ltiblockss2ltiblock(ltiss, type)
	%LTIBLOCKSS2LTIBLOCK convert ltiblock.ss to ltiblock.tf or ltiblock.pid or ltiblock.ss
	%	Input:
	%		ltiss:	system to convert from
	%		type:	type of tunable block to convert to or object of the type to convert to
	%	Output:
	%		lti:	system as specified ltiblock
	ltiblocks = {
		'ltiblock.ss',		1;
		%'ss',				1;
		'tunableSS',		1;
		'ltiblock.gain',	2;
		%'gain',			2;
		'tunableGain',		2;
		'ltiblock.tf',		3;
		%'tf',				3;
		'tunableTF',		3;
		'ltiblock.pid',		4;
		%'pid',				4;
		'tunablePID',		4;
		'ltiblock.pid2',	5;
		%'pid2',			5;
		'tunablePID2',		5
	};
	typedtype = [];
	if ~ischar(type)
		if model.isltiblock(type)
			typedtype = type;
			type = class(type);
		else
			error('model:ltiblock:input', 'Type must either be a valid type or an instance of a class to convert to.');
		end
	end
	hasposition = strcmp(type, ltiblocks(:, 1));
	if ~any(hasposition(:))
		error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks(:, 1), ''', '''), '''.']);
	end
	if sum(hasposition(:)) ~= 1
		error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks(:, 1), ''', '''), '''.']);
	end
	islti = isa(ltiss, 'ltiblock.ss');
	istunable = isa(ltiss, 'tunableSS');
	if ~islti && ~istunable
		if model.isltiblock(ltiss)
			haspositionss = strcmp(class(ltiss), ltiblocks(:, 1));
			if ~any(haspositionss(:))
				error('model:ltiblock:input', ['System must be of type ''', strjoin(ltiblocks(:, 1), ''', '''), '''.']);
			end
			if sum(haspositionss(:)) ~= 1
				error('model:ltiblock:input', ['System must be of type ''', strjoin(ltiblocks(:, 1), ''', '''), '''.']);
			end
			if any(ltiblocks{hasposition, 2} == ltiblocks{haspositionss, 2})
				if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{haspositionss, 1})
					lti = ltiss;
					return;
				else
					% static gain
					if any([ltiblocks{haspositionss, 2}] == 2)
						if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{3, 1})
							lti = ltiblock.gain(ltiss.Name, size(ltiss.Gain.Value, 1), size(ltiss.Gain.Value, 2), ltiss.Ts);
						elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{4, 1})
							lti = tunableGain(ltiss.Name, size(ltiss.Gain.Value, 1), size(ltiss.Gain.Value, 2), ltiss.Ts);
						else
							error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([3, 4], 1), ''', '''), '''.']);
						end
						lti.Gain.Value = ltiss.Gain.Value;
						lti.Gain.Minimum = ltiss.Gain.Minimum;
						lti.Gain.Maximum = ltiss.Gain.Maximum;
						lti.Gain.Free = ltiss.Gain.Free;
						lti.Gain.Scale = ltiss.Gain.Scale;
						return;
					end
					% transfer function
					if any([ltiblocks{haspositionss, 2}] == 3)
						if strcmpi(ltiblocks{haspositionss, 1}, ltiblocks{5, 1})
							Nz = length(ltiss.num);
							Nx = length(ltiss.den);
						elseif strcmpi(ltiblocks{haspositionss, 1}, ltiblocks{6, 1})
							Nz = length(ltiss.Numerator);
							Nx = length(ltiss.Denominator);
						else
							error('model:ltiblock:input', ['System must be of type ''', strjoin(ltiblocks([5, 6], 1), ''', '''), '''.']);
						end
						if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{5, 1})
							lti = ltiblock.tf(ltiss.Name, Nz, Nx, ltiss.Ts);
							num = lti.num;
							den = lti.den;
						elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{6, 1})
							lti = tunableTF(ltiss.Name, Nz, Nx, ltiss.Ts);
							num = lti.Numerator;
							den = lti.Denominator;
						else
							error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([5, 6], 1), ''', '''), '''.']);
						end
						if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{5, 1})
							lti.num.Value = num.Value;
							lti.num.Minimum = num.Minimum;
							lti.num.Maximum = num.Maximum;
							lti.num.Free = num.Free;
							lti.num.Scale = num.Scale;
							lti.den.Value = den.Value;
							lti.den.Minimum = den.Minimum;
							lti.den.Maximum = den.Maximum;
							lti.den.Free = den.Free;
							lti.den.Scale = den.Scale;
						elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{6, 1})
							lti.Numerator.Value = num.Value;
							lti.Numerator.Minimum = num.Minimum;
							lti.Numerator.Maximum = num.Maximum;
							lti.Numerator.Free = num.Free;
							lti.Numerator.Scale = num.Scale;
							lti.Denominator.Value = den.Value;
							lti.Denominator.Minimum = den.Minimum;
							lti.Denominator.Maximum = den.Maximum;
							lti.Denominator.Free = den.Free;
							lti.Denominator.Scale = den.Scale;
						else
							error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([5, 6], 1), ''', '''), '''.']);
						end
						return;
					end
					% pid
					if any([ltiblocks{haspositionss, 2}] == 4)
						pidtype = 'P';
						if ltiss.Ki.Free
							pidtype = [pidtype, 'I'];
						end
						if ltiss.Kd.Free || ltiss.Tf.Free
							pidtype = [pidtype, 'D'];
						end
						if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{7, 1})
							lti = ltiblock.pid(ltiss.Name, pidtype, ltiss.Ts);
						elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{8, 1})
							lti = tunablePID(ltiss.Name, pidtype, ltiss.Ts);
						else
							error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([7, 8], 1), ''', '''), '''.']);
						end
						lti.Kp.Value = ltiss.Kp.Value;
						lti.Kp.Minimum = ltiss.Kp.Minimum;
						lti.Kp.Maximum = ltiss.Kp.Maximum;
						lti.Kp.Free = ltiss.Kp.Free;
						lti.Kp.Scale = ltiss.Kp.Scale;
						lti.Ki.Value = ltiss.Ki.Value;
						lti.Ki.Minimum = ltiss.Ki.Minimum;
						lti.Ki.Maximum = ltiss.Ki.Maximum;
						lti.Ki.Free = ltiss.Ki.Free;
						lti.Ki.Scale = ltiss.Ki.Scale;
						lti.Kd.Value = ltiss.Kd.Value;
						lti.Kd.Minimum = ltiss.Kd.Minimum;
						lti.Kd.Maximum = ltiss.Kd.Maximum;
						lti.Kd.Free = ltiss.Kd.Free;
						lti.Kd.Scale = ltiss.Kd.Scale;
						lti.Tf.Value = ltiss.Tf.Value;
						lti.Tf.Minimum = ltiss.Tf.Minimum;
						lti.Tf.Maximum = ltiss.Tf.Maximum;
						lti.Tf.Free = ltiss.Tf.Free;
						lti.Tf.Scale = ltiss.Tf.Scale;
						return;
					end
					% pid2
					if any([ltiblocks{haspositionss, 2}] == 5)
						pidtype = 'P';
						if ltiss.Ki.Free
							pidtype = [pidtype, 'I'];
						end
						if ltiss.Kd.Free || ltiss.Tf.Free
							pidtype = [pidtype, 'D'];
						end
						if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{9, 1})
							lti = ltiblock.pid2(ltiss.Name, pidtype, ltiss.Ts);
						elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{10, 1})
							lti = tunablePID2(ltiss.Name, pidtype, ltiss.Ts);
						else
							error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([9, 10], 1), ''', '''), '''.']);
						end
						lti.Kp.Value = ltiss.Kp.Value;
						lti.Kp.Minimum = ltiss.Kp.Minimum;
						lti.Kp.Maximum = ltiss.Kp.Maximum;
						lti.Kp.Free = ltiss.Kp.Free;
						lti.Kp.Scale = ltiss.Kp.Scale;
						lti.Ki.Value = ltiss.Ki.Value;
						lti.Ki.Minimum = ltiss.Ki.Minimum;
						lti.Ki.Maximum = ltiss.Ki.Maximum;
						lti.Ki.Free = ltiss.Ki.Free;
						lti.Ki.Scale = ltiss.Ki.Scale;
						lti.Kd.Value = ltiss.Kd.Value;
						lti.Kd.Minimum = ltiss.Kd.Minimum;
						lti.Kd.Maximum = ltiss.Kd.Maximum;
						lti.Kd.Free = ltiss.Kd.Free;
						lti.Kd.Scale = ltiss.Kd.Scale;
						lti.Tf.Value = ltiss.Tf.Value;
						lti.Tf.Minimum = ltiss.Tf.Minimum;
						lti.Tf.Maximum = ltiss.Tf.Maximum;
						lti.Tf.Free = ltiss.Tf.Free;
						lti.Tf.Scale = ltiss.Tf.Scale;
						lti.b.Value = ltiss.b.Value;
						lti.b.Minimum = ltiss.b.Minimum;
						lti.b.Maximum = ltiss.b.Maximum;
						lti.b.Free = ltiss.b.Free;
						lti.b.Scale = ltiss.b.Scale;
						lti.c.Value = ltiss.c.Value;
						lti.c.Minimum = ltiss.c.Minimum;
						lti.c.Maximum = ltiss.c.Maximum;
						lti.c.Free = ltiss.c.Free;
						lti.c.Scale = ltiss.c.Scale;
						return;
					end
				end
			else
				ltiss = model.ltiblock2ss(ltiss);
				islti = isa(ltiss, 'ltiblock.ss');
				istunable = isa(ltiss, 'tunableSS');
			end
		else
			error('model:ltiblock:input', 'System model must be of type ''ltiblock.ss''.');
		end
	end
	if islti
		A = ltiss.a;
		B = ltiss.b;
		C = ltiss.c;
		D = ltiss.d;
	elseif istunable
		A = ltiss.A;
		B = ltiss.B;
		C = ltiss.C;
		D = ltiss.D;
	else
		error('model:ltiblock:input', 'system model must be of type ''ltiblock.ss''.');
	end
	% state space
	if any([ltiblocks{hasposition, 2}] == 1)
		if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{1, 1})
			lti = ltiss;
		elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{2, 1})
			lti = tunableSS(ltiss.Name, size(A, 1), size(B, 2), size(C, 1), ltiss.Ts);
			lti.A.Value = A.Value;
			lti.A.Minimum = A.Minimum;
			lti.A.Maximum = A.Maximum;
			lti.A.Free = A.Free;
			lti.A.Scale = A.Scale;
			lti.B.Value = B.Value;
			lti.B.Minimum = B.Minimum;
			lti.B.Maximum = B.Maximum;
			lti.B.Free = B.Free;
			lti.B.Scale = B.Scale;
			lti.C.Value = C.Value;
			lti.C.Minimum = C.Minimum;
			lti.C.Maximum = C.Maximum;
			lti.C.Free = C.Free;
			lti.C.Scale = C.Scale;
			lti.D.Value = D.Value;
			lti.D.Minimum = D.Minimum;
			lti.D.Maximum = D.Maximum;
			lti.D.Free = D.Free;
			lti.D.Scale = D.Scale;
		else
			error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([1, 2], 1), ''', '''), '''.']);
		end
		return;
	end
	% static gain
	if any([ltiblocks{hasposition, 2}] == 2)
		if ~isempty(A.Value) || ~isempty(B.Value) || ~isempty(C.Value)
			error('model:ltiblock:input', 'A, B and C must be empty for conversion from state space to static gain.');
		end
		if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{3, 1})
			lti = ltiblock.gain(ltiss.Name, size(D, 1), size(D, 2));
		elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{4, 1})
			lti = tunableGain(ltiss.Name, size(D, 1), size(D, 2));
		else
			error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([3, 4], 1), ''', '''), '''.']);
		end
		lti.Gain.Value = D.Value;
		lti.Gain.Minimum = D.Minimum;
		lti.Gain.Maximum = D.Maximum;
		lti.Gain.Free = D.Free;
		lti.Gain.Scale = D.Scale;
		return;
	end
	% transfer function
	if any([ltiblocks{hasposition, 2}] == 3)
		if size(B.Value, 2) ~= 1 || size(C.Value, 1) ~= 1
			error('model:ltiblock:input', 'System must be a SISO system for conversion from state space to transfer function.');
		end
		% TODO: handle free parameters
		isctrb = B.Value(end, 1) == 1 && all(B.Value(1:end - 1, 1) == 0);
		isctrb = isctrb && all(A.Value(1:end - 1, 1) == 0);
		A_integrator_chain = A.Value(1:end - 1, 2:end);
		isctrb = isctrb && all(all(A_integrator_chain == eye(size(A_integrator_chain, 1))));
		
		isobsv = C.Value(1, end) == 1 && all(C.Value(1, 1:end - 1) == 0);
		isobsv = isobsv && all(A.Value(1, 1:end - 1) == 0);
		A_integrator_chain = A.Value(2:end, 1:end - 1);
		isobsv = isobsv && all(all(A_integrator_chain == eye(size(A_integrator_chain, 1))));
		if ~isctrb && ~isobsv
			error('model:ltiblock:input', 'System must be in controllable canonical form for conversion from state space to transfer function.');
		end
		Nx = size(A.Value, 1);
		if isctrb
			idx = find(diff(C.Value == 0), 1, 'first');
		elseif isobsv
			idx = find(diff(B.Value == 0), 1, 'first');
		else
			error('model:ltiblock:input', 'System must be in controllable canonical form for conversion from state space to transfer function.');
		end
		if isempty(idx)
			idx = 0;
		end
		Nz = Nx - idx - 1;
		if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{5, 1})
			lti = ltiblock.tf(ltiss.Name, Nz, Nx, ltiss.Ts);
			num = lti.num;
			den = lti.den;
		elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{6, 1})
			lti = tunableTF(ltiss.Name, Nz, Nx, ltiss.Ts);
			num = lti.Numerator;
			den = lti.Denominator;
		else
			error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([5, 6], 1), ''', '''), '''.']);
		end
		if isctrb
			den.Value = [1, -fliplr(A.Value(end, :))];
			den.Minimum = [1, -fliplr(A.Minimum(end, :))];
			den.Maximum = [1, -fliplr(A.Maximum(end, :))];
			den.Free = [false, fliplr(A.Free(end, :))];
			den.Scale = [1, -fliplr(A.Scale(end, :))];
			
			num.Value = C.Value - den.Value(1, 2:end)*D.Value;
			num.Minimum = C.Minimum - den.Maximum(1, 2:end)*D.Maximum;
			num.Maximum = C.Maximum - den.Minimum(1, 2:end)*D.Minimum;
			num.Free = C.Free | den.Free(1, 2:end) | D.Free;
			num.Scale = C.Scale - den.Scale(1, 2:end)*D.Scale;
		elseif isobsv
			den.Value = [1, -A.Value(:, end)'];
			den.Minimum = [1, -A.Minimum(:, end)'];
			den.Maximum = [1, -A.Maximum(:, end)'];
			den.Free = [false, A.Free(:, end)'];
			den.Scale = [1, -A.Scale(:, end)'];
			
			num.Value = B.Value' - den.Value(1, 2:end)*D.Value;
			num.Minimum = B.Minimum' - den.Maximum(1, 2:end)*D.Maximum;
			num.Maximum = B.Maximum' - den.Minimum(1, 2:end)*D.Minimum;
			num.Free = B.Free' | den.Free(1, 2:end) | D.Free;
			num.Scale = B.Scale' - den.Scale(1, 2:end)*D.Scale;
		else
			error('model:ltiblock:input', 'System must be in controllable canonical form for conversion from state space to transfer function.');
		end
		if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{5, 1})
			lti.num.Value = num.Value;
			lti.num.Minimum = num.Minimum;
			lti.num.Maximum = num.Maximum;
			lti.num.Free = num.Free;
			lti.num.Scale = num.Scale;
			lti.den.Value = den.Value;
			lti.den.Minimum = den.Minimum;
			lti.den.Maximum = den.Maximum;
			lti.den.Free = den.Free;
			lti.den.Scale = den.Scale;
		elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{6, 1})
			lti.Numerator.Value = num.Value;
			lti.Numerator.Minimum = num.Minimum;
			lti.Numerator.Maximum = num.Maximum;
			lti.Numerator.Free = num.Free;
			lti.Numerator.Scale = num.Scale;
			lti.Denominator.Value = den.Value;
			lti.Denominator.Minimum = den.Minimum;
			lti.Denominator.Maximum = den.Maximum;
			lti.Denominator.Free = den.Free;
			lti.Denominator.Scale = den.Scale;
		else
			error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([5, 6], 1), ''', '''), '''.']);
		end
		return;
	end
	% pid block
	if any([ltiblocks{hasposition, 2}] == 4)
		if size(A.Value, 1) > 2
			error('model:ltiblock:input', 'PID controller can have at most 2 states.');
		end
		if size(D.Value, 1) ~= 1 || size(D.Value, 2) ~= 1
			error('model:ltiblock:input', 'PID controller must be a SISO system.');
		end
		IFormula = '';
		DFormula = '';
		if ~isempty(typedtype)
			if isa(ltiss, 'ltiblock.pid') || isa(ltiss, 'tunablePID')
				IFormula = ltiss.IFormula;
				DFormula = ltiss.DFormula;
			end
		end
		position_I = 0;
		position_D = 0;
		pidtype = 'P';
		if ltiss.Ts > 0
			if ~isempty(A.Value)
				if A.Value(1, 1) == 1
					position_I = 1;
					pidtype = [pidtype, 'I'];
				elseif size(A.Value, 1) >= 2 && A.Value(2, 2) == 1
					position_I = 2;
					pidtype = [pidtype, 'I'];
				end
				if size(A.Value, 1) >= 2
					position_D = 2;
					pidtype = [pidtype, 'D'];
				elseif A.Value(1, 1) ~= 1
					position_D = 1;
					pidtype = [pidtype, 'D'];
				end
			end
		else
			if ~isempty(A.Value)
				if A.Value(1, 1) == 0
					position_I = 1;
					pidtype = [pidtype, 'I'];
				elseif size(A.Value, 1) >= 2 && A.Value(2, 2) == 0
					position_I = 2;
					pidtype = [pidtype, 'I'];
				end
				if size(A.Value, 1) >= 2
					position_D = 2;
					pidtype = [pidtype, 'D'];
				elseif A.Value(1, 1) ~= 0
					position_D = 1;
					pidtype = [pidtype, 'D'];
				end
			end
		end
		K_P = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
		);
		K_I = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
		);
		K_D = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
		);
		T_F = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
		);
		if ltiss.Ts > 0
			if position_I ~= 0
				if ~strcmpi(IFormula, 'ForwardEuler') && ~strcmpi(IFormula, 'BackwardEuler') && ~any(strcmpi(IFormula, {'Trapezoid', 'Trapezoidal'}))
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for integral part of pid controller.', IFormula);
				end
				K_I.Value = C.Value(1, position_I)*B.Value(position_I, 1)/ltiss.Ts;
				K_I.Minimum = C.Minimum(1, position_I)*B.Minimum(position_I, 1)/ltiss.Ts;
				K_I.Maximum = C.Maximum(1, position_I)*B.Maximum(position_I, 1)/ltiss.Ts;
				K_I.Free = C.Free(1, position_I) || B.Free(position_I, 1);
				K_I.Scale = C.Scale(1, position_I)*B.Scale(position_I, 1)/ltiss.Ts;
			end
			if position_D ~= 0
				if strcmpi(DFormula, 'ForwardEuler')
					T_F.Value = ltiss.Ts/(1 - A.Value(position_D, position_D));
					T_F.Minimum = -Inf;
					T_F.Maximum = Inf;
					T_F.Free = A.Free(position_D, position_D);
					T_F.Scale = ltiss.Ts/(1 - A.Scale(position_D, position_D));
					K_D.Value = C.Value(1, position_D)*B.Value(position_D, 1)*T_F.Value^2/ltiss.Ts;
					K_D.Minimum = C.Minimum(1, position_D)*B.Minimum(position_D, 1)*T_F.Value^2/ltiss.Ts;
					K_D.Maximum = C.Maximum(1, position_D)*B.Maximum(position_D, 1)*T_F.Value^2/ltiss.Ts;
					K_D.Free = C.Free(1, position_D) || B.Free(position_D, 1) || T_F.Free;
					K_D.Scale = C.Scale(1, position_D)*B.Scale(position_D, 1)*T_F.Scale^2/ltiss.Ts;
					K_P.Value = D.Value - K_D.Value/T_F.Value;
					K_P.Minimum = D.Minimum - K_D.Minimum/T_F.Maximum;
					K_P.Maximum = D.Maximum - K_D.Maximum/T_F.Minimum;
					K_P.Free = D.Free || K_D.Free || T_F.Free;
					K_P.Scale = D.Scale - K_D.Scale/T_F.Scale;
				elseif strcmpi(DFormula, 'BackwardEuler')
					T_F.Value = A.Value(position_D, position_D)/(1 - A.Value(position_D, position_D))*ltiss.Ts;
					T_F.Minimum = -Inf;
					T_F.Maximum = Inf;
					T_F.Free = A.Free(position_D, position_D);
					T_F.Scale = A.Scale(position_D, position_D)/(1 - A.Scale(position_D, position_D))*ltiss.Ts;
					K_D.Value = -C.Value(1, position_D)*B.Value(position_D, 1)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts;
					K_D.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, 1)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts;
					K_D.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, 1)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts;
					K_D.Free = C.Free(1, position_D) || B.Free(position_D, 1) || T_F.Free;
					K_D.Scale = -C.Scale(1, position_D)*B.Scale(position_D, 1)*(T_F.Scale + ltiss.Ts)^2/ltiss.Ts;
					K_P.Value = D.Value - K_D.Value/(T_F.Value + ltiss.Ts);
					K_P.Minimum = D.Minimum - K_D.Minimum/(T_F.Maximum + ltiss.Ts);
					K_P.Maximum = D.Maximum - K_D.Maximum/(T_F.Minimum + ltiss.Ts);
					K_P.Free = D.Free || K_D.Free || T_F.Free;
					K_P.Scale = D.Scale - K_D.Scale/(T_F.Scale + ltiss.Ts);
				elseif any(strcmpi(DFormula, {'Trapezoid', 'Trapezoidal'}))
					T_F.Value = (1 + A.Value(position_D, position_D))/(1 - A.Value(position_D, position_D))*ltiss.Ts/2;
					T_F.Minimum = -Inf;
					T_F.Maximum = Inf;
					T_F.Free = A.Free(position_D, position_D);
					T_F.Scale = (1 + A.Scale(position_D, position_D))/(1 - A.Scale(position_D, position_D))*ltiss.Ts/2;
					K_D.Value = -C.Value(1, position_D)*B.Value(position_D, 1)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts;
					K_D.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, 1)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts;
					K_D.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, 1)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts;
					K_D.Free = C.Free(1, position_D) || B.Free(position_D, 1) || T_F.Free;
					K_D.Scale = -C.Scale(1, position_D)*B.Scale(position_D, 1)*(T_F.Scale + ltiss.Ts/2)^2/ltiss.Ts;
					K_P.Value = D.Value - K_D.Value/(T_F.Value + ltiss.Ts/2);
					K_P.Minimum = D.Minimum - K_D.Minimum/(T_F.Maximum + ltiss.Ts/2);
					K_P.Maximum = D.Maximum - K_D.Maximum/(T_F.Minimum + ltiss.Ts/2);
					K_P.Free = D.Free || K_D.Free || T_F.Free;
					K_P.Scale = D.Scale - K_D.Scale/(T_F.Scale + ltiss.Ts/2);
				else
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for derivative part of pid controller.', DFormula);
				end
			else
				K_P.Value = D.Value;
				K_P.Minimum = D.Minimum;
				K_P.Maximum = D.Maximum;
				K_P.Free = D.Free;
				K_P.Scale = D.Scale;
			end
		else
			if position_I ~= 0
				K_I.Value = C.Value(1, position_I)*B.Value(position_I, 1);
				K_I.Minimum = C.Minimum(1, position_I)*B.Minimum(position_I, 1);
				K_I.Maximum = C.Maximum(1, position_I)*B.Maximum(position_I, 1);
				K_I.Free = C.Free(1, position_I) || B.Free(position_I, 1);
				K_I.Scale = C.Scale(1, position_I)*B.Scale(position_I, 1);
			end
			if position_D ~= 0
				T_F.Value = -1/A.Value(position_D, position_D);
				T_F.Minimum = -1/A.Minimum(position_D, position_D);
				T_F.Maximum = -1/A.Maximum(position_D, position_D);
				T_F.Free = A.Free(position_D, position_D);
				T_F.Scale = -1/A.Scale(position_D, position_D);
				K_D.Value = -C.Value(1, position_D)*B.Value(position_D, 1)*T_F.Value^2;
				K_D.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, 1)*T_F.Value^2;
				K_D.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, 1)*T_F.Value^2;
				K_D.Free = C.Free(1, position_D) || B.Free(position_D, 1) || T_F.Free;
				K_D.Scale = -C.Scale(1, position_D)*B.Scale(position_D, 1)*T_F.Scale^2;
				K_P.Value = D.Value - K_D.Value/T_F.Value;
				K_P.Minimum = D.Minimum - K_D.Minimum/T_F.Maximum;
				K_P.Maximum = D.Maximum - K_D.Maximum/T_F.Minimum;
				K_P.Free = D.Free || K_D.Free || T_F.Free;
				K_P.Scale = D.Scale - K_D.Scale/T_F.Scale;
			else
				K_P.Value = D.Value;
				K_P.Minimum = D.Minimum;
				K_P.Maximum = D.Maximum;
				K_P.Free = D.Free;
				K_P.Scale = D.Scale;
			end
		end
		if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{7, 1})
			lti = ltiblock.pid(ltiss.Name, pidtype, ltiss.Ts);
		elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{8, 1})
			lti = tunablePID(ltiss.Name, pidtype, ltiss.Ts);
		else
			error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([7, 8], 1), ''', '''), '''.']);
		end
		if ~isempty(K_P.Value)
			lti.Kp.Value = K_P.Value;
			lti.Kp.Minimum = K_P.Minimum;
			lti.Kp.Maximum = K_P.Maximum;
			lti.Kp.Free = K_P.Free;
			lti.Kp.Scale = K_P.Scale;
		end
		if ~isempty(K_I.Value)
			lti.Ki.Value = K_I.Value;
			lti.Ki.Minimum = K_I.Minimum;
			lti.Ki.Maximum = K_I.Maximum;
			lti.Ki.Free = K_I.Free;
			lti.Ki.Scale = K_I.Scale;
		end
		if ~isempty(K_D.Value)
			lti.Kd.Value = K_D.Value;
			lti.Kd.Minimum = K_D.Minimum;
			lti.Kd.Maximum = K_D.Maximum;
			lti.Kd.Free = K_D.Free;
			lti.Kd.Scale = K_D.Scale;
		end
		if ~isempty(T_F.Value)
			lti.Tf.Value = T_F.Value;
			lti.Tf.Minimum = T_F.Minimum;
			lti.Tf.Maximum = T_F.Maximum;
			lti.Tf.Free = T_F.Free;
			lti.Tf.Scale = T_F.Scale;
		end
		return;
	end
	% 2DOF pid block
	if any([ltiblocks{hasposition, 2}] == 5)
		if size(A.Value, 1) > 2
			error('model:ltiblock:input', 'PID2 controller can have at most 2 states.');
		end
		if size(D.Value, 1) ~= 1 || size(D.Value, 2) ~= 2
			error('model:ltiblock:input', 'PID2 controller must be a SISO system.');
		end
		IFormula = '';
		DFormula = '';
		if ~isempty(typedtype)
			if isa(ltiss, 'ltiblock.pid2') || isa(ltiss, 'tunablePID2')
				IFormula = ltiss.IFormula;
				DFormula = ltiss.DFormula;
			end
		end
		position_I = 0;
		position_D = 0;
		position_R = 1;
		position_Y = 2;
		pidtype = 'P';
		if ltiss.Ts > 0
			if ~isempty(A.Value)
				if A.Value(1, 1) == 1
					position_I = 1;
					pidtype = [pidtype, 'I'];
				elseif size(A.Value, 1) >= 2 && A.Value(2, 2) == 1
					position_I = 2;
					pidtype = [pidtype, 'I'];
				end
				if size(A.Value, 1) >= 2
					position_D = 2;
					pidtype = [pidtype, 'D'];
				elseif A.Value(1, 1) ~= 1
					position_D = 1;
					pidtype = [pidtype, 'D'];
				end
			end
		else
			if ~isempty(A.Value)
				if A.Value(1, 1) == 0
					position_I = 1;
					pidtype = [pidtype, 'I'];
				elseif size(A.Value, 1) >= 2 && A.Value(2, 2) == 0
					position_I = 2;
					pidtype = [pidtype, 'I'];
				end
				if size(A.Value, 1) >= 2
					position_D = 2;
					pidtype = [pidtype, 'D'];
				elseif A.Value(1, 1) ~= 0
					position_D = 1;
					pidtype = [pidtype, 'D'];
				end
			end
		end
		K_P = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
			);
		K_I = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
			);
		K_D = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
			);
		T_F = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
			);
		B_R = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
			);
		C_R = struct(...
			'Value',	[],...
			'Minimum',	[],...
			'Maximum',	[],...
			'Free',		[],...
			'Scale',	[]...
			);
		if ltiss.Ts > 0
			if position_I ~= 0
				if ~strcmpi(IFormula, 'ForwardEuler') && ~strcmpi(IFormula, 'BackwardEuler') && ~any(strcmpi(IFormula, {'Trapezoid', 'Trapezoidal'}))
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for integral part of pid2 controller.', IFormula);
				end
				K_I.Value = C.Value(1, position_I)*B.Value(position_I, position_R)/ltiss.Ts;
				K_I.Minimum = C.Minimum(1, position_I)*B.Minimum(position_I, position_R)/ltiss.Ts;
				K_I.Maximum = C.Maximum(1, position_I)*B.Maximum(position_I, position_R)/ltiss.Ts;
				K_I.Free = C.Free(1, position_I) || B.Free(position_I, position_R);
				K_I.Scale = C.Scale(1, position_I)*B.Scale(position_I, position_R)/ltiss.Ts;
			end
			if position_D ~= 0
				if strcmpi(DFormula, 'ForwardEuler')
					T_F.Value = ltiss.Ts/(1 - A.Value(position_D, position_D));
					T_F.Minimum = -Inf;
					T_F.Maximum = Inf;
					T_F.Free = A.Free(position_D, position_D);
					T_F.Scale = ltiss.Ts/(1 - A.Scale(position_D, position_D));
					K_D.Value = -C.Value(1, position_D)*B.Value(position_D, position_Y)*T_F.Value^2/ltiss.Ts;
					K_D.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, position_Y)*T_F.Value^2/ltiss.Ts;
					K_D.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, position_Y)*T_F.Value^2/ltiss.Ts;
					K_D.Free = C.Free(1, position_D) || B.Free(position_D, position_Y) || T_F.Free;
					K_D.Scale = -C.Scale(1, position_D)*B.Scale(position_D, position_Y)*T_F.Scale^2/ltiss.Ts;
					C_R.Value = C.Value(1, position_D)*B.Value(position_D, position_R)*T_F.Value^2/ltiss.Ts/K_D.Value;
					C_R.Minimum = C.Minimum(1, position_D)*B.Minimum(position_D, position_R)*T_F.Value^2/ltiss.Ts/K_D.Maximum;
					C_R.Maximum = C.Maximum(1, position_D)*B.Maximum(position_D, position_R)*T_F.Value^2/ltiss.Ts/K_D.Minimum;
					C_R.Free = C.Free(1, position_D) || B.Free(position_D, position_R) || T_F.Free || K_D.Free;
					C_R.Scale = C.Scale(1, position_D)*B.Scale(position_D, position_R)*T_F.Scale^2/ltiss.Ts/K_D.Scale;
					K_P.Value = -D.Value(1, position_Y) - K_D.Value/T_F.Value;
					K_P.Minimum = -D.Minimum(1, position_Y) - K_D.Minimum/T_F.Maximum;
					K_P.Maximum = -D.Maximum(1, position_Y) - K_D.Maximum/T_F.Minimum;
					K_P.Free = D.Free(1, position_Y) || K_D.Free || T_F.Free;
					K_P.Scale = -D.Scale(1, position_Y) - K_D.Scale/T_F.Scale;
					B_R.Value = (D.Value(1, position_R) - C_R.Value*K_D.Value/T_F.Value)/K_P.Value;
					B_R.Minimum = (D.Minimum(1, position_R) - C_R.Value*K_D.Minimum/T_F.Maximum)/K_P.Maximum;
					B_R.Maximum = (D.Maximum(1, position_R) - C_R.Value*K_D.Maximum/T_F.Minimum)/K_P.Minimum;
					B_R.Free = D.Free(1, position_R) || C_R.Free || K_D.Free || T_F.Free;
					B_R.Scale = (D.Scale(1, position_R) - C_R.Scale*K_D.Scale/T_F.Scale)/K_P.Scale;
				elseif strcmpi(DFormula, 'BackwardEuler')
					T_F.Value = A.Value(position_D, position_D)/(1 - A.Value(position_D, position_D))*ltiss.Ts;
					T_F.Minimum = -Inf;
					T_F.Maximum = Inf;
					T_F.Free = A.Free(position_D, position_D);
					T_F.Scale = A.Scale(position_D, position_D)/(1 - A.Scale(position_D, position_D))*ltiss.Ts;
					K_D.Value = C.Value(1, position_D)*B.Value(position_D, position_Y)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts;
					K_D.Minimum = C.Minimum(1, position_D)*B.Minimum(position_D, position_Y)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts;
					K_D.Maximum = C.Maximum(1, position_D)*B.Maximum(position_D, position_Y)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts;
					K_D.Free = C.Free(1, position_D) || B.Free(position_D, position_Y) || T_F.Free;
					K_D.Scale = C.Scale(1, position_D)*B.Scale(position_D, position_Y)*(T_F.Scale + ltiss.Ts)^2/ltiss.Ts;
					C_R.Value = -C.Value(1, position_D)*B.Value(position_D, position_R)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts/K_D.Value;
					C_R.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, position_R)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts/K_D.Minimum;
					C_R.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, position_R)*(T_F.Value + ltiss.Ts)^2/ltiss.Ts/K_D.Maximum;
					C_R.Free = C.Free(1, position_D) || B.Free(position_D, position_R) || T_F.Free || K_D.Free;
					C_R.Scale = -C.Scale(1, position_D)*B.Scale(position_D, position_R)*(T_F.Scale + ltiss.Ts)^2/ltiss.Ts/K_D.Scale;
					K_P.Value = -D.Value(1, position_Y) - K_D.Value/(T_F.Value + ltiss.Ts);
					K_P.Minimum = -D.Minimum(1, position_Y) - K_D.Minimum/(T_F.Maximum + ltiss.Ts);
					K_P.Maximum = -D.Maximum(1, position_Y) - K_D.Maximum/(T_F.Minimum + ltiss.Ts);
					K_P.Free = D.Free(1, position_Y) || K_D.Free || T_F.Free;
					K_P.Scale = -D.Scale(1, position_Y) - K_D.Scale/(T_F.Scale + ltiss.Ts);
					B_R.Value = (D.Value(1, position_R) - C_R.Value*K_D.Value/(T_F.Value + ltiss.Ts))/K_P.Value;
					B_R.Minimum = (D.Minimum(1, position_R) - C_R.Minimum*K_D.Minimum/(T_F.Maximum + ltiss.Ts))/K_P.Minimum;
					B_R.Maximum = (D.Maximum(1, position_R) - C_R.Maximum*K_D.Maximum/(T_F.Minimum + ltiss.Ts))/K_P.Maximum;
					B_R.Free = D.Free(1, position_R) || C_R.Free || K_D.Free || T_F.Free || K_P.Free;
					B_R.Scale = (D.Scale(1, position_R) - C_R.Scale*K_D.Scale/(T_F.Scale + ltiss.Ts))/K_P.Scale;
				elseif any(strcmpi(DFormula, {'Trapezoid', 'Trapezoidal'}))
					T_F.Value = (1 + A.Value(position_D, position_D))/(1 - A.Value(position_D, position_D))*ltiss.Ts/2;
					T_F.Minimum = -Inf;
					T_F.Maximum = Inf;
					T_F.Free = A.Free(position_D, position_D);
					T_F.Scale = (1 + A.Scale(position_D, position_D))/(1 - A.Scale(position_D, position_D))*ltiss.Ts/2;
					K_D.Value = C.Value(1, position_D)*B.Value(position_D, position_Y)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts;
					K_D.Minimum = C.Minimum(1, position_D)*B.Minimum(position_D, position_Y)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts;
					K_D.Maximum = C.Maximum(1, position_D)*B.Maximum(position_D, position_Y)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts;
					K_D.Free = C.Free(1, position_D) || B.Free(position_D, position_Y) || T_F.Free;
					K_D.Scale = C.Scale(1, position_D)*B.Scale(position_D, position_Y)*(T_F.Scale + ltiss.Ts/2)^2/ltiss.Ts;
					C_R.Value = -C.Value(1, position_D)*B.Value(position_D, position_R)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts/K_D.Value;
					C_R.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, position_R)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts/K_D.Minimum;
					C_R.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, position_R)*(T_F.Value + ltiss.Ts/2)^2/ltiss.Ts/K_D.Maximum;
					C_R.Free = C.Free(1, position_D) || B.Free(position_D, position_R) || T_F.Free || K_D.Free;
					C_R.Scale = -C.Scale(1, position_D)*B.Scale(position_D, position_R)*(T_F.Scale + ltiss.Ts/2)^2/ltiss.Ts/K_D.Scale;
					K_P.Value = -D.Value(1, position_Y) - K_D.Value/(T_F.Value + ltiss.Ts/2);
					K_P.Minimum = -D.Minimum(1, position_Y) - K_D.Minimum/(T_F.Maximum + ltiss.Ts/2);
					K_P.Maximum = -D.Maximum(1, position_Y) - K_D.Maximum/(T_F.Minimum + ltiss.Ts/2);
					K_P.Free = D.Free(1, position_Y) || K_D.Free || T_F.Free;
					K_P.Scale = -D.Scale(1, position_Y) - K_D.Scale/(T_F.Scale + ltiss.Ts/2);
					B_R.Value = (D.Value(1, position_R) - C_R.Value*K_D.Value/(T_F.Value + ltiss.Ts/2))/K_P.Value;
					B_R.Minimum = (D.Minimum(1, position_R) - C_R.Minimum*K_D.Minimum/(T_F.Maximum + ltiss.Ts/2))/K_P.Minimum;
					B_R.Maximum = (D.Maximum(1, position_R) - C_R.Maximum*K_D.Maximum/(T_F.Minimum + ltiss.Ts/2))/K_P.Maximum;
					B_R.Free = D.Free(1, position_R) || C_R.Free || K_D.Free || T_F.Free || K_P.Free;
					B_R.Scale = (D.Scale(1, position_R) - C_R.Scale*K_D.Scale/(T_F.Scale + ltiss.Ts/2))/K_P.Scale;
				else
					error('model:ltiblock:input', 'Undefined discretization method ''%s'' for derivative part of pid controller.', DFormula);
				end
			else
				K_P.Value = -D.Value(1, position_Y);
				K_P.Minimum = -D.Maximum(1, position_Y);
				K_P.Maximum = -D.Minimum(1, position_Y);
				K_P.Free = D.Free(1, position_Y);
				K_P.Scale = -D.Scale(1, position_Y);
				B_R.Value = D.Value(1, position_R)/K_P.Value;
				B_R.Minimum = D.Minimum(1, position_R)/K_P.Maximum;
				B_R.Maximum = D.Maximum(1, position_R)/K_P.Minimum;
				B_R.Free = D.Free(1, position_R) || K_P.Free;
				B_R.Scale = D.Scale(1, position_R)/K_P.Scale;
			end
		else
			if position_I ~= 0
				K_I.Value = C.Value(1, position_I)*B.Value(position_I, position_R);
				K_I.Minimum = C.Minimum(1, position_I)*B.Minimum(position_I, position_R);
				K_I.Maximum = C.Maximum(1, position_I)*B.Maximum(position_I, position_R);
				K_I.Free = C.Free(1, position_I) || B.Free(position_I, position_R);
				K_I.Scale = C.Scale(1, position_I)*B.Scale(position_I, position_R);
			end
			if position_D ~= 0
				T_F.Value = -1/A.Value(position_D, position_D);
				T_F.Minimum = -1/A.Minimum(position_D, position_D);
				T_F.Maximum = -1/A.Maximum(position_D, position_D);
				T_F.Free = A.Free(position_D, position_D);
				T_F.Scale = -1/A.Scale(position_D, position_D);
				K_D.Value = C.Value(1, position_D)*B.Value(position_D, position_Y)*T_F.Value^2;
				K_D.Minimum = C.Minimum(1, position_D)*B.Minimum(position_D, position_Y)*T_F.Value^2;
				K_D.Maximum = C.Maximum(1, position_D)*B.Maximum(position_D, position_Y)*T_F.Value^2;
				K_D.Free = C.Free(1, position_D) || B.Free(position_D, position_Y) || T_F.Free;
				K_D.Scale = C.Scale(1, position_D)*B.Scale(position_D, position_Y)*T_F.Scale^2;
				C_R.Value = -C.Value(1, position_D)*B.Value(position_D, position_R)*T_F.Value^2/K_D.Value;
				C_R.Minimum = -C.Minimum(1, position_D)*B.Minimum(position_D, position_R)*T_F.Value^2/K_D.Maximum;
				C_R.Maximum = -C.Maximum(1, position_D)*B.Maximum(position_D, position_R)*T_F.Value^2/K_D.Minimum;
				C_R.Free = C.Free(1, position_D) || B.Free(position_D, position_R) || T_F.Free || K_D.Free;
				C_R.Scale = -C.Scale(1, position_D)*B.Scale(position_D, position_R)*T_F.Scale^2/K_D.Scale;
				K_P.Value = -D.Value(1, position_Y) - K_D.Value/T_F.Value;
				K_P.Minimum = -D.Minimum(1, position_Y) - K_D.Minimum/T_F.Maximum;
				K_P.Maximum = -D.Maximum(1, position_Y) - K_D.Maximum/T_F.Minimum;
				K_P.Free = D.Free(1, position_Y) || K_D.Free || T_F.Free;
				K_P.Scale = -D.Scale(1, position_Y) - K_D.Scale/T_F.Scale;
				B_R.Value = (D.Value(1, position_R) - C_R.Value*K_D.Value/T_F.Value)/K_P.Value;
				B_R.Minimum = (D.Minimum(1, position_R) - C_R.Minimum*K_D.Minimum/T_F.Maximum)/K_P.Maximum;
				B_R.Maximum = (D.Maximum(1, position_R) - C_R.Maximum*K_D.Maximum/T_F.Minimum)/K_P.Minimum;
				B_R.Free = D.Free(1, position_R) || C_R.Free || K_D.Free || T_F.Free || K_P.Free;
				B_R.Scale = (D.Scale(1, position_R) - C_R.Scale*K_D.Scale/T_F.Scale)/K_P.Scale;
			else
				K_P.Value = -D.Value(1, position_Y);
				K_P.Minimum = -D.Maximum(1, position_Y);
				K_P.Maximum = -D.Minimum(1, position_Y);
				K_P.Free = D.Free(1, position_Y);
				K_P.Scale = D.Scale(1, position_Y);
				B_R.Value = D.Value(1, position_R)/K_P.Value;
				B_R.Minimum = D.Minimum(1, position_R)/K_P.Maximum;
				B_R.Maximum = D.Maximum(1, position_R)/K_P.Minimum;
				B_R.Free = D.Free(1, position_R) || K_P.Free;
				B_R.Scale = D.Scale(1, position_R)/K_P.Scale;
			end
		end
		if strcmpi(ltiblocks{hasposition, 1}, ltiblocks{9, 1})
			lti = ltiblock.pid2(ltiss.Name, pidtype, ltiss.Ts);
		elseif strcmpi(ltiblocks{hasposition, 1}, ltiblocks{10, 1})
			lti = tunablePID2(ltiss.Name, pidtype, ltiss.Ts);
		else
			error('model:ltiblock:input', ['Type must be one of ''', strjoin(ltiblocks([9, 10], 1), ''', '''), '''.']);
		end
		if ~isempty(K_P.Value)
			lti.Kp.Value = K_P.Value;
			lti.Kp.Minimum = K_P.Minimum;
			lti.Kp.Maximum = K_P.Maximum;
			lti.Kp.Free = K_P.Free;
			lti.Kp.Scale = K_P.Scale;
		end
		if ~isempty(K_I.Value)
			lti.Ki.Value = K_I.Value;
			lti.Ki.Minimum = K_I.Minimum;
			lti.Ki.Maximum = K_I.Maximum;
			lti.Ki.Free = K_I.Free;
			lti.Ki.Scale = K_I.Scale;
		end
		if ~isempty(K_D.Value)
			lti.Kd.Value = K_D.Value;
			lti.Kd.Minimum = K_D.Minimum;
			lti.Kd.Maximum = K_D.Maximum;
			lti.Kd.Free = K_D.Free;
			lti.Kd.Scale = K_D.Scale;
		end
		if ~isempty(T_F.Value)
			lti.Tf.Value = T_F.Value;
			lti.Tf.Minimum = T_F.Minimum;
			lti.Tf.Maximum = T_F.Maximum;
			lti.Tf.Free = T_F.Free;
			lti.Tf.Scale = T_F.Scale;
		end
		if ~isempty(B_R.Value)
			lti.b.Value = B_R.Value;
			lti.b.Minimum = B_R.Minimum;
			lti.b.Maximum = B_R.Maximum;
			lti.b.Free = B_R.Free;
			lti.b.Scale = B_R.Scale;
		end
		if ~isempty(C_R.Value)
			lti.c.Value = C_R.Value;
			lti.c.Minimum = C_R.Minimum;
			lti.c.Maximum = C_R.Maximum;
			lti.c.Free = C_R.Free;
			lti.c.Scale = C_R.Scale;
		end
		return;
	end
	% no valid ltiblock
	error('model:ltiblock:input', 'System must be supplied as ltiblock, not as ''%s''.', class(ltiss));
end