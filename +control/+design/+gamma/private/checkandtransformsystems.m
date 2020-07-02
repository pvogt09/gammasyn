function [system, number_states, number_states_all, number_controls, number_measurements, number_measurements_xdot, number_references, descriptor, number_descriptors_all, derivative_feedback, sample_time, expanded] = checkandtransformsystems(systems, systemoptions)
	%CHECKANDTRANSFORMSYSTEMS check and convert systems for gammasyn into faster data structures to use with the objective function
	%	Input:
	%		systems:					structure/cell array or matrix with dynamic systems to take into consideration
	%		systemoptions:				structure with options for multiple models
	%	Output:
	%		system:						structure with system matrices of systems to take into consideration
	%		number_states:				maximum number of states for all systems
	%		number_states_all:			vector with orders of all systems
	%		number_controls:			number of controls of the systems
	%		number_measurements:		number of measurements of the systems
	%		number_measurements_xdot:	number of derivative measurements of the systems
	%		number_references:			number of reference inputs of the systems
	%		descriptor:					indicator, if all of the systems are real descriptor systems
	%		number_descriptors_all:		vector with descriptor number of all systems
	%		derivative_feedback:		indicator, which models have derivative feedback
	%		sample_time:				sample time of systems
	%		expanded:					number of systems that were expanded from a single entry
	if nargin <= 1 || isempty(systemoptions)
		systemoptions = struct();
	end
	if isa(systemoptions, 'control.design.gamma.GammasynOptions')
		systemoptions = struct(systemoptions);
		systemoptions = systemoptions.system;
	end
	if ~isstruct(systemoptions)
		error('control:design:gamma:input', 'Options for multiple models must be a structure.');
	end
	options = struct(...
		'usereferences',		true,...
		'usemeasurements_xdot',	true,...
		'samples',				struct()...
	);
	if isfield(systemoptions, 'usereferences') && ~isempty(systemoptions.usereferences)
		if ~islogical(systemoptions.usereferences) || ~isscalar(systemoptions.usereferences)
			error('control:design:gamma:input', 'Indicator for usage of reference outputs must be a logical scalar.');
		end
		options.usereferences = systemoptions.usereferences;
	end
	if isfield(systemoptions, 'usemeasurements_xdot') && ~isempty(systemoptions.usemeasurements_xdot)
		if ~islogical(systemoptions.usemeasurements_xdot) || ~isscalar(systemoptions.usemeasurements_xdot)
			error('control:design:gamma:input', 'Indicator for usage of derivative measurements outputs must be a logical scalar.');
		end
		options.usemeasurements_xdot = systemoptions.usemeasurements_xdot;
	end
	if isfield(systemoptions, 'samples') && ~isempty(systemoptions.samples)
		if ~isstruct(systemoptions.samples)
			error('control:design:gamma:input', 'Definition of number of sampling points for parameters must be a structure.');
		end
		names = fieldnames(systemoptions.samples);
		if ~isempty(names)
			for ii = 1:size(names, 1) %#ok<FORPF> no parfor because of error checking
				number = systemoptions.samples.(names{ii, 1});
				if ~isnumeric(number)
					error('control:design:gamma:input', 'Number of samples for parameter ''%s'' must be numeric.', names{ii, 1});
				end
				if number < 0
					error('control:design:gamma:input', 'Number of samples for parameter ''%s'' must be nonnegative.', names{ii, 1});
				end
				if isinf(number)
					error('control:design:gamma:input', 'Number of samples for parameter ''%s'' must not be Inf.', names{ii, 1});
				end
			end
			options.samples = systemoptions.samples;
		end
	elseif isfield(systemoptions, 'Blocks') && ~isempty(systemoptions.Blocks)
		if ~isstruct(systemoptions.Blocks)
			error('control:design:gamma:input', 'Definition of number of sampling points for parameters must be a structure.');
		end
		names = fieldnames(systemoptions.Blocks);
		if ~isempty(names)
			for ii = 1:size(names, 1) %#ok<FORPF> no parfor because of error checking
				number = systemoptions.Blocks.(names{ii, 1});
				if ~isnumeric(number)
					error('control:design:gamma:input', 'Number of samples for parameter ''%s'' must be numeric.', names{ii, 1});
				end
				if number < 0
					error('control:design:gamma:input', 'Number of samples for parameter ''%s'' must be nonnegative.', names{ii, 1});
				end
				if isinf(number)
					error('control:design:gamma:input', 'Number of samples for parameter ''%s'' must not be Inf.', names{ii, 1});
				end
			end
			options.samples = systemoptions.Blocks;
		end
	end
	ispsys = isnumeric(systems);
	if ispsys
		if ndims(systems) > 2 %#ok<ISMAT> compatibility with Octave
			ispsys = ltisys.ispsysarray(systems);
			isltisys = ltisys.isltisysarray(systems);
			if all(isltisys(:))
				sz = size(systems);
				systems = reshape(systems, [sz(1), sz(2), prod(sz(3:end))]);
				systems = ltisys.ltisys2ss(systems);
			elseif all(ispsys(:))
				sz = size(systems);
				systems = reshape(systems, [sz(1), sz(2), prod(sz(3:end))]);
				systems = ltisys.psys2cornermodel(systems);
			end
		else
			if ltisys.isltisys(systems)
				systems = ltisys.ltisys2ss(systems);
			elseif ltisys.ispsys(systems)
				systems = ltisys.psys2cornermodel(systems);
			end
		end
	end
	istf = isa(systems, 'tf');
	isss = isa(systems, 'ss');
	isuss = isa(systems, 'uss');
	isgenss = isa(systems, 'genss');
	if istf || isss || isuss || isgenss
		number_models = 1;
	else
		if iscell(systems)
			systems = reshape(systems, [], 1);
		end
		number_models = size(systems, 1);
	end
	number_states_temp = cell(number_models, 1);
	number_descriptors_temp = cell(number_models, 1);
	number_controls_temp = cell(number_models, 1);
	number_measurements_temp = cell(number_models, 1);
	number_measurements_xdot_temp = cell(number_models, 1);
	number_references_temp = cell(number_models, 1);
	derivative_feedback = cell(number_models, 1);
	sample_times_temp = cell(number_models, 1);
	descriptor_temp = cell(number_models, 1);
	expanded_temp = cell(number_models, 1);
	system = cell(number_models, 1);
	for jj = 1:number_models %#ok<FORPF> no parfor because systems can have different datatypes which crashes parfor
		if iscell(systems) && isnumeric(systems{jj})
			if ndims(systems{jj}) > 2 %#ok<ISMAT> compatibility with Octave
				ispsys = ltisys.ispsysarray(systems{jj});
				isltisys = ltisys.isltisysarray(systems{jj});
				if all(isltisys(:))
					sz = size(systems{jj});
					systems{jj} = reshape(systems{jj}, [sz(1), sz(2), prod(sz(3:end))]);
					systems{jj} = ltisys.ltisys2ss(systems{jj});
				elseif all(ispsys(:))
					sz = size(systems{jj});
					systems{jj} = reshape(systems{jj}, [sz(1), sz(2), prod(sz(3:end))]);
					systems{jj} = ltisys.psys2cornermodel(systems{jj});
				end
			else
				if ltisys.isltisys(systems{jj})
					systems{jj} = ltisys.ltisys2ss(systems{jj});
				elseif ltisys.ispsys(systems{jj})
					systems{jj} = ltisys.psys2cornermodel(systems{jj});
				end
			end
		end
		if isstruct(systems) && isstruct(systems(jj))
			if all(isfield(systems(jj), {'A', 'B', 'C'}))
				number_states_temp{jj} = size(systems(jj).A, 1);
				if number_states_temp{jj} ~= size(systems(jj).A, 2)
					error('control:design:gamma:dimension', 'A must be square.');
				end
				number_controls_temp{jj} = size(systems(jj).B, 2);
				if number_states_temp{jj} ~= size(systems(jj).B, 1)
					error('control:design:gamma:dimension', 'B must be %dXx.', number_states_temp{jj});
				end
				number_measurements_temp{jj} = size(systems(jj).C, 1);
				if number_states_temp{jj} ~= size(systems(jj).C, 2)
					error('control:design:gamma:dimension', 'C must be xX%d.', number_states_temp{jj});
				end
				if isfield(systems(jj), 'E') && ~isempty(systems(jj).E)
					number_descriptors_temp{jj} = size(systems(jj).E, 1);
					if number_descriptors_temp{jj} ~= size(systems(jj).E, 2)
						error('control:design:gamma:dimension', 'E must be square.');
					end
					if number_descriptors_temp{jj} ~= number_states_temp{jj}
						error('control:design:gamma:dimension', 'E must be %dX%d.', number_states_temp{jj}, number_states_temp{jj});
					end
					if isdiag(systems(jj).E) && all(diag(systems(jj).E) == 1)
						descriptor_temp{jj} = false;
						E = systems(jj).E;
					else
						descriptor_temp{jj} = true;
						E = systems(jj).E;
					end
				else
					descriptor_temp{jj} = false;
					E = eye(number_states_temp{jj});
					number_descriptors_temp{jj} = number_states_temp{jj};
				end
				if isfield(systems(jj), 'D')
					if number_measurements_temp{jj} ~= size(systems(jj).D, 1)
						error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_temp{jj}, number_controls_temp{jj});
					end
					if number_controls_temp{jj} ~= size(systems(jj).D, 2)
						error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_temp{jj}, number_controls_temp{jj});
					end
					D = systems(jj).D;
				else
					D = zeros(size(systems(jj).C, 1), size(systems(jj).B, 2));
				end
				if isfield(systems(jj), 'C_dot')
					number_measurements_xdot_temp{jj} = size(systems(jj).C_dot, 1);
					if number_states_temp{jj} ~= size(systems(jj).C_dot, 2)
						error('control:design:gamma:dimension', 'C_dot must be xX%d.', number_states_temp{jj});
					end
					derivative_feedback{jj} = true;
					C_dot = systems(jj).C_dot;
				else
					number_measurements_xdot_temp{jj} = 0;
					derivative_feedback{jj} = false;
					C_dot = zeros(0, number_states_temp{jj});
				end
				if options.usereferences && isfield(systems(jj), 'C_ref')
					number_references_temp{jj} = size(systems(jj).C_ref, 1);
					if isempty(systems(jj).C_ref)
						C_ref = zeros(0, number_states_temp{jj});
					else
						if number_states_temp{jj} ~= size(systems(jj).C_ref, 2)
							error('control:design:gamma:dimension', 'C_ref must be xX%d.', number_states_temp{jj});
						end
						C_ref = systems(jj).C_ref;
					end
				else
					number_references_temp{jj} = 0;
					C_ref = zeros(0, number_states_temp{jj});
				end
				if options.usereferences && isfield(systems(jj), 'D_ref')
					if isempty(systems(jj).D_ref)
						D_ref = zeros(number_references_temp{jj}, number_controls_temp{jj});
					else
						if number_references_temp{jj} ~= size(systems(jj).D_ref, 1)
							error('control:design:gamma:dimension', 'D_ref must be %dX%d.', number_references_temp{jj}, number_controls_temp{jj});
						end
						if number_controls_temp{jj} ~= size(systems(jj).D_ref, 2)
							error('control:design:gamma:dimension', 'D_ref must be %dX%d.', number_references_temp{jj}, number_controls_temp{jj});
						end
						D_ref = systems(jj).D_ref;
					end
				else
					D_ref = zeros(number_references_temp{jj}, number_controls_temp{jj});
				end
				if isfield(systems(jj), 'T')
					if isempty(systems(jj).T)
						sample_times_temp{jj} = -1;
					else
						if ~isscalar(systems(jj).T) || imag(systems(jj).T) ~= 0
							error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
						end
						sample_times_temp{jj} = systems(jj).T;
					end
				elseif isfield(systems(jj), 't')
					if isempty(systems(jj).t)
						sample_times_temp{jj} = -1;
					else
						if ~isscalar(systems(jj).t) || imag(systems(jj).t) ~= 0
							error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
						end
						sample_times_temp{jj} = systems(jj).t;
					end
				elseif isfield(systems(jj), 'Ts')
					if isempty(systems(jj).Ts)
						sample_times_temp{jj} = -1;
					else
						if ~isscalar(systems(jj).Ts) || imag(systems(jj).Ts) ~= 0
							error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
						end
						sample_times_temp{jj} = systems(jj).Ts;
					end
				elseif isfield(systems(jj), 'T_s')
					if isempty(systems(jj).T_s)
						sample_times_temp{jj} = -1;
					else
						if ~isscalar(systems(jj).T_s) || imag(systems(jj).T_s) ~= 0
							error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
						end
						sample_times_temp{jj} = systems(jj).T_s;
					end
				else
					sample_times_temp{jj} = -1;
				end
				expanded_temp{jj} = 1;
				system{jj, 1} = struct(...
					'E',		E,...
					'A',		systems(jj).A,...
					'B',		systems(jj).B,...
					'C',		systems(jj).C,...
					'C_dot',	C_dot,...
					'D',		D,...
					'C_ref',	C_ref,...
					'D_ref',	D_ref...
				);
			else
				error('control:design:gamma:input', 'System must must have fields A, B and C.');
			end
		elseif iscell(systems) && isstruct(systems{jj})
			[
				system{jj},...
				~,...
				number_states_temp{jj},...
				number_controls_temp{jj},...
				number_measurements_temp{jj},...
				number_measurements_xdot_temp{jj},...
				number_references_temp{jj},...
				descriptor_temp{jj},...
				number_descriptors_temp{jj},...
				derivative_feedback{jj},...
				sample_times_temp{jj},...
				expand...
			] = checkandtransformsystems(systems{jj}, options);
			expanded_temp{jj} = sum(expand(:));
		elseif (iscell(systems) && isa(systems{jj}, 'ss')) || isss
			if iscell(systems)
				if ndims(systems{jj}) > 2 %#ok<ISMAT> check dimension of model array
					sys = reshape(systems{jj}, [], 1);
				else
					sys = systems{jj};
				end
			else
				if ndims(systems) > 2 %#ok<ISMAT> check dimension of model array
					sys = reshape(systems, [], 1);
				else
					sys = systems;
				end
			end
			n_models = size(sys, 3);
			expanded_temp{jj} = n_models;
			system_loc = repmat(struct(...
				'E',		{},...
				'A',		{},...
				'B',		{},...
				'C',		{},...
				'C_dot',	{},...
				'D',		{},...
				'C_ref',	{},...
				'D_ref',	{}...
			), n_models, 1);
			number_states_loc = zeros(n_models, 1);
			number_descriptors_loc = zeros(n_models, 1);
			number_controls_loc = zeros(n_models, 1);
			number_measurements_loc = zeros(n_models, 1);
			number_measurements_xdot_loc = zeros(n_models, 1);
			derivative_feedback_loc = false(n_models, 1);
			descriptor_loc = false(n_models, 1);
			sample_times_loc = -ones(n_models, 1);
			for ii = 1:n_models
				s = sys(:, :, ii);
				if isempty(s.e)
					[systemsA, systemsB, systemsC, systemsD, Ts] = ssdata(s);
					systemsE = eye(size(systemsA, 1));
					descriptor_loc(ii) = false;
				else
					[systemsA, systemsB, systemsC, systemsD, systemsE, Ts] = dssdata(s);
					if ~isdiag(systemsE) || ~all(diag(systemsE) == 1)
						descriptor_loc(ii) = true;
					else
						descriptor_loc(ii) = false;
					end
				end
				number_states_loc(ii) = size(systemsA, 1);
				if number_states_loc(ii) ~= size(systemsA, 2)
					error('control:design:gamma:dimension', 'A must be square.');
				end
				number_descriptors_loc(ii) = size(systemsE, 1);
				if number_descriptors_loc(ii) ~= size(systemsE, 2)
					error('control:design:gamma:dimension', 'E must be square.');
				end
				if number_descriptors_loc(ii) ~= number_states_loc(ii)
					error('control:design:gamma:dimension', 'E must be %dX%d.', number_states_loc(ii), number_states_loc(ii));
				end
				number_controls_loc(ii) = size(systemsB, 2);
				if number_states_loc(ii) ~= size(systemsB, 1)
					error('control:design:gamma:dimension', 'B must be %dXx.', number_states_loc(ii));
				end
				number_measurements_loc(ii) = size(systemsC, 1);
				if options.usemeasurements_xdot
					systemsC_dot = systemsC;
					number_measurements_xdot_loc(ii) = size(systemsC, 1);
				else
					systemsC_dot = zeros(0, number_states_loc(ii));
					number_measurements_xdot_loc(ii) = 0;
				end
				derivative_feedback_loc(ii) = true;
				if number_states_loc(ii) ~= size(systemsC, 2)
					error('control:design:gamma:dimension', 'C must be xX%d.', number_states_loc(ii));
				end
				if number_measurements_loc(ii) ~= size(systemsD, 1)
					error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_loc(ii), number_controls_loc(ii));
				end
				if number_controls_loc(ii) ~= size(systemsD, 2)
					error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_loc(ii), number_controls_loc(ii));
				end
				if options.usereferences
					C_ref = systemsC;
					D_ref = systemsD;
				else
					C_ref = zeros(0, size(systemsA, 2));
					D_ref = zeros(0, size(systemsB, 2));
				end
				if ~isscalar(Ts) || imag(Ts) ~= 0
					error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
				else
					if Ts == 0
						sample_times_loc(ii) = -1;
					else
						sample_times_loc(ii) = Ts;
					end
				end
				system_loc(ii, 1) = struct(...
					'E',		systemsE,...
					'A',		systemsA,...
					'B',		systemsB,...
					'C',		systemsC,...
					'C_dot',	systemsC_dot,...
					'D',		systemsD,...
					'C_ref',	C_ref,...
					'D_ref',	D_ref...
				);
			end
			system{jj} = system_loc;
			number_states_temp{jj} = number_states_loc;
			number_descriptors_temp{jj} = number_descriptors_loc;
			number_controls_temp{jj} = number_controls_loc;
			number_measurements_temp{jj} = number_measurements_loc;
			number_measurements_xdot_temp{jj} = number_measurements_xdot_loc;
			if options.usereferences
				number_references_temp{jj} = number_measurements_loc;
			else
				number_references_temp{jj} = 0*number_measurements_loc;
			end
			derivative_feedback{jj} = derivative_feedback_loc;
			descriptor_temp{jj} = descriptor_loc;
			sample_times_temp{jj} = sample_times_loc;
		elseif (iscell(systems) && (isa(systems{jj}, 'uss') || isa(systems{jj}, 'genss'))) || (isuss || isgenss)
			if iscell(systems)
				if ndims(systems{jj}) > 2 %#ok<ISMAT> check dimension of model array
					sys = reshape(systems{jj}, [], 1);
				else
					sys = systems{jj};
				end
			else
				if ndims(systems) > 2 %#ok<ISMAT> check dimension of model array
					sys = reshape(systems, [], 1);
				else
					sys = systems;
				end
			end
			[
				system{jj},...
				~,...
				number_states_temp{jj},...
				number_controls_temp{jj},...
				number_measurements_temp{jj},...
				number_measurements_xdot_temp{jj},...
				number_references_temp{jj},...
				descriptor_temp{jj},...
				number_descriptors_temp{jj},...
				derivative_feedback{jj},...
				sample_times_temp{jj}...
			] = checkandtransformsystems_uss(sys, options);
			expanded_temp{jj} = size(system{jj}, 1);
		elseif (iscell(systems) && isa(systems{jj}, 'tf')) || istf
			if iscell(systems)
				if ndims(systems{jj}) > 2 %#ok<ISMAT> check dimension of model array
					sys = reshape(systems{jj}, [], 1);
				else
					sys = systems{jj};
				end
			else
				if ndims(systems) > 2 %#ok<ISMAT> check dimension of model array
					sys = reshape(systems, [], 1);
				else
					sys = systems;
				end
			end
			n_models = size(sys, 3);
			expanded_temp{jj} = n_models;
			system_loc = repmat(struct(...
				'E',		{},...
				'A',		{},...
				'B',		{},...
				'C',		{},...
				'C_dot',	{},...
				'D',		{},...
				'C_ref',	{},...
				'D_ref',	{}...
			), n_models, 1);
			number_states_loc = zeros(n_models, 1);
			number_descriptors_loc = zeros(n_models, 1);
			number_controls_loc = zeros(n_models, 1);
			number_measurements_loc = zeros(n_models, 1);
			number_measurements_xdot_loc = zeros(n_models, 1);
			derivative_feedback_loc = false(n_models, 1);
			descriptor_loc = false(n_models, 1);
			sample_times_loc = -ones(n_models, 1);
			[systemsAall, systemsBall, systemsCall, systemsDall, Ts] = ssdata(sys, 'cell');
			for ii = 1:n_models
				systemsA = systemsAall{ii};
				systemsB = systemsBall{ii};
				systemsC = systemsCall{ii};
				systemsD = systemsDall{ii};
				descriptor_loc(ii) = false;
				number_states_loc(ii) = size(systemsA, 1);
				number_descriptors_loc(ii) = number_states_loc(ii);
				if number_states_loc(ii) ~= size(systemsA, 2)
					error('control:design:gamma:dimension', 'A must be square.');
				end
				number_controls_loc(ii) = size(systemsB, 2);
				if number_states_loc(ii) ~= size(systemsB, 1)
					error('control:design:gamma:dimension', 'B must be %dXx.', number_states_loc(ii));
				end
				number_measurements_loc(ii) = size(systemsC, 1);
				if options.usemeasurements_xdot
					systemsC_dot = systemsC;
					number_measurements_xdot_loc(ii) = size(systemsC, 1);
				else
					systemsC_dot = zeros(0, number_states_loc(ii));
					number_measurements_xdot_loc(ii) = 0;
				end
				derivative_feedback_loc(ii) = true;
				if number_states_loc(ii) ~= size(systemsC, 2)
					error('control:design:gamma:dimension', 'C must be xX%d.', number_states_loc(ii));
				end
				if number_measurements_loc(ii) ~= size(systemsD, 1)
					error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_loc(ii), number_controls_loc(ii));
				end
				if number_controls_loc(ii) ~= size(systemsD, 2)
					error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_loc(ii), number_controls_loc(ii));
				end
				if options.usereferences
					C_ref = systemsC;
					D_ref = systemsD;
				else
					C_ref = zeros(0, size(systemsA, 2));
					D_ref = zeros(0, size(systemsB, 2));
				end
				if isempty(Ts)
					sample_times_loc(ii) = -1;
				else
					if ~isscalar(Ts) || imag(Ts) ~= 0
						error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
					end
					if Ts == 0
						sample_times_loc(ii) = -1;
					else
						sample_times_loc(ii) = Ts;
					end
				end
				system_loc(ii, 1) = struct(...
					'E',		eye(size(systemsA, 1)),...
					'A',		systemsA,...
					'B',		systemsB,...
					'C',		systemsC,...
					'C_dot',	systemsC_dot,...
					'D',		systemsD,...
					'C_ref',	C_ref,...
					'D_ref',	D_ref...
				);
			end
			system{jj} = system_loc;
			number_states_temp{jj} = number_states_loc;
			number_descriptors_temp{jj} = number_descriptors_loc;
			number_controls_temp{jj} = number_controls_loc;
			number_measurements_temp{jj} = number_measurements_loc;
			number_measurements_xdot_temp{jj} = number_measurements_xdot_loc;
			if options.usereferences
				number_references_temp{jj} = number_measurements_loc;
			else
				number_references_temp{jj} = 0*number_measurements_loc;
			end
			derivative_feedback{jj} = derivative_feedback_loc;
			descriptor_temp{jj} = descriptor_loc;
			sample_times_temp{jj} = sample_times_loc;
		elseif iscell(systems) && iscell(systems{jj})
			[
				system{jj},...
				~,...
				number_states_temp{jj},...
				number_controls_temp{jj},...
				number_measurements_temp{jj},...
				number_measurements_xdot_temp{jj},...
				number_references_temp{jj},...
				descriptor_temp{jj},...
				number_descriptors_temp{jj},...
				derivative_feedback{jj},...
				sample_times_temp{jj},...
				expand...
			] = checkandtransformsystems(systems{jj}, options);
			expanded_temp{jj} = sum(expand(:));
		else
			error('control:design:gamma:input', 'System must be some kind of control system, not a %s.', class(systems{jj}));
		end
	end
	system = cat(1, system{:});
	number_states_temp = cat(1, number_states_temp{:});
	number_descriptors_temp = cat(1, number_descriptors_temp{:});
	number_controls_temp = cat(1, number_controls_temp{:});
	number_measurements_temp = cat(1, number_measurements_temp{:});
	number_measurements_xdot_temp = cat(1, number_measurements_xdot_temp{:});
	number_references_temp = cat(1, number_references_temp{:});
	derivative_feedback = cat(1, derivative_feedback{:});
	descriptor_temp = cat(1, descriptor_temp{:});
	sample_times_temp = cat(1, sample_times_temp{:});
	if any(number_controls_temp(1) ~= number_controls_temp)
		error('control:design:gamma:dimension', 'All systems must have %d controls.', number_controls_temp(1));
	end
	if any(number_measurements_temp(1) ~= number_measurements_temp)
		error('control:design:gamma:dimension', 'All systems must have %d measurements.', number_measurements_temp(1));
	end
	if any(number_measurements_xdot_temp(1) ~= number_measurements_xdot_temp)
		error('control:design:gamma:dimension', 'All systems must have %d derivative measurements.', number_measurements_xdot_temp(1));
	end
	if any(number_references_temp(1) ~= number_references_temp)
		error('control:design:gamma:dimension', 'All systems must have %d reference inputs.', number_references_temp(1));
	end
	%if any(descriptor_temp(1) ~= descriptor_temp)
	%	error('control:design:gamma:dimension', 'All systems must be descriptor systems or no system must be a descriptor system.');
	%end
	if any(sample_times_temp(1) ~= sample_times_temp)
		if sample_times_temp(1) > 0
			error('control:design:gamma:dimension', 'All systems must have the same sample time ''%f''.', sample_times_temp(1));
		else
			error('control:design:gamma:dimension', 'All systems must have no sample time.');
		end
	end
	number_states = max(number_states_temp);
	number_states_all = number_states_temp;
	number_controls = number_controls_temp(1);
	number_measurements = number_measurements_temp(1);
	number_measurements_xdot = number_measurements_xdot_temp(1);
	number_references = number_references_temp(1);
	descriptor = any(descriptor_temp);
	number_descriptors_all = number_descriptors_temp;
	sample_time = sample_times_temp(1);
	expanded = cat(1, expanded_temp{:});
end