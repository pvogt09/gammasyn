function [system, number_states, number_states_all, number_controls, number_measurements, number_measurements_xdot, number_references, descriptor, number_descriptors_all, derivative_feedback, sample_time] = checkandtransformsystems_uss(sys, systemoptions)
	%CHECKANDTRANSFORMSYSTEMS_USS convert uss and genss objects to multiple models
	%	Input:
	%		sys:						system to convert to multiple models
	%		systemoptions:				structure with options for multiple model creation
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
		'usemeasurements_xdot',	true...
	);
	if isfield(systemoptions, 'usereferences')
		if ~islogical(systemoptions.usereferences) || ~isscalar(systemoptions.usereferences)
			error('control:design:gamma:input', 'Indicator for usage of reference outputs must be a logical scalar.');
		end
		options.usereferences = systemoptions.usereferences;
	end
	if isfield(systemoptions, 'usemeasurements_xdot')
		if ~islogical(systemoptions.usemeasurements_xdot) || ~isscalar(systemoptions.usemeasurements_xdot)
			error('control:design:gamma:input', 'Indicator for usage of derivative measurements outputs must be a logical scalar.');
		end
		options.usemeasurements_xdot = systemoptions.usemeasurements_xdot;
	end
	if isfield(systemoptions, 'samples')
		if ~isstruct(systemoptions.samples)
			error('control:design:gamma:input', 'Definition of number of sampling points for parameters must be a structure.');
		end
		names = fieldnames(systemoptions.samples);
		if ~isempty(names)
			samples = cell(size(names, 1), 2);
			samples(:, 1) = names;
			for ii = 1:size(names, 1)
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
				samples{ii, 2} = number;
			end
		else
			samples = cell(0, 2);
		end
	elseif isfield(systemoptions, 'Blocks')
		if ~isstruct(systemoptions.Blocks)
			error('control:design:gamma:input', 'Definition of number of sampling points for parameters must be a structure.');
		end
		names = fieldnames(systemoptions.Blocks);
		if ~isempty(names)
			samples = cell(size(names, 1), 2);
			samples(:, 1) = names;
			for ii = 1:size(names, 1)
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
				samples{ii, 2} = number;
			end
		else
			samples = cell(0, 2);
		end
	else
		samples = cell(0, 2);
	end
	if isa(sys, 'uss')
		uncertainty = sys.Uncertainty;
		fnames = fieldnames(uncertainty);
		unames = cell(size(fnames, 1), 1);
		neededcombinations = cell(size(unames, 1), 1);
		parfor ii = 1:size(unames, 1)
			combinations = []; %#ok<NASGU> prevent parfor message for uninitialized variables
			param = uncertainty.(fnames{ii, 1});
			number_samples = NaN;
			if ~isempty(samples)
				paramnames = samples;
				match = strcmp(fnames{ii, 1}, paramnames(:, 1));
				if any(match(:))
					if sum(match(:)) == 1
						number_samples = paramnames{match, 2};
					else
						error('control:design:gamma:input', 'Multiple uncertain parameters with the same name defined.');
					end
				end
			end
			if isa(param, 'ureal')
				unames{ii, 1} = param.Name;
				urange = param.Range;
				unom = param.NominalValue;
				if any(isinf(urange(:)))
					error('control:design:gamma:input', 'Uncertain parameter ''%s'' has infinite range and does not result in a valid model.', unames{ii, 1});
				end
				if isnan(number_samples) || number_samples == 3
					combinations = [urange(1, 1), unom, urange(1, 2)];
				else
					if number_samples == 1
						combinations = unom;
					else
						combinations = linspace(urange(1, 1), urange(1, 2), max([2, number_samples]));
						if number_samples >= 3 && unom >= urange(1, 1) && unom <= urange(1, 2)
							[~, idx] = min(abs(combinations - unom));
							combinations(1, idx) = unom;
						end
					end
				end
			else
				error('control:design:gamma:input', 'Uncertainty must be of type ''ureal'', not a %s.', class(param));
			end
			neededcombinations{ii, 1} = combinations;
		end
		grids = cell(size(neededcombinations, 1), 1);
		[grids{:}] = ndgrid(neededcombinations{:});
		grids = cellfun(@(x) x(:), grids, 'UniformOutput', false);
		args = [unames, grids].';
		systems = usubs(sys, args{:}, '-batch');
		systems = reshape(systems, [], 1);
		number_models = size(systems, 3);
		system = repmat(struct(...
			'E',		{},...
			'A',		{},...
			'B',		{},...
			'C',		{},...
			'C_dot',	{},...
			'D',		{},...
			'C_ref',	{},...
			'D_ref',	{}...
		), number_models, 1);
		number_states_temp = zeros(number_models, 1);
		number_descriptors_temp = zeros(number_models, 1);
		number_controls_temp = zeros(number_models, 1);
		number_measurements_temp = zeros(number_models, 1);
		number_measurements_xdot_temp = zeros(number_models, 1);
		number_references_temp = zeros(number_models, 1);
		derivative_feedback_temp = false(number_models, 1);
		descriptor_temp = false(number_models, 1);
		sample_times_temp = -ones(number_models, 1);
		parfor ii = 1:size(systems, 3)
			if isempty(systems(:, :, ii).e)
				[systemsA, systemsB, systemsC, systemsD, Ts] = ssdata(systems(:, :, ii));
				systemsE = eye(size(systemsA, 1));
			else
				[systemsA, systemsB, systemsC, systemsD, systemsE, Ts] = dssdata(systems(:, :, ii));
				if ~isdiag(systemsE) || ~all(diag(systemsE) == 1)
					descriptor_temp(ii) = true;
				end
			end
			number_states_temp(ii) = size(systemsA, 1);
			if number_states_temp(ii) ~= size(systemsA, 2)
				error('control:design:gamma:dimension', 'A must be square.');
			end
			number_descriptors_temp(ii) = size(systemsE, 1);
			if number_descriptors_temp(ii) ~= size(systemsE, 2)
				error('control:design:gamma:dimension', 'E must be square.');
			end
			if number_descriptors_temp(ii) ~= number_states_temp(ii)
				error('control:design:gamma:dimension', 'E must be %dX%d.', number_states_temp(ii), number_states_temp(ii));
			end
			number_controls_temp(ii) = size(systemsB, 2);
			if number_states_temp(ii) ~= size(systemsB, 1)
				error('control:design:gamma:dimension', 'B must be %dXx.', number_states_temp(ii));
			end
			number_measurements_temp(ii) = size(systemsC, 1);
			if options.usemeasurements_xdot
				systemsC_dot = systemsC;
				number_measurements_xdot_temp(ii) = size(systemsC, 1);
			else
				systemsC_dot = zeros(0, number_states_temp(ii));
				number_measurements_xdot_temp(ii) = 0;
			end
			number_references_temp(ii) = size(systemsC, 1);
			derivative_feedback_temp(ii) = true;
			if number_states_temp(ii) ~= size(systemsC, 2)
				error('control:design:gamma:dimension', 'C must be xX%d.', number_states_temp(ii));
			end
			if number_measurements_temp(ii) ~= size(systemsD, 1)
				error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_temp(ii), number_controls_temp(ii));
			end
			if number_controls_temp(ii) ~= size(systemsD, 2)
				error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_temp(ii), number_controls_temp(ii));
			end
			if options.usereferences
				C_ref = systemsC;
				D_ref = systemsD;
			else
				C_ref = zeros(0, size(systemsA, 2));
				D_ref = zeros(0, size(systemsB, 2));
				number_references_temp(ii) = 0;
			end
			if ~isscalar(Ts) || imag(Ts) ~= 0
				error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
			else
				if Ts == 0
					sample_times_temp(ii) = -1;
				else
					sample_times_temp(ii) = Ts;
				end
			end
			system(ii, 1) = struct(...
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
	elseif isa(sys, 'genss')
		uncertainty = sys.Blocks;
		fnames = fieldnames(uncertainty);
		unames = cell(size(fnames, 1), 1);
		neededcombinations = cell(size(unames, 1), 1);
		isfree = true(size(fnames, 1), 1);
		parfor ii = 1:size(unames, 1)
			param = uncertainty.(fnames{ii, 1});
			unames{ii, 1} = param.Name;
			urange = zeros(1, 2); %#ok<PREALL> prevent parfor message for uninitialized variables
			unom = 0; %#ok<NASGU> prevent parfor message for uninitialized variables
			combinations = []; %#ok<NASGU> prevent parfor message for uninitialized variables
			number_samples = NaN;
			if ~isempty(samples)
				paramnames = samples;
				match = strcmp(fnames{ii, 1}, paramnames(:, 1));
				if any(match(:))
					if sum(match(:)) == 1
						number_samples = paramnames{match, 2};
					else
						error('control:design:gamma:input', 'Multiple uncertain parameters with the same name defined.');
					end
				end
			end
			if isa(param, 'realp')
				unom = param.Value;
				isfree(ii, 1) = param.Free;
				if isnan(number_samples) || number_samples == 1
					combinations = unom;
				else
					urange = [param.Minimum, param.Maximum];
					if ~isfree(ii, 1) && any(isinf(urange(:)))
						error('control:design:gamma:input', 'Uncertain parameter ''%s'' is not free, but has infinite range and does not result in a valid model.', unames{ii, 1});
					end
					combinations = linspace(urange(1, 1), urange(1, 2), max([2, number_samples]));
					if number_samples >= 3 && unom >= urange(1, 1) && unom <= urange(1, 2)
						[~, idx] = min(abs(combinations - unom));
						combinations(1, idx) = unom;
					end
					isfree(ii, 1) = false;
				end
			elseif isa(param, 'ureal')
				urange = param.Range;
				unom = param.NominalValue;
				isfree(ii, 1) = false;
				if any(isinf(urange(:)))
					error('control:design:gamma:input', 'Uncertain parameter ''%s'' has infinite range and does not result in a valid model.', unames{ii, 1});
				end
				if isnan(number_samples) || number_samples == 3
					combinations = [urange(1, 1), unom, urange(1, 2)];
				else
					if number_samples == 1
						combinations = unom;
					else
						combinations = linspace(urange(1, 1), urange(1, 2), max([2, number_samples]));
						if number_samples >= 3 && unom >= urange(1, 1) && unom <= urange(1, 2)
							[~, idx] = min(abs(combinations - unom));
							combinations(1, idx) = unom;
						end
					end
				end
			else
				error('control:design:gamma:input', 'Uncertainty must be of type ''ureal'' or ''realp'', not a %s.', class(param));
			end
			neededcombinations{ii, 1} = combinations;
		end
		if any(isfree(:))
			error('control:design:gamma:input', 'Uncertain system has free parameters and can not be converted to multiple models.');
		end
		neededcombinations(isfree, :) = [];
		unames(isfree, :) = [];
		grids = cell(size(neededcombinations, 1), 1);
		[grids{:}] = ndgrid(neededcombinations{:});
		grids = cellfun(@(x) x(:), grids, 'UniformOutput', false);
		args = [unames, grids].';
		systems = usubs(sys, args{:}, '-batch');
		systems = reshape(systems, [], 1);
		number_models = size(systems, 3);
		system = repmat(struct(...
			'E',		{},...
			'A',		{},...
			'B',		{},...
			'C',		{},...
			'C_dot',	{},...
			'D',		{},...
			'C_ref',	{},...
			'D_ref',	{}...
		), number_models, 1);
		number_states_temp = zeros(number_models, 1);
		number_descriptors_temp = zeros(number_models, 1);
		number_controls_temp = zeros(number_models, 1);
		number_measurements_temp = zeros(number_models, 1);
		number_measurements_xdot_temp = zeros(number_models, 1);
		number_references_temp = zeros(number_models, 1);
		derivative_feedback_temp = false(number_models, 1);
		descriptor_temp = false(number_models, 1);
		sample_times_temp = -ones(number_models, 1);
		parfor ii = 1:size(systems, 3)
			s = ss(systems(:, :, ii));% TODO: see TODO above
			if isempty(s.e)
				[systemsA, systemsB, systemsC, systemsD, Ts] = ssdata(s);
				systemsE = eye(size(systemsA, 1));
			else
				[systemsA, systemsB, systemsC, systemsD, systemsE, Ts] = dssdata(s);
				if ~isdiag(systemsE) || ~all(diag(systemsE) == 1)
					descriptor_temp(ii) = true;
				end
			end
			number_states_temp(ii) = size(systemsA, 1);
			if number_states_temp(ii) ~= size(systemsA, 2)
				error('control:design:gamma:dimension', 'A must be square.');
			end
			number_descriptors_temp(ii) = size(systemsE, 1);
			if number_descriptors_temp(ii) ~= size(systemsE, 2)
				error('control:design:gamma:dimension', 'E must be square.');
			end
			if number_descriptors_temp(ii) ~= number_states_temp(ii)
				error('control:design:gamma:dimension', 'E must be %dX%d.', number_states_temp(ii), number_states_temp(ii));
			end
			number_controls_temp(ii) = size(systemsB, 2);
			if number_states_temp(ii) ~= size(systemsB, 1)
				error('control:design:gamma:dimension', 'B must be %dXx.', number_states_temp(ii));
			end
			number_measurements_temp(ii) = size(systemsC, 1);
			if options.usemeasurements_xdot
				systemsC_dot = systemsC;
				number_measurements_xdot_temp(ii) = size(systemsC, 1);
			else
				systemsC_dot = zeros(0, number_states_temp(ii));
				number_measurements_xdot_temp(ii) = 0;
			end
			number_references_temp(ii) = size(systemsC, 1);
			derivative_feedback_temp(ii) = true;
			if number_states_temp(ii) ~= size(systemsC, 2)
				error('control:design:gamma:dimension', 'C must be xX%d.', number_states_temp(ii));
			end
			if number_measurements_temp(ii) ~= size(systemsD, 1)
				error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_temp(ii), number_controls_temp(ii));
			end
			if number_controls_temp(ii) ~= size(systemsD, 2)
				error('control:design:gamma:dimension', 'D must be %dX%d.', number_measurements_temp(ii), number_controls_temp(ii));
			end
			if options.usereferences
				C_ref = systemsC;
				D_ref = systemsD;
			else
				C_ref = zeros(0, size(systemsA, 2));
				D_ref = zeros(0, size(systemsB, 2));
				number_references_temp(ii) = 0;
			end
			if ~isscalar(Ts) || imag(Ts) ~= 0
				error('control:design:gamma:dimension', 'Sample Time must be a real scalar.');
			else
				if Ts == 0
					sample_times_temp(ii) = -1;
				else
					sample_times_temp(ii) = Ts;
				end
			end
			system(ii, 1) = struct(...
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
	else
		error('control:design:gamma:input', 'System must be some kind of control system, not a %s.', class(sys));
	end
	number_states = max(number_states_temp);
	number_states_all = number_states_temp;
	number_controls = number_controls_temp(1);
	number_measurements = number_measurements_temp(1);
	number_measurements_xdot = number_measurements_xdot_temp(1);
	number_references = number_references_temp(1);
	descriptor = all(descriptor_temp);
	number_descriptors_all = number_descriptors_temp;
	derivative_feedback = derivative_feedback_temp;
	sample_time = sample_times_temp(1);
end