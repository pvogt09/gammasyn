function [system_cl, Jopt, information] = gammasyn_looptune(systems, areafun, weights, solveroptions, objectiveoptions)
	%GAMMASYN_LOOPTUNE robust pole placement for multi-models in genss representation with tunable parameters (e.g. realp or ltiblock)
	%	Input:
	%		systems:			structure/cell array or matrix with dynamic systems to take into consideration
	%		areafun:			area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		weights:			weighting matrix with number of systems columns and number of pole area border functions rows
	%		solveroptions:		options for optimization algorithm to use
	%		objectiveoptions:	options for problem functions
	%	Output:
	%		system_cl:			closed loop with optimal tunable parameters
	%		Jopt:				optimal objective function value for optimal gain
	%		information:		srtucture with information about optimization runs
	if nargin < 2
		error('control:design:gamma:input', 'Systems, poleareas and weights must be supplied.');
	end
	if ~isa(systems, 'genss') && ~isa(systems, 'uss')
		error('control:design:gamma:input', 'System must be of type ''genss'' or ''uss''.');
	end
	if size(systems, 1) < 1 || size(systems, 2) < 1
		error('control:design:gamma:input', 'System must have at least one input and one output.');
	end
	szsystem = size(systems);
	if size(szsystem, 2) >= 3 && any(szsystem(3:end) > 1)
		error('control:design:gamma:input', 'System arrays are not supported.');
	end
	% TODO: make dimensions of areafun and weights compatible with generated multiple models
	% get LFT representation with uncertain and tunable elements and sort inputs and outputs
	[H, B, S] = getLFTModel(systems);

	isrealpblock = cellfun(@(x) isa(x, 'realp'), B, 'UniformOutput', true);
	isrealpfree = cellfun(@(x) isa(x, 'realp') && x.Free, B, 'UniformOutput', true);
	isltiblock = model.isltiblock(B);
	isuncertainblock = cellfun(@(x) isuncertain(x), B, 'UniformOutput', true);
	%ispidstd = cellfun(@(x) isa(x, 'pidstd'), B, 'UniformOutput', true);% TODO: is pidstd tunable?
	idxrealp = find(isrealpblock & isrealpfree);
	idxuncertain = find(isuncertainblock | (~isrealpfree & ~isltiblock));
	idxltiblock = find(isltiblock);
	if isempty(idxrealp) && isempty(idxltiblock)
		error('control:design:gamma', 'System must have at least one tunable component or ''gammasyn'' has to be used instead of ''%s''.', mfilename());
	end
	idx = [
		idxuncertain;
		idxrealp;
		idxltiblock
	];
	number_controls_B = zeros(size(B, 1), 1);
	number_measurements_B = zeros(size(B, 1), 1);
	for ii = 1:size(B, 1)
		number_controls_B(ii, 1) = size(B{ii, 1}, 2);
		number_measurements_B(ii, 1) = size(B{ii, 1}, 1);
	end
	S_cell = cell(size(B, 1), 1);
	idxu_S = cumsum(number_controls_B);
	idxy_S = cumsum(number_measurements_B);
	for ii = 1:size(B, 1)
		if ii == 1
			idxstartu = 0;
			idxstarty = 0;
		else
			idxstartu = idxu_S(ii - 1, 1);
			idxstarty = idxy_S(ii - 1, 1);
		end
		S_cell{ii, 1} = S(idxstarty + 1:idxy_S(ii, 1), idxstartu + 1:idxu_S(ii, 1));
	end
	number_controls_total = size(H, 2) - size(B, 1);%size(systems, 2);
	number_measurements_total = size(H, 1) - size(B, 1);%size(systems, 1);
	H_sorted = H([
		(1:number_measurements_total)';
		idx + number_measurements_total
	], [
		(1:number_controls_total)';
		idx + number_controls_total
	]);
	number_controls = size(H, 2) - sum(number_measurements_B(:));%size(systems, 2);
	number_measurements = size(H, 1) - sum(number_controls_B(:));%size(systems, 1);
	B_sorted_uncertain = B(idxuncertain, 1);
	B_sorted_free = B([
		idxrealp;
		idxltiblock
	], 1);
	S_sorted_uncertain = S_cell(idxuncertain, 1);
	S_sorted_free = S_cell([
		idxrealp;
		idxltiblock
	], 1);

	number_controls_uncertain = zeros(size(B_sorted_uncertain, 1), 1);
	number_measurements_uncertain = zeros(size(B_sorted_uncertain, 1), 1);
	for ii = 1:size(B_sorted_uncertain, 1)
		number_controls_uncertain(ii, 1) = size(B_sorted_uncertain{ii, 1}, 2);
		number_measurements_uncertain(ii, 1) = size(B_sorted_uncertain{ii, 1}, 1);
	end

	% convert all types of tunable system representations to tunable gains and tunable state space realizations
	B_sorted_ss = cell(size(B_sorted_free, 1), 1);
	B_sorted_type = cell(size(B_sorted_free, 1), 1);
	B_sorted_names = cell(size(B_sorted_free, 1), 1);
	S_sorted_augmented = cell(size(B_sorted_free, 1), 2);
	intss = cell(size(B_sorted_free, 1), 1);
	K = cell(size(B_sorted_free, 1), 1);
	number_controls_ltiblock = zeros(size(B_sorted_free, 1), 1);
	number_states_ltiblock = zeros(size(B_sorted_free, 1), 1);
	number_measurements_ltiblock = zeros(size(B_sorted_free, 1), 1);
	idxu = cell(size(B_sorted_free, 1), 4);
	for ii = 1:size(B_sorted_free, 1) %#ok<FORPF> idxu spans multiple iterations
		if isa(B_sorted_free{ii, 1}, 'realp')
			B_sorted_type{ii, 1} = 'realp';
			B_sorted_names{ii, 1} = B_sorted_free{ii, 1}.Name;
			number_controls_ltiblock(ii, 1) = size(B_sorted_free{ii, 1}.Value, 2);
			number_states_ltiblock(ii, 1) = 0;
			number_measurements_ltiblock(ii, 1) = size(B_sorted_free{ii, 1}.Value, 1);
			gain = ltiblock.gain(sprintf('K%d', ii), number_measurements_ltiblock(ii, 1) + number_states_ltiblock(ii, 1), number_controls_ltiblock(ii, 1) + number_states_ltiblock(ii, 1));
			gain.Gain.Value = B_sorted_free{ii, 1}.Value;
			gain.Gain.Minimum = B_sorted_free{ii, 1}.Minimum;
			gain.Gain.Maximum = B_sorted_free{ii, 1}.Maximum;
			gain.Gain.Free = B_sorted_free{ii, 1}.Free;
			gain.Gain.Scale = ones(number_measurements_ltiblock(ii, 1), number_controls_ltiblock(ii, 1));
			B_sorted_ss{ii, 1} = ltiblock.ss(B_sorted_free{ii, 1}.Name, number_states_ltiblock(ii, 1), number_measurements_ltiblock(ii, 1), number_controls_ltiblock(ii, 1), systems.Ts);
			B_sorted_ss{ii, 1}.d.Value = B_sorted_free{ii, 1}.Value;
			B_sorted_ss{ii, 1}.d.Minimum = B_sorted_free{ii, 1}.Minimum;
			B_sorted_ss{ii, 1}.d.Maximum = B_sorted_free{ii, 1}.Maximum;
			B_sorted_ss{ii, 1}.d.Free = B_sorted_free{ii, 1}.Free;
			B_sorted_ss{ii, 1}.d.Scale = ones(number_measurements_ltiblock(ii, 1), number_controls_ltiblock(ii, 1));
		else
			B_sorted_type{ii, 1} = class(B_sorted_free{ii, 1});
			B_sorted_names{ii, 1} = B_sorted_free{ii, 1}.Name;
			B_sorted_ss{ii, 1} = model.ltiblock2ss(B_sorted_free{ii, 1});
			if systems.Ts > 0 && B_sorted_ss{ii, 1}.Ts ~= systems.Ts
				error('control:design:gamma', 'Sampling time must be equal for all tunable blocks and systems.');
			end
			number_controls_ltiblock(ii, 1) = size(B_sorted_ss{ii, 1}.b.Value, 2);
			number_states_ltiblock(ii, 1) = size(B_sorted_ss{ii, 1}.a.Value, 1);
			number_measurements_ltiblock(ii, 1) = size(B_sorted_ss{ii, 1}.c.Value, 1);
			if B_sorted_ss{ii, 1}.Ts > 0
				A = eye(number_states_ltiblock(ii, 1));
			else
				A = zeros(number_states_ltiblock(ii, 1), number_states_ltiblock(ii, 1));
			end
			intss{ii, 1} = ss(A, eye(number_states_ltiblock(ii, 1)), eye(number_states_ltiblock(ii, 1)), zeros(number_states_ltiblock(ii, 1), number_states_ltiblock(ii, 1)), B_sorted_ss{ii, 1}.Ts);
			gain = ltiblock.gain(sprintf('K%d', ii), number_measurements_ltiblock(ii, 1) + number_states_ltiblock(ii, 1), number_controls_ltiblock(ii, 1) + number_states_ltiblock(ii, 1));
			gain.Gain.Value = [
				B_sorted_ss{ii, 1}.d.Value,	B_sorted_ss{ii, 1}.c.Value;
				B_sorted_ss{ii, 1}.b.Value,	B_sorted_ss{ii, 1}.a.Value
			];
			gain.Gain.Minimum = [
				B_sorted_ss{ii, 1}.d.Minimum,	B_sorted_ss{ii, 1}.c.Minimum;
				B_sorted_ss{ii, 1}.b.Minimum,	B_sorted_ss{ii, 1}.a.Minimum
			];
			gain.Gain.Maximum = [
				B_sorted_ss{ii, 1}.d.Maximum,	B_sorted_ss{ii, 1}.c.Maximum;
				B_sorted_ss{ii, 1}.b.Maximum,	B_sorted_ss{ii, 1}.a.Maximum
			];
			gain.Gain.Free = [
				B_sorted_ss{ii, 1}.d.Free,	B_sorted_ss{ii, 1}.c.Free;
				B_sorted_ss{ii, 1}.b.Free,	B_sorted_ss{ii, 1}.a.Free
			];
			gain.Gain.Scale = [
				B_sorted_ss{ii, 1}.d.Scale,	B_sorted_ss{ii, 1}.c.Scale;
				B_sorted_ss{ii, 1}.b.Scale,	B_sorted_ss{ii, 1}.a.Scale
			];
		end
		K{ii, 1} = gain;
		S_sorted_augmented(ii, :) = {S_sorted_free{ii, 1},	zeros(number_states_ltiblock(ii, 1), number_states_ltiblock(ii, 1))};
		if ii == 1
			start_u = 0;
		else
			if isempty(idxu{ii - 1, 1})
				start_u = 0;
			else
				start_u = idxu{ii - 1, 1}(end);
			end
		end
		if ii == 1
			start_x = 0;
		else
			if isempty(idxu{ii - 1, 2})
				start_x = 0;
			else
				start_x = idxu{ii - 1, 2}(end);
			end
		end
		if ii == 1
			start_y = 0;
		else
			if isempty(idxu{ii - 1, 3})
				start_y = 0;
			else
				start_y = idxu{ii - 1, 3}(end);
			end
		end
		idxu(ii, :) = {
			((start_u + 1):(start_u + number_controls_ltiblock(ii, 1)))',	((start_x + 1):(start_x + number_states_ltiblock(ii, 1)))',	((start_y + 1):(start_y + number_measurements_ltiblock(ii, 1)))',	((start_x + 1):(start_x + number_states_ltiblock(ii, 1)))'
		};
	end

	% augment certain system with integrators for tunable state space elements and sort inputs and outputs accordingly to get generalized uncertain plant
	H_augmented = blkdiag(H_sorted, intss{:});
	shift_u = number_controls + sum(number_measurements_uncertain(:));
	shift_y = number_measurements + sum(number_controls_uncertain(:));
	shift_x = shift_y + sum(number_controls_ltiblock(:));
	shift_xdot = shift_u + sum(number_measurements_ltiblock(:));
	for ii = 1:size(idxu, 1)
		idxu(ii, :) = {idxu{ii, 1} + shift_y,	idxu{ii, 2} + shift_x,	idxu{ii, 3} + shift_u,	idxu{ii, 4} + shift_xdot};
	end
	idxy = [
		number_measurements + (1:sum(number_controls_uncertain(:)))';% uncertain outputs
		(1:number_measurements)';% regular system outputs
		cell2mat(reshape(idxu(:, 1:2)', [], 1));% controller inputs
	];
	idxu = [
		number_controls + (1:sum(number_measurements_uncertain(:)))';% uncertain inputs
		(1:number_controls)';% regular system inputs
		cell2mat(reshape(idxu(:, 3:4)', [], 1))% controller outputs
	];
	S_sorted_augmented = reshape(S_sorted_augmented', [], 1);
	S_augmented = blkdiag(S_sorted_augmented{:});
	H_augmented_sorted = H_augmented(idxy, idxu);

	% check for blocks that occur multiple times
	[uniquenames, uniquenameidx, uniquenameidxinv] = unique(B_sorted_names, 'stable');
	needsduplicatehandling = length(uniquenames) ~= length(B_sorted_names);
	% convert R to gammasyn arguments
	R_0 = cell(size(K, 1), 1);
	R_fixed = cell(size(K, 1), 1);
	K_minimum = cell(size(K, 1), 1);
	K_maximum = cell(size(K, 1), 1);
	blocksize = zeros(size(K, 1), 2);
	isduplicate = false(size(K, 1), 1);
	for ii = 1:size(K, 1)
		R_0{ii, 1} = K{ii, 1}.gain.Value;
		free = K{ii, 1}.gain.Free;
		R_fixed{ii, 1} = double(free);
		R_fixed{ii, 1}(~free) = K{ii, 1}.gain.Value(~free);
		R_fixed{ii, 1}(free) = NaN;
		K_minimum{ii, 1} = K{ii, 1}.gain.Minimum;
		K_maximum{ii, 1} = K{ii, 1}.gain.Maximum;
		blocksize(ii, :) = [
			size(K{ii, 1}.gain.Value, 1),	size(K{ii, 1}.gain.Value, 2)
		];
		isduplicate(ii, 1) = needsduplicatehandling && uniquenameidx(uniquenameidxinv(ii, 1), 1) ~= ii;
	end
	% gammasyn uses A - BRC while lft uses A + BRC
	R_0 = -(blkdiag(R_0{:}) - S_augmented);
	R_fixed_num = zeros(size(R_0));
	blockidx = cumsum(blocksize, 1);
	for ii = 1:size(K, 1)
		if ii == 1
			idxstart = [1, 1];
		else
			idxstart = blockidx(ii - 1, :) + 1;
		end
		R_fixed_num(idxstart(1, 1):blockidx(ii, 1), idxstart(1, 2):blockidx(ii, 2)) = R_fixed{ii, 1};
	end
	K_minimum_blk = blkdiag(K_minimum{:});
	K_maximum_blk = blkdiag(K_maximum{:});
	K_minimum_blk(~isnan(R_fixed_num)) = -Inf;
	K_maximum_blk(~isnan(R_fixed_num)) = Inf;
	R_fixed = -(R_fixed_num - S_augmented);
	if any(isduplicate(:))
		% convert fixed elements to equation system to honor duplicate parameters with additional equality constraints
		number_equations = sum(~isnan(R_fixed_num(:)));
		R_fixed_equation_system_A_fixed = NaN(size(R_fixed_num, 1), size(R_fixed_num, 2), number_equations);
		R_fixed_equation_system_b_fixed = NaN(number_equations, 1);
		equation_number = 1;
		for ii = 1:size(R_fixed_num, 1) %#ok<FORPF> no parfor because of dependent iterations
			for jj = 1:size(R_fixed_num, 2)
				if ~isnan(R_fixed_num(ii, jj))
					mat = zeros(size(R_fixed_num, 1), size(R_fixed_num, 2));
					mat(ii, jj) = 1;
					R_fixed_equation_system_b_fixed(equation_number, 1) = R_fixed_num(ii, jj);
					R_fixed_equation_system_A_fixed(:, :, equation_number) = mat;
					equation_number = equation_number + 1;
				end
			end
		end
		dd = [
			[0,	0];% TODO: check
			diff(blockidx, 1)
		];
		number_equations = sum(prod(dd(isduplicate, :), 2));
		R_fixed_equation_system_A_duplicate = NaN(size(R_fixed_num, 1), size(R_fixed_num, 2), number_equations);
		R_fixed_equation_system_b_duplicate = NaN(number_equations, 1);
		equation_number = 1;
		for ii = 1:size(K, 1) %#ok<FORPF> no parfor because of dependent iterations
			if isduplicate(ii, 1)
				if ii == 1
					idxstart = [1, 1];
				else
					idxstart = blockidx(ii - 1, :) + 1;
				end
				ref = uniquenameidx(uniquenameidxinv(ii, 1));
				if ref == 1
					idxstart_ref = [1, 1];
				else
					idxstart_ref = blockidx(ref - 1, :) + 1;
				end
				idx_optim_start = idxstart(1, 1):blockidx(ii, 1);
				idx_optim_end = idxstart(1, 2):blockidx(ii, 2);
				idx_ref_start = idxstart_ref(1, 1):blockidx(ref, 1);
				idx_ref_end = idxstart_ref(1, 2):blockidx(ref, 2);
				for jj = 1:size(idx_optim_start, 2)
					for kk = 1:size(idx_optim_end, 2)
						mat = zeros(size(R_fixed_num, 1), size(R_fixed_num, 2));
						mat(idx_optim_start(1, jj), idx_optim_end(1, kk)) = 1;
						mat(idx_ref_start(1, jj), idx_ref_end(1, kk)) = -1;
						R_fixed_equation_system_b_duplicate(equation_number, 1) = 0;
						R_fixed_equation_system_A_duplicate(:, :, equation_number) = mat;
						equation_number = equation_number + 1;
					end
				end
			end
		end
		R_fixed = {
			cat(3, R_fixed_equation_system_A_fixed, R_fixed_equation_system_A_duplicate),	[
				R_fixed_equation_system_b_fixed;
				R_fixed_equation_system_b_duplicate
			]
		};
	else
		R_fixed = {~isnan(R_fixed), R_fixed};
	end
	K_minimum = -(K_maximum_blk - S_augmented);
	K_maximum = -(K_minimum_blk - S_augmented);
	if any(any(K_minimum > K_maximum))
		error('control:design:gamma', 'Minimum gain must not be larger than maximum gain.');
	end
	% genss gets converted to system with derivative feedback and prefilter, so K and F are needed and fixed to be zero
	K_0 = zeros(size(R_0, 1), size(R_0, 2));
	K_fixed = zeros(size(R_0, 1), size(R_0, 2));
	K_minimum = -Inf(size(R_0, 1), size(R_0, 2));
	K_maximum = Inf(size(R_0, 1), size(R_0, 2));
	F_0 = zeros(size(R_0, 1), size(R_0, 2));
	F_fixed = zeros(size(R_0, 1), size(R_0, 2));
	F_minimum = -Inf(size(R_0, 1), size(R_0, 2));
	F_maximum = Inf(size(R_0, 1), size(R_0, 2));
	R_0 = {R_0, K_0, F_0};
	R_fixed = {
		R_fixed;
		{true(size(K_fixed)), K_fixed};
		{true(size(F_fixed)), F_fixed}
	};
	R_bounds = {
		{K_minimum, K_maximum};
		{K_minimum, K_maximum};
		{F_minimum, F_maximum}
	};
	system_uncertain = lft(blkdiag(B_sorted_uncertain{:}) - blkdiag(S_sorted_uncertain{:}), H_augmented_sorted, size(idxuncertain, 1), size(idxuncertain, 1));
	system_uncertain_gamma = system_uncertain(number_measurements + 1:end, number_controls + 1:end);

	% call gammasyn
	if nargin >= 5
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(system_uncertain_gamma, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds);
	elseif nargin >= 4
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(system_uncertain_gamma, areafun, weights, R_fixed, R_0, solveroptions, [], R_bounds);
	elseif nargin >= 3
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(system_uncertain_gamma, areafun, weights, R_fixed, R_0, [], objectiveoptions, R_bounds);
	else
		error('control:design:gamma', 'Systems, poleareas and weights must be supplied.');
	end
	% substitue solution into tunable blocks
	Ropt = -Ropt{1} + S_augmented;
	R_opt_block = cell(size(K, 1), 1);
	hasNaN = false(size(K, 1), 1);
	for ii = 1:size(K, 1)
		if ii == 1
			idxstart = [1, 1];
		else
			idxstart = blockidx(ii - 1, :) + 1;
		end
		R_opt_block{ii, 1} = Ropt(idxstart(1, 1):blockidx(ii, 1), idxstart(1, 2):blockidx(ii, 2));
		hasNaN(ii, 1) = any(isnan(R_opt_block{ii, 1}(:)));
	end
	B_sorted_ss_notNaN = B_sorted_ss;
	B_sorted_solution = cell(size(K, 1), 1);
	B_sorted_solution_notNaN = cell(size(K, 1), 1);
	B_sorted_ss_names = cell(size(K, 1), 1);
	for ii = 1:size(K, 1)
		B_sorted_ss_names{ii, 1} = B_sorted_ss{ii, 1}.Name;
		B_sorted_ss{ii, 1}.d.Value = R_opt_block{ii, 1}(1:number_measurements_ltiblock(ii, 1), 1:number_controls_ltiblock(ii, 1));
		B_sorted_ss{ii, 1}.c.Value = R_opt_block{ii, 1}(1:number_measurements_ltiblock(ii, 1), number_controls_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)));
		B_sorted_ss{ii, 1}.b.Value = R_opt_block{ii, 1}(number_measurements_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)), 1:number_controls_ltiblock(ii, 1));
		B_sorted_ss{ii, 1}.a.Value = R_opt_block{ii, 1}(number_measurements_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)), number_controls_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)));
		if strcmpi(B_sorted_type{ii, 1}, 'realp')
			B_sorted_solution{ii, 1} = realp(B_sorted_ss{ii, 1}.Name, R_opt_block{ii, 1});
		else
			B_sorted_solution{ii, 1} = model.ltiblockss2ltiblock(B_sorted_ss{ii, 1}, B_sorted_type{ii, 1});
		end
		if hasNaN(ii, 1)
			% prevent substitution of NaN into values because that crashes lft below while substituting afterwards works
			R_opt_block_notNaN = R_opt_block{ii, 1};
			R_opt_block_notNaN(isnan(R_opt_block_notNaN)) = 0;
			B_sorted_ss_notNaN{ii, 1}.d.Value = R_opt_block_notNaN(1:number_measurements_ltiblock(ii, 1), 1:number_controls_ltiblock(ii, 1));
			B_sorted_ss_notNaN{ii, 1}.c.Value = R_opt_block_notNaN(1:number_measurements_ltiblock(ii, 1), number_controls_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)));
			B_sorted_ss_notNaN{ii, 1}.b.Value = R_opt_block_notNaN(number_measurements_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)), 1:number_controls_ltiblock(ii, 1));
			B_sorted_ss_notNaN{ii, 1}.a.Value = R_opt_block_notNaN(number_measurements_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)), number_controls_ltiblock(ii, 1) + (1:number_states_ltiblock(ii, 1)));
			if strcmpi(B_sorted_type{ii, 1}, 'realp')
				B_sorted_solution_notNaN{ii, 1} = realp(B_sorted_ss{ii, 1}.Name, R_opt_block_notNaN);
			else
				B_sorted_solution_notNaN{ii, 1} = model.ltiblockss2ltiblock(B_sorted_ss_notNaN{ii, 1}, B_sorted_type{ii, 1});
			end
		else
			B_sorted_solution_notNaN{ii, 1} = B_sorted_solution{ii, 1};
		end
	end
	% restore closed loop system with solution substituted into tunable coefficients
	try
		% HINT: sometimes lft crashes because of same parameters with different values although the parameters are equal
		system_cl = lft(H_sorted, blkdiag(B_sorted_uncertain{:}, B_sorted_solution_notNaN{:}) - blkdiag(S_sorted_uncertain{:}, S_sorted_free{:}));
		if any(hasNaN(:))
			% replace blocks where NaN occured and was replaced by a safe value with NaN again, because this works after the closed loop system is built for non-NaN values
			for ii = 1:size(hasNaN, 1) %#ok<FORPF> no parfor because of dependent assignment in object
				if hasNaN(ii, 1)
					system_cl = model.ltiblock_set_value(system_cl, B_sorted_names{ii, 1}, B_sorted_solution{ii, 1});
				end
			end
		end
	catch e
		if strcmp(e.identifier, 'Control:lftmodel:BlockName2')
			% TODO: would it be better to use model.ltiblock_set_value instead of recreating the closed loop above?
			system_cl = systems;
			for ii = 1:size(hasNaN, 1) %#ok<FORPF> no parfor because of object changes
				system_cl = model.ltiblock_set_value(system_cl, B_sorted_names{ii, 1}, B_sorted_solution{ii, 1});
			end
		else
			rethrow(e);
		end
	end
end