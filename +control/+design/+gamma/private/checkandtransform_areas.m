function [...
	number_areaargs_strict, area_parts_strict, number_areas_max_strict, area_hasgrad_strict, area_hashess_strict, area_parameters_strict, areafun_strict,...
	number_areaargs_loose, area_parts_loose, number_areas_max_loose, area_hasgrad_loose, area_hashess_loose, area_parameters_loose, areafun_loose...
] = checkandtransform_areas(areafun, number_models, number_systems, expanded_models)
	%CHECKANDTRANSFORM_AREAS check areafunctions for validity, retrieve specific information from areafunctions and uniformize representation of arefunctions to support code generation or function handle calling
	%	Input:
	%		areafun:					area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		number_models:				number of systems after model expansion
	%		number_systems:				number of systems before model expansion
	%		expanded_models:			vector with number of expanded models
	%	Output:
	%		number_areaargs_strict:		number of arguments of area border functions (1 (= {sigma + j omega}) or 2 (= {sigma, omega}))
	%		area_parts_strict:			number of areafunction return values for every area border function
	%		number_areas_max_strict:	maximum number of areafunction return values
	%		area_hasgrad_strict:		indicator whether area border function returns gradient information
	%		area_hashess_strict:		indicator whether area border function returns hessian information
	%		area_parameters_strict:		structure with parameters of area border functions for use in generated code with predefined area border functions
	%		areafun_strict:				area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for strict optimization of areafunctions
	%		number_areaargs_loose:		number of arguments of area border functions (1 (= {sigma + j omega}) or 2 (= {sigma, omega}))
	%		area_parts_loose:			number of areafunction return values for every area border function
	%		number_areas_max_loose:		maximum number of areafunction return values
	%		area_hasgrad_loose:			indicator whether area border function returns gradient information
	%		area_hashess_loose:			indicator whether area border function returns hessian information
	%		area_parameters_loose:		structure with parameters of area border functions for use in generated code with predefined area border functions
	%		areafun_loose:				area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for loose optimization of areafunctions
	if ~isnumeric(number_models) || ~isscalar(number_models)
		error('control:design:gamma:dimension', 'Number of models must be a numeric scalar.');
	end
	if ~isnumeric(number_systems) || ~isscalar(number_systems)
		error('control:design:gamma:dimension', 'Number of systems must be a numeric scalar.');
	end
	if ~isnumeric(expanded_models) || size(expanded_models, 2) ~= 1 || size(expanded_models, 1) ~= number_systems
		error('control:design:gamma:dimension', 'Expanded models must have %d elements.', number_systems);
	end
	if ~iscell(areafun)
		if isa(areafun, 'control.design.gamma.area.GammaArea')
			if size(areafun, 1) < number_models
				if size(areafun, 1) == 1
					areafun = repmat(areafun, number_models, 1);
				elseif size(areafun, 1) == number_systems
					areafun_temp = cell(size(areafun, 1), 1);
					for ii = 1:size(areafun_temp)
						areafun_temp{ii, 1} = repmat(areafun(ii, :, :), expanded_models(ii, 1), 1);
					end
					areafun = cat(1, areafun_temp{:});
				else
					if any(expanded_models ~= 1)
						error('control:design:gamma:input', 'For %d systems %d polearea functions have to be supplied, not %d when model expansion is used.', number_systems, number_systems, size(areafun, 1));
					else
						error('control:design:gamma:input', 'For %d systems %d polearea functions have to be supplied, not %d.', number_models, number_models, size(areafun, 1));
					end
				end
			end
			if size(areafun, 3) == 2
				areafun_loose = squeeze(areafun(:, :, 2));
				areafun_strict = squeeze(areafun(:, :, 1));
			else
				areafun_strict = areafun;
				areafun_loose = control.design.gamma.area.GammaArea.empty();
			end
		else
			temp = cell(number_models, 1);
			parfor jj = 1:size(temp, 1);% TODO: check if only scalar function handles are supplied and what other types are allowed in combination with model expansion
				temp(jj) = {areafun};
			end
			areafun_strict = temp;
			areafun_loose = cell(number_models, 1);
		end
	else
		if size(areafun, 2) == 2
			if size(areafun, 1) == 1
				areafun_strict = areafun{:, 1};
				areafun_loose = areafun{:, 2};
			else
				areafun_strict = areafun(:, 1);
				areafun_loose = areafun(:, 2);
			end
		else
			areafun_strict = areafun;
			areafun_loose = cell(number_models, 1);
		end
		if size(areafun_strict, 1) > 1 && size(areafun_strict, 1) < number_models
			if size(areafun_strict, 1) == number_systems
				areafun_temp = cell(size(areafun_strict, 1), 1);
				for ii = 1:size(areafun_temp)
					if iscell(areafun_strict)
						if isfunctionhandle(areafun_strict{ii, :})
							areafun_temp{ii, 1} = repmat({areafun_strict{ii, :}}, expanded_models(ii, 1), 1);
						else
							areafun_temp{ii, 1} = repmat(areafun_strict{ii, :}, expanded_models(ii, 1), 1);
						end
					else
						areafun_temp{ii, 1} = repmat(areafun_strict(ii, :), expanded_models(ii, 1), 1);
					end
				end
				areafun_strict = cat(1, areafun_temp{:});
			else
				if any(expanded_models ~= 1)
					error('control:design:gamma:input', 'For %d systems %d polearea functions have to be supplied, not %d when model expansion is used.', number_systems, number_systems, size(areafun_strict, 1));
				end
			end
		end
		if ~isempty(areafun_loose) && size(areafun_loose, 1) > 1 && size(areafun_loose, 1) < number_models
			if size(areafun_loose, 1) == number_systems
				areafun_temp = cell(size(areafun_loose, 1), 1);
				for ii = 1:size(areafun_temp)
					if iscell(areafun_loose)
						if isfunctionhandle(areafun_loose{ii, :})
							areafun_temp{ii, 1} = repmat({areafun_loose{ii, :}}, expanded_models(ii, 1), 1);
						else
							areafun_temp{ii, 1} = repmat(areafun_loose{ii, :}, expanded_models(ii, 1), 1);
						end
					else
						areafun_temp{ii, 1} = repmat(areafun_loose(ii, :), expanded_models(ii, 1), 1);
					end
				end
				areafun_loose = cat(1, areafun_temp{:});
			else
				if any(expanded_models ~= 1)
					error('control:design:gamma:input', 'For %d systems %d polearea functions have to be supplied, not %d when model expansion is used.', number_systems, number_systems, size(areafun_loose, 1));
				end
			end
		end
	end
	if isfunctionhandle(areafun_strict)
		areafun_strict = repmat({areafun_strict}, number_models, 1);
		isfun = true(size(areafun_strict));
	else
		isfun = false(size(areafun_strict));
	end
	if isa(areafun_strict, 'control.design.gamma.area.GammaArea')
		isCustomGammaArea = reshape([areafun_strict(:).type] == GammaArea.CUSTOM, size(areafun_strict));
	else
		isCustomGammaArea = false(size(areafun_strict));
	end
	if iscell(areafun_strict)
		isGammaArea = cellfun(@(x) isempty(x) || isa(x, 'control.design.gamma.area.GammaArea'), areafun_strict, 'UniformOutput', true);
		isCustomGammaArea = cellfun(@(x) isa(x, 'control.design.gamma.area.Custom'), areafun_strict, 'UniformOutput', true);
		isfun = cellfun(@isfunctionhandle, areafun_strict, 'UniformOutput', true);
		if all(isGammaArea(:))
			allempty = isemptycell(areafun_strict);
			areafun_strict(allempty) = {control.design.gamma.area.None()};
			if isscalar(areafun_strict)
				areafun_strict = [areafun_strict{:}];
			else
				areafun_strict = reshape([areafun_strict{:}], size(areafun_strict));
			end
			isCustomGammaArea = false(size(areafun_strict));
		end
	end
	if isempty(areafun_strict)
		areafun_strict = control.design.gamma.area.GammaArea.empty(1, 0);
	end
	if any(isCustomGammaArea(:)) && ~any(isfun(:))
		areafun_strict = num2cell(areafun_strict);
	end
	if iscell(areafun_strict)
		allempty = isemptycell(areafun_strict);
		if all(allempty(:))
			error('control:design:gamma:input', 'At least one polearea function must be supplied.');
		end
		isfun = cellfun(@isfunctionhandle, areafun_strict, 'UniformOutput', true);
		if size(areafun_strict, 1) == 1
			areafun_strict = repmat(areafun_strict, number_models, 1);
			isfun = repmat(isfun, number_models, 1);
			isCustomGammaArea = repmat(isCustomGammaArea, number_models, 1);
			allempty = repmat(allempty, number_models, 1);
		end
		if number_models ~= size(areafun_strict, 1)
			error('control:design:gamma:input', 'For %d systems %d polearea functions have to be supplied, not %d.', number_models, number_models, size(areafun_strict, 1));
		end
		szareafun_strict = size(areafun_strict);
		areafun_strict = reshape(areafun_strict, [], 1);
		isfunvec = reshape(isfun, [], 1);
		iscustomvec = reshape(isCustomGammaArea, [], 1);
		allempty = reshape(allempty, [], 1);
		number_areaargs_strict = zeros(size(areafun_strict, 1), 1);
		area_hasgrad_temp_strict = false(size(areafun_strict, 1), 1);
		area_hashess_temp_strict = false(size(areafun_strict, 1), 1);
		area_parts_strict = zeros(size(areafun_strict, 1), 1);
		parfor jj = 1:size(areafun_strict, 1)
			if allempty(jj)
				areafun_strict{jj} = control.design.gamma.area.None();
			end
			if ~isfunvec(jj) && ~iscustomvec(jj)
				if isa(areafun_strict{jj}, 'control.design.gamma.area.GammaArea')
					number_areaargs_strict(jj, 1) = 2;
					area_hasgrad_temp_strict(jj, 1) = true;
					area_hashess_temp_strict(jj, 1) = true;
					area_parts_strict(jj, 1) = size(areafun_strict{jj}, 2);
					continue;
				else
					error('control:design:gamma:input', 'Polearea function must be a function handle.');
				end
			elseif iscustomvec(jj)
				areafun_strict{jj} = areafun_strict{jj}.fun;
			end
			number_areaargs_strict(jj, 1) = nargin(areafun_strict{jj});
			area_hasgrad_temp_strict(jj, 1) = nargout(areafun_strict{jj}) >= 2 || nargout(areafun_strict{jj}) <= -1;
			area_hashess_temp_strict(jj, 1) = nargout(areafun_strict{jj}) >= 4 || nargout(areafun_strict{jj}) <= -1;
			if number_areaargs_strict(jj, 1) > 2
				error('control:design:gamma:input', 'Polearea function must have at most 2 input arguments.');
			end
			temp = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			tempgraddelta = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			tempgradomega = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			temphessdeltadelta = NaN;
			temphessdeltaomega = NaN;
			temphessomegadelta = NaN;
			temphessomegaomega = NaN;
			if area_hashess_temp_strict(jj, 1)
				if nargout(areafun_strict{jj}) == -1
					if number_areaargs_strict(jj, 1) == 1
						try
							[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_strict{jj}(0);
						catch ehess
							if strcmpi(ehess.identifier, 'MATLAB:maxlhs') || strcmpi(ehess.identifier, 'MATLAB:TooManyOutputs') || strcmpi(ehess.identifier, 'MATLAB:unassignedOutputs')
								try
									[temp, tempgraddelta, tempgradomega] = areafun_strict{jj}(0);
								catch egrad
									if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
										area_hasgrad_temp_strict(jj, 1) = false;
										area_hashess_temp_strict(jj, 1) = false;
										[temp] = areafun_strict{jj}(0);
										tempgraddelta = NaN(1, size(temp, 1));
										tempgradomega = NaN(1, size(temp, 1));
										temphessdeltadelta = NaN(1, size(temp, 1));
										temphessdeltaomega = NaN(1, size(temp, 1));
										temphessomegadelta = NaN(1, size(temp, 1));
										temphessomegaomega = NaN(1, size(temp, 1));
									else
										rethrow(egrad);
									end
								end
							else
								rethrow(ehess);
							end
						end
					else
						try
							[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_strict{jj}(0, 0);
						catch ehess
							if strcmpi(ehess.identifier, 'MATLAB:maxlhs') || strcmpi(ehess.identifier, 'MATLAB:TooManyOutputs') || strcmpi(ehess.identifier, 'MATLAB:unassignedOutputs')
								try
									[temp, tempgraddelta, tempgradomega] = areafun_strict{jj}(0, 0);
								catch egrad
									if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
										area_hasgrad_temp_strict(jj, 1) = false;
										area_hashess_temp_strict(jj, 1) = false;
										[temp] = areafun_strict{jj}(0, 0);
										tempgraddelta = NaN(1, size(temp, 1));
										tempgradomega = NaN(1, size(temp, 1));
										temphessdeltadelta = NaN(1, size(temp, 1));
										temphessdeltaomega = NaN(1, size(temp, 1));
										temphessomegadelta = NaN(1, size(temp, 1));
										temphessomegaomega = NaN(1, size(temp, 1));
									else
										rethrow(egrad);
									end
								end
							else
								rethrow(ehess);
							end
						end
					end
				else
					if number_areaargs_strict(jj, 1) == 1
						[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_strict{jj}(0);
					else
						[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_strict{jj}(0, 0);
					end
				end
				if ~isrow(tempgraddelta)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if ~isrow(tempgradomega)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if size(temp, 2) ~= size(tempgraddelta, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(tempgradomega, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if ~isrow(temphessdeltadelta)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessdeltaomega)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessomegadelta)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessomegaomega)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if size(temp, 2) ~= size(temphessdeltadelta, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessdeltaomega, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessomegadelta, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessomegaomega, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
			elseif area_hasgrad_temp_strict(jj, 1)
				if nargout(areafun_strict{jj}) == -1
					if number_areaargs_strict(jj, 1) == 1
						try
							[temp, tempgraddelta, tempgradomega] = areafun_strict{jj}(0);
						catch egrad
							if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
								area_hasgrad_temp_strict(jj, 1) = false;
								[temp] = areafun_strict{jj}(0);
								tempgraddelta = NaN(1, size(temp, 1));
								tempgradomega = NaN(1, size(temp, 1));
							else
								rethrow(egrad);
							end
						end
					else
						try
							[temp, tempgraddelta, tempgradomega] = areafun_strict{jj}(0, 0);
						catch egrad
							if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
								area_hasgrad_temp_strict(jj, 1) = false;
								[temp] = areafun_strict{jj}(0, 0);
								tempgraddelta = NaN(1, size(temp, 1));
								tempgradomega = NaN(1, size(temp, 1));
							else
								rethrow(egrad);
							end
						end
					end
				else
					if number_areaargs_strict(jj, 1) == 1
						[temp, tempgraddelta, tempgradomega] = areafun_strict{jj}(0);
					else
						[temp, tempgraddelta, tempgradomega] = areafun_strict{jj}(0, 0);
					end
				end
				if ~isrow(tempgraddelta)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if ~isrow(tempgradomega)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if size(temp, 2) ~= size(tempgraddelta, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(tempgradomega, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				area_hashess_temp_strict(jj, 1) = false;
			else
				if number_areaargs_strict(jj, 1) == 1
					temp = areafun_strict{jj}(0);
				else
					temp = areafun_strict{jj}(0, 0);
				end
				area_hashess_temp_strict(jj, 1) = false;
			end
			if ~isrow(temp)
				error('control:design:gamma:input', 'Polearea function must return row vectors.');
			end
			area_parts_strict(jj, 1) = size(temp, 2);
		end
		areafun_strict = reshape(areafun_strict, szareafun_strict);
		area_hasgrad_strict = all(area_hasgrad_temp_strict(:));
		area_parts_strict = reshape(area_parts_strict, szareafun_strict);
		number_areas_max_strict = max(sum(area_parts_strict, 2));
		area_hashess_strict = all(area_hashess_temp_strict(:));
		area_parameters_strict = repmat(control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY, number_models, number_areas_max_strict);
		number_areaargs_strict = reshape(number_areaargs_strict, szareafun_strict);
		areafun_strict_combined = cell(size(areafun_strict, 1), 1);
		number_areaargs_temp_strict = zeros(size(areafun_strict, 1), 1);
		for jj = 1:size(areafun_strict, 1)
			[areafun_strict_combined{jj, 1}, number_areaargs_temp_strict(jj, 1)] = toareafun_combined(areafun_strict(jj, :), area_hasgrad_strict, area_hashess_strict, area_parts_strict(jj, :), number_areaargs_strict(jj, :));
		end
		number_areaargs_strict = number_areaargs_temp_strict;
		areafun_strict = areafun_strict_combined;
		area_parts_strict = sum(area_parts_strict, 2);
	elseif ~isempty(areafun_strict) && isa(areafun_strict, 'control.design.gamma.area.GammaArea')
		if size(areafun_strict, 1) == 1 && size(areafun_strict, 1) < number_models
			areafun_strict = repmat(areafun_strict, number_models, 1);
		end
		number_areaargs_strict = 2*ones(size(areafun_strict, 1), 1);
		area_parts_strict = size(areafun_strict, 2);
		number_areas_max_strict = size(areafun_strict, 2);
		area_hasgrad_strict = true;
		area_hashess_strict = true;
		area_parameters_strict = areafun_strict.tofunctionstruct();
		areafun_strict = reshape([areafun_strict.type], size(areafun_strict));
	elseif isempty(areafun_strict)
		number_areaargs_strict = 2*ones(size(areafun_strict, 1), 1);
		area_parts_strict = size(areafun_strict, 2);
		number_areas_max_strict = size(areafun_strict, 2);
		area_hasgrad_strict = true;
		area_hashess_strict = true;
		area_parameters_strict = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
		areafun_strict = GammaArea.empty();
	else
		error('control:design:gamma:input', 'Polearea function must be a ''GammaArea'' or a function handle, not a ''%s''.', class(areafun_strict));
	end
	if ~isempty(areafun_loose) && isfunctionhandle(areafun_loose)
		areafun_loose = repmat({areafun_loose}, number_models, 1);
		isfun = true(size(areafun_loose));
	else
		isfun = false(size(areafun_loose));
	end
	if isa(areafun_loose, 'control.design.gamma.area.GammaArea')
		isCustomGammaArea = reshape([areafun_loose(:).type] == GammaArea.CUSTOM, size(areafun_loose));
	else
		isCustomGammaArea = false(size(areafun_loose));
	end
	if ~isempty(areafun_loose) && iscell(areafun_loose)
		isGammaArea = cellfun(@(x) isempty(x) || isa(x, 'control.design.gamma.area.GammaArea'), areafun_loose, 'UniformOutput', true);
		isCustomGammaArea = cellfun(@(x) isa(x, 'control.design.gamma.area.Custom'), areafun_loose, 'UniformOutput', true);
		isfun = cellfun(@isfunctionhandle, areafun_loose, 'UniformOutput', true);
		if all(isGammaArea(:))
			allempty = isemptycell(areafun_loose);
			areafun_loose(allempty) = {control.design.gamma.area.None()};
			if isscalar(areafun_loose)
				areafun_loose = [areafun_loose{:}];
			else
				areafun_loose = reshape([areafun_loose{:}], size(areafun_loose));
			end
			isCustomGammaArea = false(size(areafun_loose));
		end
	end
	if any(isCustomGammaArea(:)) && ~any(isfun(:))
		areafun_loose = num2cell(areafun_loose);
	end
	if ~isempty(areafun_loose) && iscell(areafun_loose)
		isfun = cellfun(@isfunctionhandle, areafun_loose, 'UniformOutput', true);
		if size(areafun_loose, 1) == 1
			areafun_loose = repmat(areafun_loose, number_models, 1);
			isfun = repmat(isfun, number_models, 1);
			isCustomGammaArea = repmat(isCustomGammaArea, number_models, 1);
		end
		if number_models ~= size(areafun_loose, 1)
			error('control:design:gamma:input', 'For %d systems %d polearea functions have to be supplied, not %d.', number_models, number_models, size(areafun_loose, 1));
		end
		szareafun_loose = size(areafun_loose);
		areafun_loose = reshape(areafun_loose, [], 1);
		isfunvec = reshape(isfun, [], 1);
		iscustomvec = reshape(isCustomGammaArea, [], 1);
		number_areaargs_loose = zeros(size(areafun_loose, 1), 1);
		area_hasgrad_temp_loose = false(size(areafun_loose, 1), 1);
		area_hashess_temp_loose = false(size(areafun_loose, 1), 1);
		area_parts_loose = zeros(size(areafun_loose, 1), 1);
		area_empty_loose = false(size(areafun_loose, 1), 1);
		parfor jj = 1:size(areafun_loose, 1)
			area_empty_loose(jj, 1) = isempty(areafun_loose{jj});
			if area_empty_loose(jj, 1)
				continue;
			end
			if ~isfunvec(jj) && ~iscustomvec(jj)
				if isa(areafun_loose{jj}, 'control.design.gamma.area.GammaArea')
					number_areaargs_loose(jj, 1) = 2;
					area_hasgrad_temp_loose(jj, 1) = true;
					area_hashess_temp_loose(jj, 1) = true;
					area_parts_loose(jj, 1) = size(areafun_loose{jj}, 2);
					continue;
				else
					error('control:design:gamma:input', 'Polearea function must be a function handle.');
				end
			elseif iscustomvec(jj)
				areafun_loose{jj} = areafun_loose{jj}.fun;
			end
			number_areaargs_loose(jj, 1) = nargin(areafun_loose{jj});
			area_hasgrad_temp_loose(jj, 1) = nargout(areafun_loose{jj}) >= 2 || nargout(areafun_loose{jj}) <= -1;
			area_hashess_temp_loose(jj, 1) = nargout(areafun_loose{jj}) >= 4 || nargout(areafun_loose{jj}) <= -1;
			if number_areaargs_loose(jj, 1) > 2
				error('control:design:gamma:input', 'Polearea function must have at most 2 input arguments.');
			end
			temp = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			tempgraddelta = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			tempgradomega = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			temphessdeltadelta = NaN;
			temphessdeltaomega = NaN;
			temphessomegadelta = NaN;
			temphessomegaomega = NaN;
			if area_hashess_temp_loose(jj, 1)
				if nargout(areafun_loose{jj}) == -1
					if number_areaargs_loose(jj, 1) == 1
						try
							[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_loose{jj}(0);
						catch ehess
							if strcmpi(ehess.identifier, 'MATLAB:maxlhs') || strcmpi(ehess.identifier, 'MATLAB:TooManyOutputs') || strcmpi(ehess.identifier, 'MATLAB:unassignedOutputs')
								try
									[temp, tempgraddelta, tempgradomega] = areafun_loose{jj}(0);
								catch egrad
									if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
										area_hasgrad_temp_loose(jj, 1) = false;
										area_hashess_temp_loose(jj, 1) = false;
										[temp] = areafun_loose{jj}(0);
										tempgraddelta = NaN(1, size(temp, 1));
										tempgradomega = NaN(1, size(temp, 1));
										temphessdeltadelta = NaN(1, size(temp, 1));
										temphessomegadelta = NaN(1, size(temp, 1));
										temphessdeltaomega = NaN(1, size(temp, 1));
										temphessomegaomega = NaN(1, size(temp, 1));
									else
										rethrow(egrad);
									end
								end
							else
								rethrow(ehess);
							end
						end
					else
						try
							[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_loose{jj}(0, 0);
						catch ehess
							if strcmpi(ehess.identifier, 'MATLAB:maxlhs') || strcmpi(ehess.identifier, 'MATLAB:TooManyOutputs') || strcmpi(ehess.identifier, 'MATLAB:unassignedOutputs')
								try
									[temp, tempgraddelta, tempgradomega] = areafun_loose{jj}(0, 0);
								catch egrad
									if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
										area_hasgrad_temp_loose(jj, 1) = false;
										[temp] = areafun_loose{jj}(0, 0);
										tempgraddelta = NaN(1, size(temp, 1));
										tempgradomega = NaN(1, size(temp, 1));
										temphessdeltadelta = NaN(1, size(temp, 1));
										temphessomegadelta = NaN(1, size(temp, 1));
										temphessdeltaomega = NaN(1, size(temp, 1));
										temphessomegaomega = NaN(1, size(temp, 1));
									else
										rethrow(egrad);
									end
								end
							else
								rethrow(ehess);
							end
						end
					end
				else
					if number_areaargs_loose(jj, 1) == 1
						[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_loose{jj}(0);
					else
						[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = areafun_loose{jj}(0, 0);
					end
				end
				if ~isrow(tempgraddelta)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if ~isrow(tempgradomega)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if size(temp, 2) ~= size(tempgraddelta, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(tempgradomega, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if ~isrow(temphessdeltadelta)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessdeltaomega)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessomegadelta)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessomegaomega)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if size(temp, 2) ~= size(temphessdeltadelta, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessdeltaomega, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessomegadelta, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessomegaomega, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
			elseif area_hasgrad_temp_loose(jj, 1)
				if nargout(areafun_loose{jj}) == -1
					if number_areaargs_loose(jj, 1) == 1
						try
							[temp, tempgraddelta, tempgradomega] = areafun_loose{jj}(0);
						catch egrad
							if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
								area_hasgrad_temp_loose(jj, 1) = false;
								[temp] = areafun_loose{jj}(0);
								tempgraddelta = NaN(1, size(temp, 1));
								tempgradomega = NaN(1, size(temp, 1));
							else
								rethrow(egrad);
							end
						end
					else
						try
							[temp, tempgraddelta, tempgradomega] = areafun_loose{jj}(0, 0);
						catch egrad
							if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
								area_hasgrad_temp_loose(jj, 1) = false;
								[temp] = areafun_loose{jj}(0, 0);
								tempgraddelta = NaN(1, size(temp, 1));
								tempgradomega = NaN(1, size(temp, 1));
							else
								rethrow(egrad);
							end
						end
					end
				else
					if number_areaargs_loose(jj, 1) == 1
						[temp, tempgraddelta, tempgradomega] = areafun_loose{jj}(0);
					else
						[temp, tempgraddelta, tempgradomega] = areafun_loose{jj}(0, 0);
					end
				end
				if ~isrow(tempgraddelta)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if ~isrow(tempgradomega)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if size(temp, 2) ~= size(tempgraddelta, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(tempgradomega, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				area_hashess_temp_loose(jj, 1) = false;
			else
				if number_areaargs_loose(jj, 1) == 1
					temp = areafun_loose{jj}(0);
				else
					temp = areafun_loose{jj}(0, 0);
				end
				area_hashess_temp_loose(jj, 1) = false;
			end
			if ~isrow(temp)
				error('control:design:gamma:input', 'Polearea function must return row vectors.');
			end
			area_parts_loose(jj, 1) = size(temp, 2);
		end
		if all(area_empty_loose)
			areafun_loose = GammaArea.empty();
			number_areaargs_loose = 2*ones(size(areafun_loose, 1), 1);
			area_parts_loose = size(areafun_loose, 2);
			number_areas_max_loose = size(areafun_loose, 2);
			area_hasgrad_loose = true;
			area_hashess_loose = true;
			area_parameters_loose = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
		else
			areafun_loose = reshape(areafun_loose, szareafun_loose);
			area_hasgrad_loose = all(area_hasgrad_temp_loose(:));
			area_parts_loose = reshape(area_parts_loose, szareafun_loose);
			number_areaargs_loose = reshape(number_areaargs_loose, szareafun_loose);
			number_areas_max_loose = max(sum(area_parts_loose, 2));
			area_hashess_loose = all(area_hashess_temp_loose(:));
			area_parameters_loose = repmat(control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY, number_models, number_areas_max_loose);
			areafun_loose_combined = cell(size(areafun_loose, 1), 1);
			number_areaargs_temp_loose = zeros(size(areafun_loose, 1), 1);
			for jj = 1:size(areafun_loose, 1)
				[areafun_loose_combined{jj, 1}, number_areaargs_temp_loose(jj, 1)] = toareafun_combined(areafun_loose(jj, :), area_hasgrad_loose, area_hashess_loose, area_parts_loose(jj, :), number_areaargs_loose(jj, :));
			end
			areafun_loose = areafun_loose_combined;
			area_parts_loose = sum(area_parts_loose, 2);
			number_areaargs_loose = number_areaargs_temp_loose;
		end
	elseif ~isempty(areafun_loose) && isa(areafun_loose, 'control.design.gamma.area.GammaArea')
		if size(areafun_loose, 1) == 1 && size(areafun_loose, 1) < number_models
			areafun_loose = repmat(areafun_loose, number_models, 1);
		end
		number_areaargs_loose = 2*ones(size(areafun_loose, 1), 1);
		area_parts_loose = size(areafun_loose, 2);
		number_areas_max_loose = size(areafun_loose, 2);
		area_hasgrad_loose = true;
		area_hashess_loose = true;
		area_parameters_loose = areafun_loose.tofunctionstruct();
		areafun_loose = reshape([areafun_loose.type], size(areafun_loose));
	elseif isempty(areafun_loose)
		number_areaargs_loose = 2*ones(size(areafun_loose, 1), 1);
		area_parts_loose = size(areafun_loose, 2);
		number_areas_max_loose = size(areafun_loose, 2);
		area_hasgrad_loose = true;
		area_hashess_loose = true;
		area_parameters_loose = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
		areafun_loose = GammaArea.empty();
	else
		error('control:design:gamma:input', 'Polearea function must be a ''GammaArea'' or a function handle, not a ''%s''.', class(areafun_loose));
	end
end

function [fun] = toareafun(areafun)
	%TOAREAFUN convert GammaArea to function handle
	%	Input:
	%		areafun:	GammaArea to convert to function handle
	%	Output:
	%		fun:		function handle
	parameter = areafun.tofunctionstruct();
	function [area, dareadre, dareadim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = f(re, im)
		area = zeros(1, size(areafun, 2));
		if nargout >= 2
			dareadre = zeros(1, size(areafun, 2));
		end
		if nargout >= 3
			dareadim = zeros(1, size(areafun, 2));
		end
		if nargout >= 4
			d2fdredre = zeros(1, size(areafun, 2));
		end
		if nargout >= 5
			d2fdimdre = zeros(1, size(areafun, 2));
		end
		if nargout >= 6
			d2fdredim = zeros(1, size(areafun, 2));
		end
		if nargout >= 7
			d2fdimdim = zeros(1, size(areafun, 2));
		end
		for ii = 1:size(areafun, 2) %#ok<FORPF> parfor does not recognize areafun as a function handle
			if nargout >= 7
				[area(ii), dareadre(ii), dareadim(ii), d2fdredre(ii), d2fdimdre(ii), d2fdredim(ii), d2fdimdim(ii)] = areafun(ii).border(re, im, parameter(ii));
			elseif nargout >= 6
				[area(ii), dareadre(ii), dareadim(ii), d2fdredre(ii), d2fdimdre(ii), d2fdredim(ii)] = areafun(ii).border(re, im, parameter(ii));
			elseif nargout >= 5
				[area(ii), dareadre(ii), dareadim(ii), d2fdredre(ii), d2fdimdre(ii)] = areafun(ii).border(re, im, parameter(ii));
			elseif nargout >= 4
				[area(ii), dareadre(ii), dareadim(ii), d2fdredre(ii)] = areafun(ii).border(re, im, parameter(ii));
			elseif nargout >= 3
				[area(ii), dareadre(ii), dareadim(ii)] = areafun(ii).border(re, im, parameter(ii));
			elseif nargout >= 2
				[area(ii), dareadre(ii)] = areafun(ii).border(re, im, parameter(ii));
			else
				area(ii) = areafun(ii).border(re, im, parameter(ii));
			end
		end
	end
	fun = @f;
end

function [fun, number_area_args] = toareafun_combined(areafun, hasgrad, hashess, area_parts, number_area_args)
	%TOAREAFUN_COMBINED convert cell array of GammaArea and function handles to a single function handle
	%	Input:
	%		areafun:			cell array of GammaArea and function hanles to convert to a single function handle
	%		hasgrad:			indicator if gradient information is supplied
	%		hashess:			indicator if hessian information is supplied
	%		area_parts:			number of areas the functions return
	%		number_area_args:	number of arguments for area functions
	%	Output:
	%		fun:				function handle
	%		number_area_args:	number of arguments for area functions
	if ~iscell(areafun)
		error('control:design:gamma:input', 'Polearea function must be of type ''cell''.');
	end
	if size(areafun, 1) ~= 1
		error('control:design:gamma:input', 'Polearea function must be a row vector.');
	end
	if size(areafun, 2) == 1
		if isfunctionhandle(areafun{1})
			fun = areafun{1};
			number_area_args = nargin(areafun{1});
		else
			fun = toareafun(areafun{1});
			%number_area_args = nargin(fun);
		end
		return;
	end
	parameter = cell(1, size(areafun, 2));
	isfun = true(1, size(areafun, 2));
	bordername = cell(1, size(areafun, 2));
	for hh = 1:size(areafun, 2)
		isfun(1, hh) = isfunctionhandle(areafun{1, hh});
		if ~isfun(1, hh)
			parameter{1, hh} = areafun{1, hh}.tofunctionstruct();
			bordername{1, hh} = [class(areafun{1, hh}), '.border'];
		end
	end
	number_areas_max_row = sum(area_parts(:));
	area_parts_idx = cumsum(area_parts);
	number_area_args = 2;
	function [area, dareadre, dareadim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = f(re, im)
		area = zeros(1, number_areas_max_row);
		if nargout >= 2
			dareadre = zeros(1, number_areas_max_row);
		end
		if nargout >= 3
			dareadim = zeros(1, number_areas_max_row);
		end
		if nargout >= 4
			d2fdredre = zeros(1, number_areas_max_row);
		end
		if nargout >= 5
			d2fdimdre = zeros(1, number_areas_max_row);
		end
		if nargout >= 6
			d2fdredim = zeros(1, number_areas_max_row);
		end
		if nargout >= 7
			d2fdimdim = zeros(1, number_areas_max_row);
		end
		for ii = 1:size(areafun, 2) %#ok<FORPF> parfor does not recognize areafun as a function handle
			if ii == 1
				idxstart = 1;
			else
				idxstart = area_parts_idx(ii - 1) + 1;
			end
			idxend = area_parts_idx(ii);
			funhandle = areafun{ii};
			if ~isfun(1, ii)
				if nargout >= 7
					[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend), d2fdredim(idxstart:idxend), d2fdimdim(idxstart:idxend)] = feval(bordername{ii}, re, im, parameter{ii});
				elseif nargout >= 6
					[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend), d2fdredim(idxstart:idxend)] = feval(bordername{ii}, re, im, parameter{ii});
				elseif nargout >= 5
					[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend)] = feval(bordername{ii}, re, im, parameter{ii});
				elseif nargout >= 4
					[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend)] = feval(bordername{ii}, re, im, parameter{ii});
				elseif nargout >= 3
					[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend)] = feval(bordername{ii}, re, im, parameter{ii});
				elseif nargout >= 2
					[area(idxstart:idxend), dareadre(idxstart:idxend)] = feval(bordername{ii}, re, im, parameter{ii});
				else
					area(idxstart:idxend) = feval(bordername{ii}, re, im, parameter{ii});
				end
			else
				if hashess && nargout >= 4
					if nargin(funhandle) == 1
						if nargout >= 7
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend), d2fdredim(idxstart:idxend), d2fdimdim(idxstart:idxend)] = funhandle(re + 1i*im);
						elseif nargout >= 6
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend), d2fdredim(idxstart:idxend)] = funhandle(re + 1i*im);
						elseif nargout >= 5
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend)] = funhandle(re + 1i*im);
						elseif nargout >= 4
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend)] = funhandle(re + 1i*im);
						elseif nargout >= 3
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend)] = funhandle(re + 1i*im);
						elseif nargout >= 2
							[area(idxstart:idxend), dareadre(idxstart:idxend)] = funhandle(re + 1i*im);
						else
							area(idxstart:idxend) = funhandle(re + 1i*im);
						end
					else
						if nargout >= 7
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend), d2fdredim(idxstart:idxend), d2fdimdim(idxstart:idxend)] = funhandle(re, im);
						elseif nargout >= 6
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend), d2fdredim(idxstart:idxend)] = funhandle(re, im);
						elseif nargout >= 5
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend), d2fdimdre(idxstart:idxend)] = funhandle(re, im);
						elseif nargout >= 3
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend), d2fdredre(idxstart:idxend)] = funhandle(re, im);
						elseif nargout >= 3
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend)] = funhandle(re, im);
						elseif nargout >= 2
							[area(idxstart:idxend), dareadre(idxstart:idxend)] = funhandle(re, im);
						else
							area(idxstart:idxend) = funhandle(re, im);
						end
					end
				elseif hasgrad
					if nargout >= 4
						error('control:design:gamma:hessian', 'Hessian for polearea functions was not supplied.');
					end
					if nargin(funhandle) == 1
						if nargout >= 3
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend)] = funhandle(re + 1i*im);
						elseif nargout >= 2
							[area(idxstart:idxend), dareadre(idxstart:idxend)] = funhandle(re + 1i*im);
						else
							area(idxstart:idxend) = funhandle(re + 1i*im);
						end
					else
						if nargout >= 3
							[area(idxstart:idxend), dareadre(idxstart:idxend), dareadim(idxstart:idxend)] = funhandle(re, im);
						elseif nargout >= 2
							[area(idxstart:idxend), dareadre(idxstart:idxend)] = funhandle(re, im);
						else
							area(idxstart:idxend) = funhandle(re, im);
						end
					end
				else
					if nargout >= 2
						error('control:design:gamma:gradient', 'Gradient for polearea functions was not supplied.');
					end
					if nargin(areafun{ii}) == 1
						area(idxstart:idxend) = funhandle(re + 1i*im);
					else
						area(idxstart:idxend) = funhandle(re, im);
					end
				end
			end
		end
	end
	fun = @f;
end