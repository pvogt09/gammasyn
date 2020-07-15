function [success, fileinfo] = calculate_areas_fixed(overwrite, nobuild)
	%CALCULATE_EIGENVALUES compile eigenvalue calulation helper function for gamma pole placement
	%	Input:
	%		overwrite:	indicator, if mex file should be overwritten
	%		nobuild:	indicator, if compilation should not be done and only the file to compile should be returned
	%	Output:
	%		success:	true, if compilation was successful, else false
	%		fileinfo:	information about the compiled function
	if nargin < 1
		overwrite = true;
	end
	if nargin <= 1
		nobuild = false;
	end
	file		= realpath(fullfile(compile.destpath(), '+control', '+design', '+gamma', 'private', 'calculate_areas_fixed'));
	makePfad	= realpath(fullfile(compile.buildpath(), 'calculate_areas_fixed'));
	if nargout >= 2
		fileinfo = struct(...
			'm',		[file, '_m.m'],...
			'mex',		[file, '_mex.', mexext],...
			'build',	[mfilename('fullpath'), '.m']...
		);
	end
	if nobuild
		success = true;
		return;
	end
	settings = compile.control.design.gamma.constant();
	max_system_states = settings.max_system_states;
	max_system_controls = settings.max_system_controls;
	max_system_measurements = settings.max_system_measurements;
	max_number_of_systems = settings.max_number_of_systems;
	max_number_of_area_functions = settings.max_number_of_area_functions;

	%#ok<*NASGU> unused variables are function arguments
	areaparameters = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
	areaparametersizes = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPESIZE;
	areaparametersdatatypes = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPE;
	areaparameters(1).type = coder.newtype('GammaArea', 1, false);
	areaparameters.reshift = coder.typeof(1);
	areaparameters.imshift = coder.typeof(1);
	names = fieldnames(areaparameters);
	for ii = 1:size(names, 1)
		if ~any(strcmpi(names{ii}, {'type', 'reshift', 'imshift'}))
			complex = isnumeric(areaparametersizes.(names{ii})) && ~isreal(areaparametersizes.(names{ii}));
			sz = real(areaparametersizes.(names{ii}));
			varsize = false(size(sz));
			if any(isinf(sz))
				varsize(isinf(sz)) = true;
				sz(isinf(sz)) = settings.max_number_of_area_function_parameters;
			end
			if complex
				areaparameters.(names{ii}) = coder.typeof(ones(1, 1, 'like', areaparameters.(names{ii})) + 1i, sz, varsize);
			else
				if ~isnumeric(areaparametersdatatypes.(names{ii}))
					areaparameters.(names{ii}) = coder.typeof(areaparametersdatatypes.(names{ii}), sz, varsize);
				else
					areaparameters.(names{ii}) = coder.typeof(areaparameters.(names{ii}), sz, varsize);
				end
			end
		end
	end
	dimensions = dimensionarg(settings, areaparameters);

	Tareafun = coder.newtype('GammaArea', [max_number_of_systems, max_number_of_area_functions], [true, true]);
	Tweight = coder.typeof(1, [max_number_of_systems, max_number_of_area_functions], [true, true]);
	Teigenvalues = coder.typeof(1 + 1i, [max_system_states, max_number_of_systems], [true, true]);
	Tdimensions = coder.typeof(dimensions);
	Tnumthreads = coder.typeof(uint32(1));
	Teigenvalueignoreinf = coder.typeof(true);

	config = compile.constant();

	Args = '{Tareafun Tweight Teigenvalues Tdimensions Tnumthreads Teigenvalueignoreinf}';
	try
		if overwrite
			codegen([file, '_m.m'], '-args', Args, '-d', makePfad, '-config', 'config', '-o', [file, '_mex']);
		end
		success = true;
		update_hascompiled();
	catch e
		if ~configuration.matlab.hascodertoolbox()
			warning(e.identifier, 'No license for Matlab Coder Toolbox available.');
		else
			if strcmpi('', e.message)
				warning(e.identifier, 'Error in code generation.');
			else
				warning(e.identifier, e.message);
			end
		end
		success = false;
	end
	if nargout < 1
		clear success;
	end
end