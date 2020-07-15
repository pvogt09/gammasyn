function [success, fileinfo] = c_mex_common(file, makePfad, buildfile, overwrite, nobuild)
	%C_MEX_COMMON compile constraint function for gamma pole placement with different number of output arguments
	%	Input:
	%		file:		file to compile
	%		makePfad:	path to working directory
	%		buildfile:	file with code for invocation of codegeneration
	%		overwrite:	indicator, if mex file should be overwritten
	%		nobuild:	indicator, if compilation should not be done and only the file to compile should be returned
	%	Output:
	%		success:	true, if compilation was successful, else false
	%		fileinfo:	information about the compiled function
	if nargin < 4
		overwrite = true;
	end
	if nargin <= 4
		nobuild = false;
	end
	if nargout >= 2
		fileinfo = struct(...
			'm',		[file, '_m.m'],...
			'mex',		[file, '_mex.', mexext],...
			'build',	[buildfile, '.m']...
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
	max_number_of_objective_functions = settings.max_number_of_objective_functions;
	max_number_of_area_functions = settings.max_number_of_area_functions;
	%#ok<*NASGU> unused variables are function arguments
	system = struct(...
		'E',		coder.typeof(1, [max_system_states, max_system_states],			[true, true]),...
		'A',		coder.typeof(1, [max_system_states, max_system_states],			[true, true]),...
		'B',		coder.typeof(1, [max_system_controls, max_system_states],		[true, true]),...
		'C',		coder.typeof(1, [max_system_measurements, max_system_states],	[true, true]),...
		'C_dot',	coder.typeof(1, [max_system_measurements, max_system_states],	[true, true]),...
		'D',		coder.typeof(1, [max_system_measurements, max_system_controls],	[true, true]),...
		'C_ref',	coder.typeof(1, [max_system_measurements, max_system_states],	[true, true]),...
		'D_ref',	coder.typeof(1, [max_system_measurements, max_system_controls],	[true, true])...
	);
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

	Tx = coder.typeof(1, [inf, 1], [true, false]);
	Tsystem = coder.typeof(system, [max_number_of_systems, 1], [true, false]);
	Tweight = coder.typeof(1, [max_number_of_systems, max_number_of_area_functions], [true, true]);
	Tareafun = coder.newtype('GammaArea', [max_number_of_systems, max_number_of_area_functions], [true, true]);
	Tdimensions = coder.typeof(dimensions);
	Toptions = coder.typeof(struct(...
		'usecompiled',			coder.typeof(true),...
		'numthreads',			coder.typeof(uint32(1)),...
		'type',					coder.newtype('GammaJType', [max_number_of_objective_functions, 1], [true, false]),...
		'weight',				coder.typeof(1, [max_number_of_objective_functions, 1], [true, false]),...
		'allowvarorder',		coder.typeof(true),...
		'eigenvaluederivative',	coder.newtype('GammaEigenvalueDerivativeType', [1, 1], [false, false]),...
		'eigenvaluefilter',		coder.newtype('GammaEigenvalueFilterType', [Inf, 1], [true, false]),...
		'eigenvalueignoreinf',	coder.typeof(true),...
		'objective',			struct(...
			'preventNaN',		coder.typeof(true),...
			'kreisselmeier',	struct(...
				'rho',				coder.typeof(1),...
				'max',				coder.typeof(1)...
			),...
			'lyapunov',		struct(...
				'Q',			coder.typeof(1, [max_system_states, max_system_states, max_number_of_systems], [true, true, true])...
			),...
			'normgain',		struct(...
				'R',			coder.typeof(1, [max_system_controls, max_system_measurements], [true, true]),...
				'R_shift',		coder.typeof(1, [max_system_controls, max_system_measurements], [true, true]),...
				'K',			coder.typeof(1, [max_system_controls, max_system_measurements], [true, true]),...
				'K_shift',		coder.typeof(1, [max_system_controls, max_system_measurements], [true, true]),...
				'F',			coder.typeof(1, [max_system_controls, max_system_measurements], [true, true]),...
				'F_shift',		coder.typeof(1, [max_system_controls, max_system_measurements], [true, true])...
			)...
		)...
	));

	config = compile.constant();

	Args = '{Tx Tsystem Tweight Tareafun Tdimensions Toptions}';
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