function ok = compiler_supports_eml_openmp(compilerName)
    % 1 5
    % 2 5
    % 3 5
    supported_compiler = not(ismac);
    if strcmp(compilerName, 'lcc64') || strcmp(compilerName, 'mingw64') || strcmp(compilerName, 'msvcsdk')
        % 6 9
        % 7 9
        supported_compiler = false;
    end
	if strcmp(compilerName, 'mingw64')
		supported_compiler = true;
	end
    % 10 14
    % 11 14
    % 12 14
    ok = supported_compiler;
end