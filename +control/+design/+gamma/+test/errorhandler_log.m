function [k_opt, J_opt, output] = errorhandler_log(err, n)
	%ERRORHANDLER_LOG write errors to log file during test runs
	%	Input:
	%		err:	MException that occured during gammasyn run
	%		n:		number of optimization variables
	%	Output:
	%		k_opt:	optimal value
	%		J_opt:	optimal function value
	%		output:	information structure
	SAVEPATH = realpath(fullfile(mfilename('fullpath'), '..'));

	k_opt = ones(n, 1);
	J_opt = 1;
	output = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
	output.message = err.message;
	output.information.output = err.getReport('extended', 'hyperlinks', 'off');
	warning(err.identifier, err.message);

	nl = sprintf('\n');
	tab = sprintf('\t');
	htmlpattern = '<[^>]*>';
	string = sprintf('Fehlermeldung: %s\n', strrep(regexprep(err.getReport, htmlpattern, ''), nl, [nl, tab, tab]));
	fid = fopen(fullfile(SAVEPATH, 'ErrorLog.log'), 'a', 'n', 'UTF-8');
	if fid >= 0
		fprintf(fid, '%s', string);
		fclose(fid);
	end
end