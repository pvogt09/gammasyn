function [iter, funevals, time, fval, constrviol, infeas, primalinfeas, dualinfeas, outputstr] = parsefile(file)
	%PARSEFILE parse printfile for optimization run statistics
	%	Input:
	%		file:			file to parse
	%	Output:
	%		iter:			number of iterations
	%		funevals:		number of function evaluations
	%		time:			duration of optimization run
	%		fval:			optimal function value
	%		infeas:			infeasibility measure
	%		primalinfeas:	primal infeasibility measure
	%		dualinfeas:		dual infeasibility measure
	%		outputstr:		optimizer message
	if ~ischar(file)
		error('optimization:solver:snopt:input', 'Printfile must be of type char.');
	end
	if ~exist(file, 'file')
		error('optimization:solver:snopt:input', 'Printfile ''%s'' does not exist.', file);
	end
	fid = fopen(file, 'r');
	if fid ~= -1
		text = fread(fid, '*char')';
		[iterstart, iterend] = regexp(text, 'No. of iterations\s+\d+\s+Objective');
		if ~isempty(iterstart) && ~isempty(iterend)
			iter = str2double(strtrim(strrep(strrep(text(iterstart(end):iterend(end)), 'Objective', ''), 'No. of iterations', '')));
		else
			iter = NaN;
		end
		[funstart, funend] = regexp(text, 'User function calls \(total\)\s+\d+');
		if ~isempty(funstart) && ~isempty(funend)
			funevals = str2double(strtrim(strrep(text(funstart(end):funend(end)), 'User function calls (total)', '')));
		else
			funevals = NaN;
		end
		if nargout >= 3
			[timestart, timeend] = regexp(text, 'Time for solving problem\s+\d+\.\d+\s+seconds');
			if ~isempty(timestart) && ~isempty(timeend)
				time = str2double(strtrim(strrep(strrep(text(timestart(end):timeend(end)), 'seconds', ''), 'Time for solving problem', '')));
			else
				time = NaN;
			end
		end
		if nargout >= 4
			[fminstart, fminend] = regexp(text, 'Objective\s+[+-]?\d+\.?\d*E?[+-]?\d*');
			if ~isempty(fminstart) && ~isempty(fminend)
				fval = str2double(strtrim(strrep(text(fminstart(end):fminend(end)), 'Objective', '')));
			else
				[fminstart, fminend] = regexp(text, 'Objective\s+[+-]?[Ii]nfinity');
				if ~isempty(fminstart) && ~isempty(fminend)
					fval = strtrim(strrep(text(fminstart(end):fminend(end)), 'Objective', ''));
					if strcmpi(fval, 'infinity')
						fval = Inf;
					else
						fval = -Inf;
					end
				else
					fval = NaN;
				end
			end
		end
		if nargout >= 5
			[constrstart, constrend] = regexp(text, 'Nonlinear constraint violn\s+\d+\.?\d*E?[+-]?\d*');
			if ~isempty(constrstart) && ~isempty(constrend)
				constrviol = str2double(strtrim(strrep(text(constrstart(end):constrend(end)), 'Nonlinear constraint violn', '')));
			else
				constrviol = NaN;
			end
		end
		if nargout >= 6
			[primalinfeasstart, primalinfeasend] = regexp(text, 'Max Primal infeas\s+\d+ \d+\.?\d*E?[+-]?\d*');
			if ~isempty(primalinfeasstart) && ~isempty(primalinfeasend)
				primalinfeas = str2double(strtrim(regexprep(text(primalinfeasstart(end):primalinfeasend(end)), 'Max Primal infeas\s+\d+ ', '')));
			else
				primalinfeas = NaN;
			end
			[dualinfeasstart, dualinfeasend] = regexp(text, 'Max Dual infeas\s+\d+ \d+\.?\d*E?[+-]?\d*');
			if ~isempty(dualinfeasstart) && ~isempty(dualinfeasend)
				dualinfeas = str2double(strtrim(regexprep(text(dualinfeasstart(end):dualinfeasend(end)), 'Max Dual infeas\s+\d+ ', '')));
			else
				dualinfeas = NaN;
			end
			if ~isnan(primalinfeas) || ~isnan(dualinfeas)
				if isnan(primalinfeas)
					infeas = dualinfeas;
				elseif isnan(dualinfeas)
					infeas = primalinfeas;
				else
					infeas = max(primalinfeas, dualinfeas);
				end
			else
				infeas = NaN;
			end
		end
		if nargout >= 9
			[outputstart, outputend] = regexp(text, 'SNOPT.? EXIT.*?Problem name');
			if ~isempty(outputstart) && ~isempty(outputend)
				outputstr = strtrim(strrep(text(outputstart(end):outputend(end)), 'Problem name', ''));
			else
				outputstr = '';
			end
		end
		fclose(fid);
	else
		iter = NaN;
		funevals = NaN;
		infeas = NaN;
		fval = NaN;
		constrviol = NaN;
		outputstr = '';
		time = NaN;
	end
end