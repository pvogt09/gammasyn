function [yesno] = logicaltoyesno(log)
	%LOGICALTOYESNO convert logical to 'yes' or 'no'
	%	Input:
	%		log:	logical to convert
	%	Output:
	%		yesno:	'yes' or cell array with 'yes' where log is true and 'no' or cell array with 'no' where log is false
	if ~islogical(log)
		if ischar(log)
			if any(strcmpi(log, {'yes', 'no'}))
				yesno = log;
				return;
			else
				error('logical2yesno', 'Undefined function or variable logicaltoyesno for input arguments of type ''%s''.', class(log));
			end
		elseif iscellstr(log)
			on = strcmpi(log, 'yes');
			off = strcmpi(log, 'no');
			if all(all(on | off))
				yesno = log;
				return;
			else
				error('logical2yesno', 'Undefined function or variable logicaltoyesno for input arguments of type ''%s''.', class(log));
			end
		else
			error('logical2yesno', 'Undefined function or variable logicaltoyesno for input arguments of type ''%s''.', class(log));
		end
	end
	if isscalar(log)
		if log
			yesno = 'yes';
		else
			yesno = 'no';
		end
	else
		yesno = cell(size(log));
		yesno(log) = {'yes'};
		yesno(~log) = {'no'};
	end
end