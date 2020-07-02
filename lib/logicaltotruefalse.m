function [onoff] = logicaltotruefalse(log)
	%LOGICALTOTRUEFALSE convert logical to 'true' or 'false'
	%	Input:
	%		log:	logical to convert
	%	Output:
	%		onoff:	'true' or cell array with 'true' where log is true and 'false' or cell array with 'false' where log is false
	if ~islogical(log)
		if ischar(log)
			if any(strcmpi(log, {'true', 'false'}))
				onoff = log;
				return;
			else
				error('logical2truefalse', 'Undefined function or variable logicaltotruefalse for input arguments of type ''%s''.', class(log));
			end
		elseif iscellstr(log)
			on = strcmpi(log, 'true');
			off = strcmpi(log, 'false');
			if all(all(on | off))
				onoff = log;
				return;
			else
				error('logical2truefalse', 'Undefined function or variable logicaltotruefalse for input arguments of type ''%s''.', class(log));
			end
		else
			error('logical2truefalse', 'Undefined function or variable logicaltotruefalse for input arguments of type ''%s''.', class(log));
		end
	end
	if isscalar(log)
		if log
			onoff = 'true';
		else
			onoff = 'false';
		end
	else
		onoff = cell(size(log));
		onoff(log) = {'true'};
		onoff(~log) = {'false'};
	end
end