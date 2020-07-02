function [onoff] = logicaltoonoff(log)
	%LOGICALTOONOFF convert logical to 'on' or 'off'
	%	Input:
	%		log:	logical to convert
	%	Output:
	%		onoff:	'on' or cell array with 'on' where log is true and 'off' or cell array with 'off' where log is false
	if ~islogical(log)
		if ischar(log)
			if any(strcmpi(log, {'on', 'off'}))
				onoff = log;
				return;
			else
				error('logical2onoff', 'Undefined function or variable logicaltoonoff for input arguments of type ''%s''.', class(log));
			end
		elseif iscellstr(log)
			on = strcmpi(log, 'on');
			off = strcmpi(log, 'off');
			if all(all(on | off))
				onoff = log;
				return;
			else
				error('logical2onoff', 'Undefined function or variable logicaltoonoff for input arguments of type ''%s''.', class(log));
			end
		else
			error('logical2onoff', 'Undefined function or variable logicaltoonoff for input arguments of type ''%s''.', class(log));
		end
	end
	if isscalar(log)
		if log
			onoff = 'on';
		else
			onoff = 'off';
		end
	else
		onoff = cell(size(log));
		onoff(log) = {'on'};
		onoff(~log) = {'off'};
	end
end