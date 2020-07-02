function [log] = onofftological(onoff)
	%ONOFFTOLOGICAL convert 'on' or 'off' to logicale
	%	Input:
	%		onoff:	'on' or 'off' or cell array with 'on' and 'off'
	%	Output:
	%		log:	true, where onoff is 'on' and false, where onoff is 'off'
	if ~ischar(onoff)
		if islogical(onoff)
			log = onoff;
			return;
		elseif iscellstr(onoff)
			on = strcmpi(onoff, 'on');
			off = strcmpi(onoff, 'off');
			if all(all(on | off))
				log = on & ~off;
				return;
			else
				error('onoff2logical', 'Undefined function or variable onofftological for input arguments of type ''%s''.', class(onoff));
			end
		else
			error('onoff2logical', 'Undefined function or variable onofftological for input arguments of type ''%s''.', class(onoff));
		end
	else
		if strcmpi(onoff, 'on')
			log = true;
		elseif strcmpi(onoff, 'off')
			log = false;
		end
	end
end