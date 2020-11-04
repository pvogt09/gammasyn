function [output] = output_verbosity(verbosity, level)
	%OUTPUT_VERBOSITY indicator if verbosity level is above a certain threshold
	%	Input:
	%		verbosity:	verbosity level to check for threshold
	%		level:		threshold to output something
	%	Ouput:
	%		output:		indicator if verbosity is above output threshold
	if nargin <= 1
		level = 'iter';
	end
	display = {
		'off',				0;
		'iter',				5;
		'iter-detailed',	6;
		'notify',			1;
		'notify-detailed',	2;
		'final',			3;
		'final-detailed',	4
	};
	if ~ischar(level)
		error('control:design:gamma', 'Verbosity level threshold must be of type ''char'', not ''%s''.', class(level));
	end
	match = strcmpi(level, display(:, 1));
	if ~any(match(:)) || sum(match(:)) > 1
		error('control:design:gamma', 'Verbosity level threshold must be one of ''%s''.', strjoin(display(:, 1), ''', '''));
	end
	level = display{match, 2};
	if ~ischar(verbosity)
		if isa(verbosity, 'optimization.options.Options')
			verbosity = verbosity.Display;
		elseif isstruct(verbosity)
			if isfield(verbosity, 'Display')
				verbosity = verbosity.Display;
			else
				error('control:design:gamma', 'Verbosity level must have a field ''Display''.');
			end
		elseif isobject(verbosity)
			if isprop(verbosity, 'Display')
				verbosity = verbosity.Display;
			else
				error('control:design:gamma', 'Verbosity level must have a property ''Display''.');
			end
		else
			error('control:design:gamma', 'Verbosity level must be of type ''char'', not ''%s''.', class(level));
		end
	end
	if ~ischar(verbosity)
		error('control:design:gamma', 'Verbosity level must be of type ''char'', not ''%s''.', class(level));
	end
	match = strcmpi(verbosity, display(:, 1));
	if ~any(match(:)) || sum(match(:)) > 1
		error('control:design:gamma', 'Verbosity level must be one of ''%s''.', strjoin(display(:, 1), ''', '''));
	end
	output = display{match, 2} >= level;
end