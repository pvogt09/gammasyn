classdef Progress < handle
	%PROGRESS class for progressbar
	properties(Access=private)
		%handle to current waitbar
		handle,
		%number of steps
		steps,
		%current step
		iter,
		%show progressbar
		display,
		%write to console instead of waitbar
		console,
		%text to display
		text,
		%step only every i-th iteration
		stepevery
	end
	properties(Access=private, Transient=true)
		%running under old version of matlab
		oldversion = false
	end
	methods(Static=true)
		function [] = close_all()
			%CLOSE_ALL close all open waitbars
			wait = findall(0, 'tag', 'TMWWaitbar');
			delete(wait);
		end
	end
	methods
		function [this] = Progress(steps, text, display, stepevery)
			%PROGRESS create new progressbar
			%	Input:
			%		steps:		number of steps
			%		text:		text to display
			%		display:	show progressbar
			%		stepevery:	step counter only every i-th iteration
			%	Output:
			%		this:		instance
			this.steps = steps;
			if nargin <= 1
				text = '';
			end
			if nargin <= 2
				display = true;
			end
			if nargin <= 3
				this.stepevery = this.steplength();
			else
				this.stepevery = this.steplength(stepevery);
			end
			this.iter = 0;
			if ischar(display) && strcmpi(display, 'console')
				this.console = true;
				display = true;
			elseif ischar(display)
				this.console = false;
				display = true;
			else
				this.console = false;
			end
			this.display = logical(display);
			if this.console
				if isempty(text)
					this.text = [];
				else
					this.text = strrep(strrep(strrep(strrep(text, '$', ''), '\', ''), '{', ''), '}', '');
				end
			else
				this.text = text;
			end
			this.oldversion = false;
			if this.display
				if this.console
					if ~isempty(this.text)
						fprintf('%s\n', this.text);
					end
				else
					if verLessThan('matlab', '8.0.1') || ~verLessThan('matlab', '8.6')
						this.handle = waitbar(0, {this.text, ''} , 'CreateCancelBtn', 'setappdata(gcbf, ''cancelling'', 1)', 'CloseRequestFcn', 'delete(gcf)');
						this.oldversion = true;
					else
						this.handle = waitbar(0, {this.text, ''} , 'CreateCancelButton', 'setappdata(gcbf, ''cancelling'', 1)', 'CloseRequestFcn', 'delete(gcf)');
						this.oldversion = false;
					end
				end
			end
		end
		function [length] = steplength(this, stepevery)
			%STEPLENGTH steplength for progressbar
			%	Input:
			%		this:		instance
			%		stepevery:	display only every stepevery-th step
			%	Output:
			%		length:		steplength
			if nargin <= 1
				if this.steps <= 1000
					length = max(1, ceil(this.steps/100));
				else
					length = max(ceil(this.steps/100), 100);
				end
			else
				length = max(1, ceil(stepevery));
			end
		end
		function [] = step(this, text)
			%STEP step progressbar forward
			%	Input:
			%		this:		instance
			%		text:		text to display (can contain at most two format strings like %d or %f for current iteration and number of iterations
			this.iter = min(this.iter + 1, this.steps);
			if nargin <= 1
				text = sprintf('%d/%d', this.iter, this.steps);
			else
				if length(strfind(text, '%')) == 1
					text = sprintf(text, this.iter);
				elseif length(strfind(text, '%')) == 2
					text = sprintf(text, this.iter, this.steps);
				end
			end
			if this.display && (this.iter == 1 || this.iter == this.steps || mod(this.iter, this.stepevery) == 0)
				if this.console
					fprintf('.');
					if  this.iter == this.steps
						fprintf('\n');
					end
				else
					if (~this.oldversion && isvalid(this.handle)) || (this.oldversion && this.handle ~= -1)
						waitbar(this.iter/this.steps, this.handle, {this.text, text});
					else
						if this.oldversion
							this.handle = waitbar(0, {this.text, ''} , 'CreateCancelBtn', 'setappdata(gcbf, ''cancelling'', 1)', 'CloseRequestFcn', 'delete(gcf)');
							waitbar(this.iter/this.steps, this.handle, {this.text, text});
						else
							this.handle = waitbar(0, {this.text, ''} , 'CreateCancelButton', 'setappdata(gcbf, ''cancelling'', 1)', 'CloseRequestFcn', 'delete(gcf)');
							waitbar(this.iter/this.steps, this.handle, {this.text, text});
						end
					end
				end
			end
		end
		function [cancelled] = iscancelled(this)
			%ISCANCELLED return if progressbar was cancelled
			%	Input:
			%		this:		instance
			%	Output:
			%		cancelled:	true, if progressbar was cancelled
			if ~isempty(this.handle) && ishandle(this.handle)
				if (~this.oldversion && isvalid(this.handle)) || (this.oldversion && this.handle ~= -1)
					if this.oldversion
						temp = getappdata(this.handle, 'cancelling');
					else
						temp = getappdata(this.handle, 'cancelled');
					end
					if isempty(temp)
						cancelled = false;
					else
						cancelled = logical(temp);
					end
				else
					cancelled = true;
				end
			else
				cancelled = ~isempty(this.handle);
			end
		end
		function [] = delete(this)
			%DELETE destructor
			%	Input:
			%		this:		instance
			if ishandle(this.handle)
				delete(this.handle);
			end
		end
	end
end