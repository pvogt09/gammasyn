classdef SolutionSet < handle
	%SOLUTIONSET set of solutions of gammasyn

	properties(Constant=true)
		% location for saved files
		SAVEPATH = realpath(fullfile(export.destpath(), 'ControllerGamma'));
	end

	properties(SetAccess=protected)
		% ids for solutions
		ids;
		% controller the solutions were calcualted for
		controller;
		% number of controls of controlled system
		p;
		% number of measurements of controlled system
		q;
		% number of derivative measurements of controlled system
		q_dot;
		% number of references of controlled system
		r;
		% controlled systems
		systems;
		% GammaAreas used for strict problem formulation
		areafun_strict;
		% GammaAreas used for loose problem formulation
		areafun_loose;
		% weights used for strict problem formulation
		weight_strict;
		% weights used for loose problem formulation
		weight_loose;
		% system and gain settings used for strict problem formulation
		dimensions_strict;
		% system and gain settings used for loose problem formulation
		dimensions_loose;
		% bound information for gain matrices
		bounds;
		% constraints for proportional gain matrices
		R_fixed;
		% constraints for derivative gain matrices
		K_fixed;
		% constraints for prefilter gain matrices
		F_fixed;
		% constraints for all gain matrices
		RKF_fixed;
		% inequality constraints for proportional gain matrices
		R_bounds
		% inequality constraints for derivative gain matrices
		K_bounds;
		% inequality constraints for prefilter gain matrices
		F_bounds;
		% inequality constraints for all gain matrices
		RKF_bounds;
		% comment on specific solution
		solutioncomment = cell(0, 1)
	end

	properties(Access=protected)
		% initial gain matrix
		R_0;
		% optimal proportional gain matrix
		R_opt;
		% optimal derivative gain matrix
		K_opt;
		% optimal prefilter gain matrix
		F_opt;
		% optimal function value
		J_opt;
		% optimizer output structure
		info;
		% optimization options
		options;
		% objective options used for strict problem formulation
		objectiveoptions_strict;
		% objective options used for loose problem formulation
		objectiveoptions_loose;
		% comment on solution set
		comment = ''
	end

	methods(Static=true)
		function [this, isloaded] = load(controller, system, areafun, weight, R_fixed, R_bounds, loadfirst)
			%LOAD load solutions from file
			%	Input:
			%		controller:	controller the solution was calculated for
			%		p:			number of controls or system to read number of controls from
			%		q:			number of measurements or desired pole area
			%		q_dot:		number of derivative measurements or weight for different objective function parts
			%		R_fixed:	number of references or constraint on optimal solution
			%		R_bounds:	ineqaulity constraints on optimal solution
			%		loadfirst:	load first file without user interaction
			%	Output:
			%		this:		instance or empty array if nothing was loaded
			%		isloaded:	indicator, if a valid object was loaded from the file
			if ~isa(controller, 'control.design.outputfeedback.OutputFeedback')
				error('control:design:gamma:solution:input', 'Controller must be a subclass of OutputFeedback.');
			end
			if nargin >= 6 && ~islogical(R_fixed) && ~isscalar(R_fixed)
				if nargin <= 6
					loadfirst = false;
				end
				[this, isloaded] = control.design.gamma.SolutionSet.loadproblem(controller, system, areafun, weight, R_fixed, R_bounds, [], loadfirst);
			elseif nargin >= 5 && ~islogical(R_fixed) && ~isscalar(R_fixed)
				if nargin <= 6
					R_bounds = [];
				end
				if nargin <= 7
					loadfirst = false;
				end
				[this, isloaded] = control.design.gamma.SolutionSet.loadproblem(controller, system, areafun, weight, R_fixed, R_bounds, [], loadfirst);
			else
				if nargin <= 4
					loadfirst = false;
				end
				qset = false;
				q_dotset = false;
				rset = false;
				if isscalar(system)
					if isnumeric(system)
						loadp = system;
					elseif isstruct(system) && isfield(system, 'B')
						loadp = size(system.B, 2);
						if isfield(system, 'C')
							loadq = size(system.C, 1);
							qset = true;
						end
						if isfield(system, 'c')
							loadq = size(system.c, 1);
							qset = true;
						end
						if isfield(system, 'C_dot')
							loadq_dot = size(system.C_dot, 1);
							q_dotset = true;
						end
						if isfield(system, 'c_dot')
							loadq_dot = size(system.c_dot, 1);
							q_dotset = true;
						end
						if isfield(system, 'C_ref')
							loadr = size(system.C_ref, 1);
							rset = true;
						end
						if isfield(system, 'c_ref')
							loadr = size(system.c_ref, 1);
							rset = true;
						end
					elseif isstruct(system) && isfield(system, 'b')
						loadp = size(system.b, 2);
						if isfield(system, 'C')
							loadq = size(system.C, 1);
							qset = true;
						end
						if isfield(system, 'c')
							loadq = size(system.c, 1);
							qset = true;
						end
						if isfield(system, 'C_dot')
							loadq_dot = size(system.C_dot, 1);
							q_dotset = true;
						end
						if isfield(system, 'c_dot')
							loadq_dot = size(system.c_dot, 1);
							q_dotset = true;
						end
						if isfield(system, 'c_dot')
							loadq_dot = size(system.c_dot, 1);
							q_dotset = true;
						end
						if isfield(system, 'C_ref')
							loadr = size(system.C_ref, 1);
							rset = true;
						end
						if isfield(system, 'c_ref')
							loadr = size(system.c_ref, 1);
							rset = true;
						end
					elseif isa(system, 'tf')
						[~, B, C, ~] = ssdata(system);
						loadp = size(B, 2);
						if nargin <= 2 || (nargin >= 3 && islogical(areafun))
							loadq = size(C, 1);
							qset = true;
						end
						if nargin <= 3 || (nargin >= 4 && islogical(weight))
							loadq_dot = size(C, 1);
							q_dotset = true;
						end
						if nargin <= 4 || (nargin >= 5 && islogical(R_fixed))
							loadr = size(C, 1);
							rset = true;
						end
					elseif isa(system, 'ss')
						[~, B, C, ~] = ssdata(system);
						loadp = size(B, 2);
						if nargin <= 2 || (nargin >= 3 && islogical(areafun))
							loadq = size(C, 1);
							qset = true;
						end
						if nargin <= 3 || (nargin >= 4 && islogical(weight))
							loadq_dot = size(C, 1);
							q_dotset = true;
						end
						if nargin <= 4 || (nargin >= 5 && islogical(R_fixed))
							loadr = size(C, 1);
							rset = true;
						end
					else
						error('control:design:gamma:solution:input', 'Number of controls must be given.');
					end
				else
					if isstruct(system) && ~isempty(system)
						if isfield(system, 'B')
							loadp = size(system(1).B, 2);
							if isfield(system, 'C')
								loadq = size(system(1).C, 1);
								qset = true;
							end
							if isfield(system, 'c')
								loadq = size(system(1).c, 1);
								qset = true;
							end
							if isfield(system, 'C_dot')
								loadq_dot = size(system(1).C_dot, 1);
								q_dotset = true;
							end
							if isfield(system, 'c_dot')
								loadq_dot = size(system(1).c_dot, 1);
								q_dotset = true;
							end
							if isfield(system, 'C_ref')
								loadr = size(system(1).C_ref, 1);
								rset = true;
							end
							if isfield(system, 'c_ref')
								loadr = size(system(1).c_ref, 1);
								rset = true;
							end
						elseif isfield(system, 'b')
							loadp = size(system(1).b, 2);
							if isfield(system, 'C')
								loadq = size(system(1).C, 1);
								qset = true;
							end
							if isfield(system, 'c')
								loadq = size(system(1).c, 1);
								qset = true;
							end
							if isfield(system, 'C_dot')
								loadq_dot = size(system(1).C_dot, 1);
								q_dotset = true;
							end
							if isfield(system, 'c_dot')
								loadq_dot = size(system(1).c_dot, 1);
								q_dotset = true;
							end
							if isfield(system, 'C_ref')
								loadr = size(system(1).C_ref, 1);
								rset = true;
							end
							if isfield(system, 'c_ref')
								loadr = size(system(1).c_ref, 1);
								rset = true;
							end
						else
							error('control:design:gamma:solution:input', 'Number of controls must be given.');
						end
					else
						error('control:design:gamma:solution:input', 'Number of controls must be given.');
					end
				end
				if nargin >= 3
					if islogical(areafun)
						loadfirst = areafun;
						if ~qset
							error('control:design:gamma:solution:input', 'Number of measurements must be given.');
						end
					else
						if isscalar(areafun)
							if isnumeric(areafun)
								loadq = areafun;
							elseif isstruct(areafun) && isfield(areafun, 'C')
								loadq = size(areafun.C, 1);
							elseif isstruct(areafun) && isfield(areafun, 'c')
								loadq = size(areafun.c, 1);
							elseif isa(areafun, 'tf')
								[~, ~, C, ~] = ssdata(areafun);
								loadq = size(C, 1);
							elseif isa(areafun, 'ss')
								[~, ~, C, ~] = ssdata(areafun);
								loadq = size(C, 1);
							else
								error('control:design:gamma:solution:input', 'Number of measurements must be given.');
							end
						else
							if isstruct(areafun) && ~isempty(areafun)
								if isfield(areafun, 'C')
									loadq = size(areafun(1).C, 1);
								elseif isfield(areafun, 'c')
									loadq = size(areafun(1).c, 1);
								else
									error('control:design:gamma:solution:input', 'Number of measurements must be given.');
								end
							else
								error('control:design:gamma:solution:input', 'Number of measurements must be given.');
							end
						end
						qset = true;
					end
					if nargin >= 4
						if islogical(weight)
							loadfirst = weight;
							if ~q_dotset
								error('control:design:gamma:solution:input', 'Number of derivative measurements must be given.');
							end
						else
							if isscalar(weight)
								if isnumeric(weight)
									loadq_dot = weight;
								elseif isstruct(weight) && isfield(weight, 'C_dot')
									loadq_dot = size(weight.C, 1);
								elseif isstruct(weight) && isfield(weight, 'c_dot')
									loadq_dot = size(weight.c, 1);
								elseif isa(weight, 'tf')
									[~, ~, C, ~] = ssdata(weight);
									loadq_dot = size(C, 1);
								elseif isa(areafun, 'ss')
									[~, ~, C, ~] = ssdata(weight);
									loadq_dot = size(C, 1);
								else
									error('control:design:gamma:solution:input', 'Number of derivative derivative measurements must be given.');
								end
							else
								if isstruct(weight) && ~isempty(weight)
									if isfield(weight, 'C')
										loadq_dot = size(weight(1).C, 1);
									elseif isfield(weight, 'c')
										loadq_dot = size(weight(1).c, 1);
									else
										error('control:design:gamma:solution:input', 'Number of derivative measurements must be given.');
									end
								else
									error('control:design:gamma:solution:input', 'Number of derivative measurements must be given.');
								end
							end
						end
						q_dotset = true;
					end
					if nargin >= 5
						if islogical(R_fixed)
							loadfirst = R_fixed;
							if ~rset
								error('control:design:gamma:solution:input', 'Number of references must be given.');
							end
						else
							if isscalar(R_fixed)
								if isnumeric(R_fixed)
									loadr = R_fixed;
								elseif isstruct(R_fixed) && isfield(R_fixed, 'C_ref')
									loadr = size(R_fixed.C_ref, 1);
								elseif isstruct(R_fixed) && isfield(R_fixed, 'c_ref')
									loadr = size(R_fixed.c_ref, 1);
								elseif isa(R_fixed, 'tf')
									[~, ~, C, ~] = ssdata(R_fixed);
									loadr = size(C, 1);
								elseif isa(areafun, 'ss')
									[~, ~, C, ~] = ssdata(R_fixed);
									loadr = size(C, 1);
								else
									error('control:design:gamma:solution:input', 'Number of references must be given.');
								end
							else
								if isstruct(R_fixed) && ~isempty(R_fixed)
									if isfield(R_fixed, 'C_ref')
										loadr = size(R_fixed(1).C_ref, 1);
									elseif isfield(R_fixed, 'c_ref')
										loadr = size(R_fixed(1).c_ref, 1);
									else
										error('control:design:gamma:solution:input', 'Number of references must be given.');
									end
								else
									error('control:design:gamma:solution:input', 'Number of references must be given.');
								end
							end
							rset = true;
							if nargin < 6
								loadfirst = false;
							end
						end
					end
				end
				if ~qset
					error('control:design:gamma:solution:input', 'Number of measurements must be given.');
				end
				if ~q_dotset
					error('control:design:gamma:solution:input', 'Number of derivative measurements must be given.');
				end
				if ~rset
					error('control:design:gamma:solution:input', 'Number of references must be given.');
				end
				if ~isreal(loadp) || isnan(loadp) || isinf(loadp) || floor(loadp) ~= ceil(loadp) && loadp > 0
					error('control:design:gamma:solution:input', 'Number of controls must be a finite number.');
				end
				if ~isreal(loadq) || isnan(loadq) || isinf(loadq) || floor(loadq) ~= ceil(loadq) && loadq > 0
					error('control:design:gamma:solution:input', 'Number of measurements must be a finite number.');
				end
				if ~isreal(loadq_dot) || isnan(loadq_dot) || isinf(loadq_dot) || floor(loadq_dot) ~= ceil(loadq_dot) && loadq_dot > 0
					error('control:design:gamma:solution:input', 'Number of derivative measurements must be a finite number.');
				end
				if ~isreal(loadr) || isnan(loadr) || isinf(loadr) || floor(loadr) ~= ceil(loadr) && loadr > 0
					error('control:design:gamma:solution:input', 'Number of references must be a finite number.');
				end
				[this, isloaded] = control.design.gamma.SolutionSet.loadcontroller(controller, loadp, loadq, loadq_dot, loadr, [], loadfirst);
			end
		end

		function [this, isloaded] = loadas(prefix, controller, system, areafun, weight, R_fixed, R_bounds, loadfirst)
			%LOADAS load solutions from file with prefix
			%	Input:
			%		prefix:		prefix for the solution filename
			%		controller:	controller the solution was calculated for
			%		p:			number of controls or system to read number of controls from
			%		q:			number of measurements or desired pole area
			%		q_dot:		number of derivative measurements or weight for different objective function parts
			%		R_fixed:	number of references or constraint on optimal solution
			%		R_bounds:	inequality constraints on optimal solution
			%		loadfirst:	load first file without user interaction
			%	Output:
			%		this:		instance or empty array if nothing was loaded
			%		isloaded:	indicator, if a valid object was loaded from the file
			if ~ischar(prefix) || ~isempty(strfind(prefix, '/')) || ~isempty(strfind(prefix, '\'))
				error('control:design:gamma:solution:load', 'Filename is invalid.');
			end
			if ~isa(controller, 'control.design.outputfeedback.OutputFeedback')
				error('control:design:gamma:solution:input', 'Controller must be a subclass of OutputFeedback.');
			end
			if nargin >= 7 && ~islogical(R_bounds)
				if nargin <= 7
					loadfirst = false;
				end
				[this, isloaded] = control.design.gamma.SolutionSet.loadproblem(controller, system, areafun, weight, R_fixed, R_bounds, prefix, loadfirst);
			elseif nargin >= 6 && ~islogical(R_fixed) && ~isscalar(R_fixed)
				if nargin <= 6
					R_bounds = [];
				end
				if nargin <= 7
					loadfirst = false;
				end
				[this, isloaded] = control.design.gamma.SolutionSet.loadproblem(controller, system, areafun, weight, R_fixed, R_bounds, prefix, loadfirst);
			else
				if nargin <= 5
					loadfirst = false;
				end
				qset = false;
				if isscalar(system)
					if isnumeric(system)
						loadp = system;
					elseif isstruct(system) && isfield(system, 'B')
						loadp = size(system.B, 2);
						if isfield(system, 'C')
							loadq = size(system.C, 1);
							qset = true;
						end
						if isfield(system, 'c')
							loadq = size(system.c, 1);
							qset = true;
						end
					elseif isstruct(system) && isfield(system, 'b')
						loadp = size(system.b, 2);
						if isfield(system, 'C')
							loadq = size(system.C, 1);
							qset = true;
						end
						if isfield(system, 'c')
							loadq = size(system.c, 1);
							qset = true;
						end
					elseif isa(system, 'tf')
						[~, B, C, ~] = ssdata(system);
						loadp = size(B, 2);
						if nargin <= 2 || (nargin >= 3 && islogical(areafun))
							loadq = size(C, 1);
							qset = true;
						end
					elseif isa(system, 'ss')
						[~, B, C, ~] = ssdata(system);
						loadp = size(B, 2);
						if nargin <= 2 || (nargin >= 3 && islogical(areafun))
							loadq = size(C, 1);
							qset = true;
						end
					else
						error('control:design:gamma:solution:input', 'Number of controls must be given.');
					end
				else
					if isstruct(system) && ~isempty(system)
						if isfield(system, 'B')
							loadp = size(system(1).B, 2);
							if isfield(system, 'C')
								loadq = size(system(1).C, 1);
								qset = true;
							end
							if isfield(system, 'c')
								loadq = size(system(1).c, 1);
								qset = true;
							end
						elseif isfield(system, 'b')
							loadp = size(system(1).b, 2);
							if isfield(system, 'C')
								loadq = size(system(1).C, 1);
								qset = true;
							end
							if isfield(system, 'c')
								loadq = size(system(1).c, 1);
								qset = true;
							end
						else
							error('control:design:gamma:solution:input', 'Number of controls must be given.');
						end
					else
						error('control:design:gamma:solution:input', 'Number of controls must be given.');
					end
				end
				if nargin >= 4
					if islogical(areafun)
						loadfirst = areafun;
						if ~qset
							error('control:design:gamma:solution:input', 'Number of measurements must be given.');
						end
					else
						if isscalar(areafun)
							if isnumeric(areafun)
								loadq = areafun;
							elseif isstruct(areafun) && isfield(areafun, 'C')
								loadq = size(areafun.C, 1);
							elseif isstruct(areafun) && isfield(areafun, 'c')
								loadq = size(areafun.c, 1);
							elseif isa(areafun, 'tf')
								[~, ~, C, ~] = ssdata(areafun);
								loadq = size(C, 1);
							elseif isa(areafun, 'ss')
								[~, ~, C, ~] = ssdata(areafun);
								loadq = size(C, 1);
							else
								error('control:design:gamma:solution:input', 'Number of measurements must be given.');
							end
						else
							if isstruct(areafun) && ~isempty(areafun)
								if isfield(areafun, 'C')
									loadq = size(areafun(1).C, 1);
								elseif isfield(areafun, 'c')
									loadq = size(areafun(1).c, 1);
								else
									error('control:design:gamma:solution:input', 'Number of measurements must be given.');
								end
							else
								error('control:design:gamma:solution:input', 'Number of measurements must be given.');
							end
						end
					end
					if nargin >= 5
						if islogical(weight)
							loadfirst = weight;
							if ~qset
								error('control:design:gamma:solution:input', 'Number of measurements must be given.');
							end
						else
							if isscalar(weight)
								if isnumeric(weight)
									loadq_dot = weight;
								elseif isstruct(weight) && isfield(weight, 'C')
									loadq_dot = size(weight.C, 1);
								elseif isstruct(weight) && isfield(weight, 'c')
									loadq_dot = size(weight.c, 1);
								elseif isa(weight, 'tf')
									[~, ~, C, ~] = ssdata(weight);
									loadq_dot = size(C, 1);
								elseif isa(areafun, 'ss')
									[~, ~, C, ~] = ssdata(weight);
									loadq_dot = size(C, 1);
								else
									error('control:design:gamma:solution:input', 'Number of derivative measurements must be given.');
								end
							else
								if isstruct(weight) && ~isempty(weight)
									if isfield(weight, 'C')
										loadq_dot = size(weight(1).C, 1);
									elseif isfield(weight, 'c')
										loadq_dot = size(weight(1).c, 1);
									else
										error('control:design:gamma:solution:input', 'Number of measurements must be given.');
									end
								else
									error('control:design:gamma:solution:input', 'Number of measurements must be given.');
								end
							end
						end
					end
					if nargin >= 6
						if islogical(R_fixed)
							loadfirst = R_fixed;
							if ~rset
								error('control:design:gamma:solution:input', 'Number of references must be given.');
							end
						else
							if isscalar(R_fixed)
								if isnumeric(R_fixed)
									loadr = R_fixed;
								elseif isstruct(R_fixed) && isfield(R_fixed, 'C_ref')
									loadr = size(R_fixed.C_ref, 1);
								elseif isstruct(R_fixed) && isfield(R_fixed, 'c_ref')
									loadr = size(R_fixed.c_ref, 1);
								elseif isa(R_fixed, 'tf')
									[~, ~, C, ~] = ssdata(R_fixed);
									loadr = size(C, 1);
								elseif isa(areafun, 'ss')
									[~, ~, C, ~] = ssdata(R_fixed);
									loadr = size(C, 1);
								else
									error('control:design:gamma:solution:input', 'Number of references must be given.');
								end
							else
								if isstruct(R_fixed) && ~isempty(R_fixed)
									if isfield(R_fixed, 'C_ref')
										loadr = size(R_fixed(1).C_ref, 1);
									elseif isfield(R_fixed, 'c_ref')
										loadr = size(R_fixed(1).c_ref, 1);
									else
										error('control:design:gamma:solution:input', 'Number of references must be given.');
									end
								else
									error('control:design:gamma:solution:input', 'Number of references must be given.');
								end
							end
							if nargin < 7
								loadfirst = false;
							end
						end
					end
				end
				if ~isreal(loadp) || isnan(loadp) || isinf(loadp) || floor(loadp) ~= ceil(loadp) && loadp > 0
					error('control:design:gamma:solution:input', 'Number of controls must be a finite number.');
				end
				if ~isreal(loadq) || isnan(loadq) || isinf(loadq) || floor(loadq) ~= ceil(loadq) && loadq > 0
					error('control:design:gamma:solution:input', 'Number of measurements must be a finite number.');
				end
				if ~isreal(loadq_dot) || isnan(loadq_dot) || isinf(loadq_dot) || floor(loadq_dot) ~= ceil(loadq_dot) && loadq_dot > 0
					error('control:design:gamma:solution:input', 'Number of derivative measurements must be a finite number.');
				end
				if ~isreal(loadr) || isnan(loadr) || isinf(loadr) || floor(loadr) ~= ceil(loadr) && loadr > 0
					error('control:design:gamma:solution:input', 'Number of references must be a finite number.');
				end
				[this, isloaded] = control.design.gamma.SolutionSet.loadcontroller(controller, loadp, loadq, loadq_dot, loadr, prefix, loadfirst);
			end
		end

		function [this, isloaded, file] = loadfilename(filename, loadfirst)
			%LOADFILENAME load a SolutionSet from a filename in the savepath folder recursively
			%	Input:
			%		filename:	filename or regular expression for filename to load
			%		loadfirst:	load first matching file
			%	Output:
			%		this:		instance or cell array of instances that were loaded
			%		isloaded:	indicator, if a valid solutionSet was loaded
			%		file:		file, the solution was loaded from
			if nargin <= 1
				loadfirst = false;
			end
			if ~ischar(filename)
				error('control:design:gamma:solution:input', 'Filename must be of type ''char''.');
			end
			if ~isempty(regexp(filename, '^(\w+_)?Controller_\w+_\d+_\d+_\d+_\d+_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}(\.mat)?$', 'once'))
				if ~isempty(strfind(filename, '.mat')) && strcmpi(filename(end - 3:end), '.mat')
					filename = ['^', regexptranslate('escape', filename), '$'];
				else
					filename = ['^', regexptranslate('escape', filename), '\.mat$'];
				end
			else
				if ~isempty(strfind(filename, '.mat')) && strcmpi(filename(end - 3:end), '.mat')
					if filename(end) ~= '$'
						filename = [filename, '\.mat$'];
					end
				end
			end
			files = find_files_regexp(control.design.gamma.SolutionSet.SAVEPATH, filename);
			if isempty(files)
				this = [];
				if nargout >= 2
					isloaded = false;
				end
				return;
			else
				if length(files) == 1 || loadfirst
					load(files{1});
					if ~exist('this', 'var') || isempty(this) %#ok<NODEF> this is loaded from the file
						warning('control:design:gamma:solution:load', 'File ''%s'' is invalid.', files{ii});
						this = [];
						if nargout >= 2
							isloaded = true;
						end
						return;
					end
					if nargout >= 2
						isloaded = true;
					end
				else
					objects = cell(size(files, 2));
					parfor ii = 1:size(files, 1)
						temp = load(files{ii});
						if ~isfield(temp, 'this') || isempty(temp.this)
							warning('control:design:gamma:solution:load', 'File ''%s'' is invalid.', files{ii});
							this = [];
						else
							this = temp.this;
						end
						objects{ii, 1} = this;
					end
					empty = isemptycell(objects);
					objects(empty) = [];
					files(empty) = [];
					if all(empty)
						this = [];
						if nargout >= 2
							isloaded = false;
							if nargout >= 3
								file = cell(0, 1);
							end
						end
					else
						this = objects;
						if size(objects, 1) == 1
							this = this{1};
						end
						if nargout >= 2
							isloaded = true;
							if nargout >= 3
								if size(objects, 1) == 1
									file = files{1};
								else
									file = files;
								end
							end
						end
					end
				end
			end
		end

		function [this, isloaded] = loadfile(file)
			%LOADFILE load a SolutionSet from a file
			%	Input:
			%		file:		file to load
			%	Output:
			%		this:		instance or cell array of instances that were loaded
			%		isloaded:	indicator, if a valid solutionSet was loaded
			if ~ischar(file)
				error('control:design:gamma:solution:input', 'Filename must be of type ''char''.');
			end
			if ~exist(file, 'file')
				this = [];
				isloaded = false;
				return;
			end
			[~, filename, ext] = fileparts(file);
			filename = [filename, ext];
			if isempty(regexp(filename, '^(\w+_)?Controller_\w+_\d+_\d+_\d+_\d+_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}(\.mat)?$', 'once'))
				this = [];
				isloaded = false;
				return;
			end
			temp = load(file);
			if ~isfield(temp, 'this') || isempty(temp.this)
				warning('control:design:gamma:solution:load', 'File ''%s'' is invalid.', file);
				this = [];
			else
				this = temp.this;
			end
			isloaded = ~isempty(this);
		end

		function [files] = clearcache()
			%CLEARCACHE delete all controllers in cache
			%	Output:
			%		files:	deleted files
			filesindir = dir(control.design.gamma.SolutionSet.SAVEPATH);
			files = cell(0);
			for i = 1:length(filesindir)
				if ~filesindir(i).isdir
					if regexp(filesindir(i).name, '^Controller_.+\.mat$')
						files{end+1} = filesindir(i).name;
					end
				end
			end
			rem = input('Should all controllers in cache be deleted?\n');
			if ~isempty(rem) && ((ischar(rem) && (strcmpi(rem, 'y') || strcmpi(rem, 'j'))) || (~ischar(rem) && logical(rem)))
				for i =1:length(files) %#ok<FORPF> no parfor for file system operations
					delete(fullfile(control.design.gamma.SolutionSet.SAVEPATH, files{i}));
				end
			else
				files = cell(0);
			end
		end

		function [is] = isvalidfilename(filename)
			%ISVALIDFILENAME return if a filename is a valid name for a solution
			%	Input:
			%		filename:	filename to check
			%	Output:
			%		is:			true, if the filename is valid, else false
			if ~ischar(filename)
				if iscellstr(filename)
					names = reshape(filename, [], 1);
					is = false(size(names), 1);
					parfor ii = 1:size(names, 1)
						is(ii, 1) = ~isempty(regexp(names{ii, 1}, '^(\w+_)?Controller_\w+_\d+_\d+_\d+_\d+_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}(\.mat)?$', 'once'));
					end
					is = reshape(is, size(filename));
				else
					error('control:design:gamma:solution:input', 'Filename must be of type ''char''.');
				end
			else
				is = ~isempty(regexp(filename, '^(\w+_)?Controller_\w+_\d+_\d+_\d+_\d+_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}_[0-9a-fA-F]{32}(\.mat)?$', 'once'));
			end
		end
	end

	methods(Static=true, Access=protected)
		function [this, isloaded] = loadproblem(controller, system, areafun, weight, R_fixed, R_bounds, prefix, loadfirst)
			%LOADPROBLEM load controller for problem from matching file
			%	Input:
			%		controller:	controller for the problem
			%		system:		systems to place poles for
			%		areafun:	desired pole area
			%		weight:		weight for different objective function parts
			%		R_fixed:	constraint on optimal solution
			%		R_bounds:	inequality constraints on optimal solution
			%		prefix:		prefix for filename
			%		loadfirst:	load first matching file without user input
			%	Output:
			%		this:		instance or empty array if nothing was loaded
			%		isloaded:	indicator, if a valid object was loaded
			if nargin <= 5
				R_bounds = [];
			end
			if nargin <= 6
				prefix = [];
			end
			if nargin <= 7
				loadfirst = false;
			end
			if nargin <= 4
				[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF();
			else
				[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF(R_fixed);
			end
			if nargin <= 5
				[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF();
			else
				[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF(R_bounds);
			end
			systemoptions = struct();
			[system, areafunstrict, areafunloose, weightstrict, weightloose, dimensionsstrict, dimensionsloose, ~, bound] = checkandtransformargs(system, areafun, weight, systemoptions, R_fixed, K_fixed, F_fixed, RKF_fixed, true, true, R_bounds, K_bounds, RKF_bounds, F_bounds);
			p = size(system(1).B, 2);
			q = size(system(1).C, 1);
			q_dot = size(system(1).C_dot, 1);
			r = size(system(1).C_ref, 1);
			name = calculate_filename(controller, p, q, q_dot, r, system, areafunstrict, areafunloose, weightstrict, weightloose, dimensionsstrict, dimensionsloose, bound);
			if isempty(prefix)
				teststring = [name, '.mat'];
			else
				if ~ischar(prefix) || ~isempty(strfind(prefix, '/')) || ~isempty(strfind(prefix, '\'))
					error('control:design:gamma:solution:load', 'Prefix is invalid.');
				end
				if prefix(end) ~= '_'
					prefix = [prefix, '_'];
				end
				teststring = [prefix, name, '.mat'];
			end
			filesindir = dir(control.design.gamma.SolutionSet.SAVEPATH);
			files = cell(0);
			for i = 1:length(filesindir)
				if ~filesindir(i).isdir
					if any(strcmpi(filesindir(i).name, teststring))
						files{end+1} = fullfile(control.design.gamma.SolutionSet.SAVEPATH, filesindir(i).name);
					end
				end
			end
			if isempty(files)
				this = [];
				if nargout >= 2
					isloaded = false;
				end
				return;
			else
				if length(files) == 1 || loadfirst
					load(files{1});
					if ~exist('this', 'var') || isempty(this) %#ok<NODEF> this is loaded from the file
						warning('control:design:gamma:solution:load', 'File ''%s'' is invalid.', files{i});
						this = [];
						if nargout >= 2
							isloaded = true;
						end
						return;
					end
					if nargout >= 2
						isloaded = true;
					end
				else
					this = [];
					if nargout >= 2
						isloaded = false;
					end
				end
			end
		end

		function [this, isloaded] = loadcontroller(controller, p, q, q_dot, r, prefix, loadfirst)
			%LOADCONTROLLER load controller for problem from matching files
			%	Input:
			%		controller:	controller for the problem
			%		p:			number of controls
			%		q:			number of outputs
			%		q_dot:		number of derivative outputs
			%		prefix:		prefix for filename
			%		loadfirst:	load first matching file without user input
			%	Output:
			%		this:		instance or empty array if nothing was loaded
			%		isloaded:	indicator, if a valid object was loaded
			if nargin <= 5
				prefix = [];
			end
			if nargin <= 6
				loadfirst = false;
			end
			if ~isnumeric(p) || ~isscalar(p) || p <= 0 || floor(p) ~= p
				error('control:design:gamma:solution:load', 'number of controls must be a scalar number.');
			end
			if ~isnumeric(q) || ~isscalar(q) || q <= 0 || floor(q) ~= q
				error('control:design:gamma:solution:load', 'number of measurements must be a scalar number.');
			end
			if ~isnumeric(q_dot) || ~isscalar(q_dot) || q_dot < 0 || floor(q_dot) ~= q_dot
				error('control:design:gamma:solution:load', 'number of derivative measurements must be a scalar number.');
			end
			if ~isnumeric(r) || ~isscalar(r) || r < 0 || floor(r) ~= r
				error('control:design:gamma:solution:load', 'number of references must be a scalar number.');
			end
			name = calculate_filename(controller, p, q, q_dot, r, [], [], [], [], [], [], [], [], true);
			teststringprefix = [];
			if isempty(prefix)
				teststring = ['^', regexptranslate('escape', name), '_.+\.mat$'];
			else
				if ~ischar(prefix) || ~isempty(strfind(prefix, '/')) || ~isempty(strfind(prefix, '\'))
					error('control:design:gamma:solution:load', 'Prefix is invalid.');
				end
				teststring = ['^', regexptranslate('escape', prefix), regexptranslate('escape', name), '_.+\.mat$'];
				teststringprefix = ['^', regexptranslate('escape', prefix), regexptranslate('escape', name), '\.mat$'];
			end
			filesindir = dir(control.design.gamma.SolutionSet.SAVEPATH);
			files = cell(0);
			for i = 1:length(filesindir)
				if ~filesindir(i).isdir
					if any(regexp(filesindir(i).name, teststring)) || (~isempty(teststringprefix) && any(regexp(filesindir(i).name, teststringprefix)))
						files{end+1} = fullfile(control.design.gamma.SolutionSet.SAVEPATH, filesindir(i).name);
					end
				end
			end
			if isempty(files)
				this = [];
				if nargout >= 2
					isloaded = false;
				end
				return;
			else
				if length(files) == 1 || loadfirst
					load(files{1});
					if ~exist('this', 'var') || isempty(this) %#ok<NODEF> this is loaded from the file
						warning('control:design:gamma:solution:load', 'File ''%s'' is invalid.', files{i});
						this = [];
						if nargout >= 2
							isloaded = true;
						end
						return;
					end
					if nargout >= 2
						isloaded = true;
					end
				else
					disp('Available Controllers:');
					display = cell(length(files), 6);
					controllers = cell(length(files), 1);
					maxsolver = zeros(size(controllers));
					maxJ = zeros(size(controllers));
					maxsystem = zeros(size(controllers));
					maxareas = zeros(size(controllers));
					parfor i = 1:length(files)
						parthis = []; %#ok<NASGU> prevent parfor warning because of temporary variables
						try
							parthis = load(files{i});
						catch e
							warning(e.identifier, e.message);
							parthis = [];
						end
						if isempty(parthis) || ~isstruct(parthis) || ~isfield(parthis, 'this')
							warning('control:design:gamma:solution:load', 'File ''%s'' is invalid.', files{i});
							continue;
						end
						parthis = parthis.this;
						[~, optidx] = min(parthis.J_opt);
						if isempty(optidx)
							optidx = 1;
							if isempty(parthis.J_opt)
								continue;
							end
						end
						solver = parthis.options{optidx}.Solver;
						areas_strict = unique(parthis.areafun_strict);
						areas_strict = strjoin(arrayfun(@char, areas_strict, 'UniformOutput', false), ', ');
						areas_loose = unique(parthis.areafun_loose);
						areas_loose = strjoin(arrayfun(@char, areas_loose, 'UniformOutput', false), ', ');
						temp = {i, char(solver), parthis.J_opt(optidx), length(parthis.systems), areas_strict, areas_loose};
						display(i, :) = temp;
						maxsolver(i, 1) = length(char(temp{1, 2}));
						maxJ(i, 1) = temp{1, 3};
						maxsystem(i, 1) = temp{1, 4};
						maxareas(i, 1) = length(temp{1, 5});
						controllers{i, 1} = parthis;
						parthis = []; %#ok<NASGU> prevent parfor warning because of temporary variables
					end
					maxsolver = max(maxsolver);
					maxJ = max(maxJ);
					maxsystem = max(maxsystem);
					maxArea = max(maxareas);
					formatstring1 = ['  % ', num2str(length(num2str(length(files))) + 1), 'd: solver=% ', num2str(maxsolver + 1), 's, J=% ', num2str(length(num2str(maxJ))), 'd, sys=% ', num2str(length(num2str(maxsystem))), 'd, A=%s, A_l=%s'];
					formatstring2 = ', Name=%s\n';
					for i = 1:length(files) %#ok<FORPF> no parfor since command line output
						[~, filename] = fileparts(files{i});
						fprintf([formatstring1, repmat(' ', 1, maxArea - maxareas(i, 1)), formatstring2], display{i, :}, filename);
					end
					rem = input('Load Controller (i):\n');
					if ~isempty(rem) && ((isnumeric(rem) && rem == floor(rem) && rem > 0 && rem <= length(files)))
						%load(files{rem});
						this = controllers{rem, 1};
						if nargout >= 2
							isloaded = true;
						end
					else
						this = [];
						if nargout >= 2
							isloaded = false;
						end
					end
				end
			end
		end
	end

	methods(Access=protected)
		function [name] = filename(this, noproblemsettings)
			%FILENAME filename for saving and loading solutions
			%	Input:
			%		this:				instance
			%		noproblemsettings:	indicator, if problem settings like systems and objective function parametrization should be included in filename
			%	Output:
			%		name:				filename
			if nargin <= 1
				noproblemsettings = false;
			end
			name = calculate_filename(this.controller, this.p, this.q, this.q_dot, this.r, this.systems, this.areafun_strict, this.areafun_loose, this.weight_strict, this.weight_loose, this.dimensions_strict, this.dimensions_loose, this.bounds, noproblemsettings);
			name = [name, '.mat'];
		end
	end

	methods
		function [this] = SolutionSet(controller, system, areafun, weight, R_fixed, R_bounds, reload)
			%SOLUTIONSET save optimal solution for specific controller and system order
			%	Input:
			%		controller:		controller type the solution was calculated for
			%		system:			systems to place poles for
			%		areafun:		desired pole area
			%		weight:			weight for different objective function parts
			%		R_fixed:		constraint on optimal solution
			%		R_bounds:		inequality constraints on optimal solution
			%		reload:			reload solution set from file
			%	Output:
			%		this:			instance
			if nargin <= 5
				R_bounds = [];
			end
			if nargin <= 6
				if islogical(R_bounds)
					reload = R_bounds;
					R_bounds = [];
				else
					reload = false;
				end
			end
			if ~islogical(reload) || ~isscalar(reload)
				error('control:design:gamma:solution:input', 'Reload indicator must be of type ''logical''.');
			end
			if nargin <= 4
				[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF();
			else
				[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF(R_fixed);
			end
			if nargin <= 5
				[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF();
			else
				[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF(R_bounds);
			end
			systemoptions = struct();
			[system, areafunstrict, areafunloose, weightstrict, weightloose, dimensionsstrict, dimensionsloose, ~, bound] = checkandtransformargs(system, areafun, weight, systemoptions, R_fixed, K_fixed, F_fixed, RKF_fixed, true, true, R_bounds, K_bounds, F_bounds, RKF_bounds);
			if ~isa(controller, 'control.design.outputfeedback.OutputFeedback')
				error('control:design:gamma:solution:input', 'Controller must be a subclass of OutputFeedback.');
			end
			this.controller = controller;
			this.p = size(system(1).B, 2);
			this.q = size(system(1).C, 1);
			this.q_dot = size(system(1).C_dot, 1);
			this.r = size(system(1).C_ref, 1);
			this.systems = system;
			this.areafun_strict = areafunstrict;
			this.weight_strict = weightstrict;
			this.dimensions_strict = dimensionsstrict;
			this.areafun_loose = areafunloose;
			this.weight_loose = weightloose;
			this.dimensions_loose = dimensionsloose;
			this.bounds = bound;
			this.R_fixed = R_fixed;
			this.K_fixed = K_fixed;
			this.F_fixed = F_fixed;
			this.RKF_fixed = RKF_fixed;
			this.R_bounds = R_bounds;
			this.K_bounds = K_bounds;
			this.F_bounds = F_bounds;
			this.RKF_bounds = RKF_bounds;
			this.R_0 = cell(0, 3);
			this.R_opt = zeros(this.p, this.q, 0);
			this.K_opt = zeros(this.p, this.q_dot, 0);
			this.F_opt = zeros(this.p, this.r, 0);
			if reload && exist(fullfile(this.SAVEPATH, this.filename()), 'file')
				load(fullfile(this.SAVEPATH, this.filename()));
			else
				this.clear();
			end
		end

		function [] = add_solution(this, R_opt, J_opt, info, R_0, options, objectiveoptions, comment)
			%ADD_SOLUTION add optimal solution to solution set
			%	Input:
			%		this:				instance
			%		R_opt:				optimal gain matrix
			%		J_opt:				optimal objective function value
			%		info:				output of optimization algorithm
			%		R_0:				initial value for optimization
			%		options:			options used for optimization
			%		objectiveoptions:	options used for objective
			%		comment:			comment to add to solution
			allowvarorder = false;
			if nargin >= 7
				if isstruct(objectiveoptions) && isfield(objectiveoptions, 'allowvarorder')
					allowvarorder = objectiveoptions.allowvarorder;
				end
			else
				objectiveoptions = struct();
			end
			if isnumeric(R_opt)
				K_opt_val = zeros(this.p, this.q_dot);
				F_opt_val = zeros(this.p, this.r);
			else
				[R_opt, K_opt_val, F_opt_val] = checkinitialRKF(R_opt, this.dimensions_strict, true);
			end
			if size(R_opt, 1) ~= size(this.R_opt, 1)
				error('control:design:gamma:solution:input', 'Optimal proportional gain must have %d rows.', size(this.R_opt, 1));
			end
			if size(R_opt, 2) ~= size(this.R_opt, 2)
				error('control:design:gamma:solution:input', 'Optimal proportional gain must have %d columns.', size(this.R_opt, 2));
			end
			if size(K_opt_val, 1) ~= size(this.K_opt, 1)
				error('control:design:gamma:solution:input', 'Optimal derivative gain must have %d rows.', size(this.K_opt, 1));
			end
			if size(K_opt_val, 2) ~= size(this.K_opt, 2)
				error('control:design:gamma:solution:input', 'Optimal derivative gain must have %d columns.', size(this.K_opt, 2));
			end
			if size(F_opt_val, 1) ~= size(this.F_opt, 1)
				error('control:design:gamma:solution:input', 'Optimal prefilter gain must have %d rows.', size(this.F_opt, 1));
			end
			if size(F_opt_val, 2) ~= size(this.F_opt, 2)
				error('control:design:gamma:solution:input', 'Optimal prefilter gain must have %d columns.', size(this.F_opt, 2));
			end
			if ~isnumeric(J_opt)
				error('control:design:gamma:solution:input', 'Optimal function value must be numeric.');
			end
			if ~isscalar(J_opt)
				error('control:design:gamma:solution:input', 'Optimal function value must be scalar.');
			end
			if ~isa(options, 'optimization.options.Options')
				error('control:design:gamma:solution:input', 'Options must be of type ''optimization.options.Options''.');
			end
			if ~isstruct(info)
				error('control:design:gamma:solution:input', 'Information must be of type ''struct''.');
			end
			if any(~isfield(info, fieldnames(optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE))) || any(~isfield(optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, fieldnames(info)))
				error('control:design:gamma:solution:input', 'Information must be of the same type as ''optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE''.');
			end
			if isa(R_0, 'control.design.gamma.InitialValue')
				initialoptions = getinitialoptions(struct(), objectiveoptions);
				if isempty(fieldnames(initialoptions))
					initialoptions = [];
				end
				R_0 = R_0.getall(initialoptions, this.systems, this.areafun_strict, this.weight_strict, {this.R_fixed, this.K_fixed, this.F_fixed, this.RKF_fixed}, {this.R_bounds, this.K_bounds, this.F_bounds, this.RKF_bounds}, []);
			end
			x_0 = checkinitialR(R_0, this.dimensions_strict);
			[strict, loose] = checkobjectiveoptions(objectiveoptions, x_0, double(allowvarorder), this.systems, options, this.dimensions_strict, this.dimensions_loose, this.areafun_strict, this.areafun_loose, this.weight_strict, this.weight_loose);
			[R_0_init, K_0_init, F_0_init] = checkinitialRKF(R_0, this.dimensions_strict);
			this.info(end + 1, 1) = info;% assign first in case of error for not matching structures
			this.R_opt(:, :, end + 1) = R_opt;
			this.K_opt(:, :, end + 1) = K_opt_val;
			this.F_opt(:, :, end + 1) = F_opt_val;
			this.R_0(end + 1, :) = {R_0_init, K_0_init, F_0_init};
			this.J_opt(end + 1, 1) = J_opt;
			this.options{end + 1, 1} = options;
			this.objectiveoptions_strict(end + 1, 1) = strict;
			this.objectiveoptions_loose(end + 1, 1) = loose;
			if isempty(this.ids)
				this.ids(1, 1) = 1;
			else
				this.ids(end + 1, 1) = max(this.ids) + 1;
			end
			if nargin >= 8
				if ischar(comment)
					this.solutioncomment{end + 1, 1} = comment;
				else
					if iscellstr(comment)
						this.solutioncomment{end + 1, 1} = strjoin(comment, sprintf('\n'));
					else
						error('control:design:gamma:solution:input', 'Comment must be of type ''char''.');
					end
				end
			else
				this.solutioncomment{end + 1, 1} = '';
			end
		end

		function [idx] = id2idx(this, id)
			%ID2IDX return index for given ID
			%	Input:
			%		this:	instance
			%		id:		id to get index for
			%	Output:
			%		idx:	index for the corresponding ID
			if nargin <= 1
				error('control:design:gamma:solution:input', 'ID must be supplied.');
			end
			if isempty(id)
				error('control:design:gamma:solution:input', 'ID must be supplied.');
			end
			idxid = find(this.ids == id);
			if isempty(idxid) || numel(idxid) ~= numel(id)
				error('control:design:gamma:solution:input', 'ID %d does not exist in solution set.', id);
			end
			%if length(idxid) > 1
			%	error('control:design:gamma:solution:input', 'ID %d is multiply defined.', idx);
			%end
			idx = idxid;
		end

		function [id] = idx2id(this, idx)
			%IDX2ID return ID for given index
			%	Input:
			%		this:	instance
			%		idx:	index to get ID for
			%	Output:
			%		id:		ID for the corresponding index
			if nargin <= 1
				error('control:design:gamma:solution:input', 'Index must be supplied.');
			end
			if isempty(idx)
				error('control:design:gamma:solution:input', 'Index must be supplied.');
			end
			if ndims(idx) > 2 %#ok<ISMAT> compatibility with Octave
				error('control:design:gamma:solution:input', 'Index must be a row vector.');
			end
			if size(idx, 1) == 1 && size(idx, 2) > 1
				idx = idx.';
			end
			if size(idx, 2) > 1
				error('control:design:gamma:solution:input', 'Index must be a row vector.');
			end
			if size(idx, 1) > size(this.ids, 1) || any(idx > size(this.ids, 1))
				error('control:design:gamma:solution:input', 'Index must be smaller than number of IDs.');
			end
			id = this.ids(idx);
		end

		function [optidx, R_opt, J_opt, info, R_0, options, objectiveoptions, comment] = getbest(this, feasible, tolerance)
			%GETBEST return best solution for current SolutionSet
			%	Input:
			%		this:				instance
			%		feasible:			return only feasible solutions
			%		tolerance:			tolerance for feasibility
			%	Output:
			%		R_opt:				optimal gain matrix
			%		J_opt:				optimal objective function value
			%		info:				output of optimization algorithm
			%		system:				systems to place poles for
			%		areafun:			desired pole area
			%		weight:				weight for different objective function parts
			%		R_fixed:			constraint on optimal solution
			%		R_0:				initial value for optimization
			%		options:			options used for optimization
			%		objectiveoptions:	options used for objective
			%		comment:			comment set for the solution
			if nargin <= 1
				feasible = false;
			end
			if nargin <= 2
				tolerance = 0;
			end
			if ~islogical(feasible) || ~isscalar(feasible)
				error('control:design:gamma:solution:input', 'Feasibility indicator must be of type ''logical''.');
			end
			if feasible
				has = this.hasallpolesinarea(true, tolerance);
				if isempty(has) || all(~has)
					warning('control:design:gamma:solution:best', 'No feasible solution was found, returning solution with best objective value.');
					[J_opt, optidx] = min(this.J_opt);
				else
					J = this.J_opt;
					J(~has) = NaN;
					[J_opt, optidx] = min(J);
				end
			else
				[J_opt, optidx] = min(this.J_opt);
			end
			if nargout >= 2
				R_opt = {
					this.R_opt(:, :, optidx);
					this.K_opt(:, :, optidx);
					this.F_opt(:, :, optidx)
				};
				if nargout >= 4
					info = this.info(optidx, 1);
					if nargout >= 5
						R_0 = this.R_0{optidx, 1};
						if nargout >= 6
							options = this.options{optidx, 1};
							if nargout >= 7
								objectiveoptions = {this.objectiveoptions_strict(optidx, 1), this.objectiveoptions_loose(optidx, 1)};
								if nargout >= 8
									comment = this.solutioncomment{optidx, 1};
								end
							end
						end
					end
				end
			end
		end

		function [idx, R_opt, J_opt, info, R_0, options, objectiveoptions, comment, feasibilitymargin] = getfeasible(this, tolerance)
			%GETFEASIBLE return feasible solutions for current SolutionSet
			%	Input:
			%		this:				instance
			%		tolerance:			tolerance for feasibility
			%	Output:
			%		idx:				indices of feasible solutions
			%		R_opt:				optimal gain matrix
			%		J_opt:				optimal objective function value
			%		info:				output of optimization algorithm
			%		system:				systems to place poles for
			%		areafun:			desired pole area
			%		weight:				weight for different objective function parts
			%		R_fixed:			constraint on optimal solution
			%		R_0:				initial value for optimization
			%		options:			options used for optimization
			%		objectiveoptions:	options used for objective
			%		comment:			comment set for the solution
			%		feasibilitymargin:	distance to feasibility
			if nargin <= 1
				tolerance = 0;
			end
			if nargout >= 9
				[has, feasibilitymargin] = this.hasallpolesinarea(true, tolerance);
			else
				has = this.hasallpolesinarea(true, tolerance);
			end
			idx = (1:size(has, 1))';
			idx = idx(has);
			if nargout >= 2
				R_opt = {
					this.R_opt(:, :, has);
					this.K_opt(:, :, has);
					this.F_opt(:, :, has)
				};
				if nargout >= 3
					J_opt = this.J_opt(has, 1);
					if nargout >= 4
						info = this.info(has, 1);
						if nargout >= 5
							R_0 = this.R_0{has, 1};
							if nargout >= 6
								options = this.options{has, 1};
								if nargout >= 7
									objectiveoptions = {this.objectiveoptions_strict(has, 1), this.objectiveoptions_loose(has, 1)};
									if nargout >= 8
										comment = this.solutioncomment{has, 1};
									end
								end
							end
						end
					end
				end
			end
		end

		function [idx, R_opt, J_opt, info, R_0, options, objectiveoptions, comment, stabmargin] = getstable(this, T, tolerance)
			%GETSTABLE return stable solutions for current SolutionSet
			%	Input:
			%		this:				instance
			%		T:					sample time
			%		tolerance:			tolerance for feasibility
			%	Output:
			%		idx:				indices of feasible solutions
			%		R_opt:				optimal gain matrix
			%		J_opt:				optimal objective function value
			%		info:				output of optimization algorithm
			%		system:				systems to place poles for
			%		areafun:			desired pole area
			%		weight:				weight for different objective function parts
			%		R_fixed:			constraint on optimal solution
			%		R_0:				initial value for optimization
			%		options:			options used for optimization
			%		objectiveoptions:	options used for objective
			%		comment:			comment set for the solution
			%		stabmargin:			distance to stable region
			if nargin <= 1
				T = -1;
			end
			if nargin <= 2
				tolerance = 0;
			end
			if ~isscalar(T)
				error('control:design:gamma:solution:get', 'Sampling time must be scalar.');
			end
			if isempty(T)
				T = -1;
			end
			if ~isscalar(tolerance)
				error('control:design:gamma:solution:get', 'Tolerance must be scalar.');
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if tolerance < -1
					error('control:design:gamma:solution:get', 'Tolerance must be greater than -1.');
				end
				areafun = control.design.gamma.area.Circlesquare(1 + tolerance);
			else
				areafun = control.design.gamma.area.Imag(1, 0, tolerance);
			end
			system = this.systems;
			R = this.R_opt;
			if nargout >= 9
				[has, stabmargin] = control.design.gamma.hasallpolesinarea(system, R, areafun, false, 0);
			else
				has = control.design.gamma.hasallpolesinarea(system, R, areafun, false, 0);
			end
			idx = (1:size(has, 1))';
			idx = idx(has);
			if nargout >= 2
				R_opt = {
					this.R_opt(:, :, has);
					this.K_opt(:, :, has);
					this.F_opt(:, :, has)
				};
				if nargout >= 3
					J_opt = this.J_opt(has, 1);
					if nargout >= 4
						info = this.info(has, 1);
						if nargout >= 5
							R_0 = this.R_0{has, 1};
							if nargout >= 6
								options = this.options{has, 1};
								if nargout >= 7
									objectiveoptions = {this.objectiveoptions_strict(has, 1), this.objectiveoptions_loose(has, 1)};
									if nargout >= 8
										comment = this.solutioncomment{has, 1};
									end
								end
							end
						end
					end
				end
			end
		end

		function [R_opt, J_opt, info, R_0, options, objectiveoptions, comment] = get(this, idx, byid)
			%GET return solution for current SolutionSet
			%	Input:
			%		this:				instance
			%		idx:				solution to return
			%		byid:				return solution by id
			%	Output:
			%		R_opt:				optimal gain matrix
			%		J_opt:				optimal objective function value
			%		info:				output of optimization algorithm
			%		system:				systems to place poles for
			%		areafun:			desired pole area
			%		weight:				weight for different objective function parts
			%		R_fixed:			constraint on optimal solution
			%		R_0:				initial value for optimization
			%		options:			options used for optimization
			%		objectiveoptions:	options used for objective
			%		comment:			comment set for the solution
			if nargin <= 1 && size(this.ids, 1) == 1
				idx = 1;
			end
			if nargin <= 2
				byid = false;
			end
			if islogical(idx)
				id = (1:length(idx))';
				idx = id(idx);
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					if byid
						idx = this.ids(end, 1);
					else
						idx = size(this.ids, 1);
					end
				end
				if strcmpi(idx, 'all')
					idx = 1:size(this.ids, 1);
				end
			end
			if isempty(idx) || ~isnumeric(idx) || all(floor(idx) ~= ceil(idx))
				error('control:design:gamma:solution:input', 'Index must be a finite number.');
			end
			if byid
				idxid = find(this.ids == idx);
				if isempty(idxid)
					error('control:design:gamma:solution:input', 'ID %d does not exist in solution set.', idx);
				end
				%if length(idxid) > 1
				%	error('control:design:gamma:solution:input', 'ID %d is multiply defined.', idx);
				%end
				idx = idxid;
			end
			if any(idx > size(this.J_opt, 1))
				error('control:design:gamma:solution:input', 'Index must be smaller than number of solutions.');
			end
			optidx = idx;
			if isscalar(idx)
				J_opt = this.J_opt(optidx, 1);
				R_opt = {
					this.R_opt(:, :, optidx);
					this.K_opt(:, :, optidx);
					this.F_opt(:, :, optidx)
				};
				if nargout >= 3
					info = this.info(optidx, 1);
					if nargout >= 4
						R_0 = this.R_0{optidx, 1};
						if nargout >= 5
							options = this.options{optidx, 1};
							if nargout >= 6
								objectiveoptions = {this.objectiveoptions_strict(optidx, 1), this.objectiveoptions_loose(optidx, 1)};
								if nargout >= 7
									comment = this.solutioncomment{optidx, 1};
								end
							end
						end
					end
				end
			else
				J_opt = this.J_opt(optidx, 1);
				R_opt = {
					this.R_opt(:, :, optidx);
					this.K_opt(:, :, optidx);
					this.F_opt(:, :, optidx)
				};
				if nargout >= 3
					info = this.info(optidx, 1);
					if nargout >= 4
						R_0 = this.R_0(optidx, 1);
						if nargout >= 5
							options = this.options(optidx, 1);
							if nargout >= 6
								objectiveoptions = {this.objectiveoptions_strict(optidx, 1), this.objectiveoptions_loose(optidx, 1)};
								if nargout >= 7
									comment = this.solutioncomment(optidx, 1);
								end
							end
						end
					end
				end
			end
		end

		function [system, polearea, weight, R_fixed, R_0, options, objectiveoptions, R_bounds, R_opt] = getproblem(this, idx, byid)
			%GETPROBLEM return arguments needed for solving the problem used to get the solution with gammasyn
			%	Input:
			%		this:				instance
			%		idx:				indices of solutions to check
			%		byid:				indicator, if indices should be interpreted as ids
			%	Output:
			%		system:				systems used for calcualtion of the solution
			%		polearea:			pole area function used for calculation of the solution
			%		weight:				weighting matrix used for calculation of the solution
			%		R_fixed:			gain constraint system used for calculation of the solution
			%		options:			optimization options used for calculation of the solution
			%		objectiveoption:	objective options used for calculation of the solution
			%		R_bounds:			gain inequality constraint system used for calculation of the solution
			%		R_opt:				optimal gain matrix of the solution
			if nargin <= 1 && size(this.ids, 1) == 1
				idx = 1;
			end
			if nargin <= 2
				byid = false;
			end
			system = this.systems;
			R_fixed = {this.R_fixed, this.K_fixed, this.F_fixed, this.RKF_fixed};
			R_bounds = {this.R_bounds, this.K_bounds, this.F_bounds, this.RKF_bounds};
			weight = this.weight_strict;
			polearea = {control.design.gamma.area.GammaArea.fromfunctionstruct(this.dimensions_strict.area_parameters), control.design.gamma.area.GammaArea.fromfunctionstruct(this.dimensions_loose.area_parameters)};
			[R_opt, ~, ~, R_0, options, objectiveoptions] = this.get(idx, byid);
			if isequalstruct(objectiveoptions{1}, objectiveoptions{2})
				objectiveoptions = objectiveoptions{1};
			else
				objectiveoptions = objectiveoptions{2};
			end
		end

		function [] = clear(this, idx, byid)
			%CLEAR delete solutions
			%	Input:
			%		this:	instance
			%		idx:	indices of solutions to delete
			if nargin <= 1
				this.R_opt = NaN(this.p, this.q, 0);
				this.K_opt = NaN(this.p, this.q_dot, 0);
				this.F_opt = NaN(this.p, this.r, 0);
				this.R_0 = cell(0, 3);
				this.J_opt = NaN(0, 1);
				prototype = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
				f = fieldnames(prototype)';
				f{2,1} = {};
				this.info = struct(f{:});
				this.options = cell(0, 1);
				prototype = control.design.gamma.objectiveoptions_prototype();
				f = fieldnames(prototype)';
				f{2,1} = {};
				this.objectiveoptions_strict = struct(f{:});
				this.objectiveoptions_loose = struct(f{:});
				this.ids = zeros(0, 1);
				this.solutioncomment = cell(0, 1);
			else
				if nargin <= 2
					byid = false;
				end
				if byid
					if islogical(idx)
						temp = (1:length(idx))';
						idx = temp(idx);
					elseif ischar(idx)
						if any(strcmpi(idx, {'end', 'last'}))
							idx = this.ids(end, 1);
						elseif strcmpi(idx, 'all')
							idx = 1:size(this.ids, 1);
						else
							error('control:design:gamma:solution:clear', 'IDs of solutions to delete are invalid.');
						end
					else
						if any(idx <= 0) || any(idx > max(this.ids))
							error('control:design:gamma:solution:clear', 'IDs of solutions to delete are invalid.');
						end
						idx = ismember(this.ids, idx);
					end
				else
					if length(idx) > size(this.J_opt, 1)
						error('control:design:gamma:solution:clear', 'More solution indices to delete than available in current SolutionSet.');
					end
					if islogical(idx)
						temp = (1:length(idx))';
						idx = temp(idx);
					elseif ischar(idx)
						if any(strcmpi(idx, {'end', 'last'}))
							idx = size(this.ids, 1);
						else
							error('control:design:gamma:solution:clear', 'IDs of solutions to delete are invalid.');
						end
					else
						if any(idx <= 0) || any(idx > size(this.J_opt, 1))
							error('control:design:gamma:solution:clear', 'Indices of solutions to delete are invalid.');
						end
					end
				end
				if isempty(idx) || all(idx == 0)
					return;
				end
				this.R_opt(:, :, idx) = [];
				this.K_opt(:, :, idx) = [];
				this.F_opt(:, :, idx) = [];
				this.R_0(idx, :) = [];
				this.J_opt(idx, :) = [];
				this.info(idx, :) = [];
				this.options(idx, :) = [];
				this.objectiveoptions_strict(idx, :) = [];
				this.objectiveoptions_loose(idx, :) = [];
				this.ids(idx, :) = [];
				this.solutioncomment(idx, :) = [];
			end
		end

		function [has, areamargin] = hasallpolesinarea(this, strict, tolerance, idx, byid)
			%HASALLPOLESINAREA return if all poles of the solution are in the specified area
			%	Input:
			%		this:		instance
			%		strict:		indicator, if areafunctions of strict or loose problem formulation should be used
			%		tolerance:	tolerance for checking
			%		idx:		indices of solutions to check
			%		byid:		indicator, if indices should be interpreted as ids
			%	Output
			%		has:		indicator if all systems have all poles in the specified area
			%		areamargin:	distance to area
			if nargin <= 1
				strict = true;
			end
			if nargin <= 2
				tolerance = 0;
			end
			if nargin <= 4
				byid = false;
			end
			if nargin <= 3
				idx = (1:size(this.ids, 1))';
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					if byid
						idx = this.ids(end, 1);
					else
						idx = size(this.ids, 1);
					end
				end
				if strcmpi(idx, 'all')
					idx = 1:size(this.ids, 1);
					byid = false;
				end
			end
			if byid
				idxid = ismember(idx, this.ids);
				if isempty(idxid)
					error('control:design:gamma:solution:input', 'ID %d does not exist in solution set.', idx);
				end
				idx = idxid;
			end
			if any(idx <= 0) || any(idx > length(this.ids))
				error('control:design:gamma:solution:input', 'ID must be a positive integer smaller than number of solutions in current set.');
			end
			system = this.systems;
			if strict
				if isa(this.areafun_strict, 'GammaArea')
					areafun = control.design.gamma.area.GammaArea.fromfunctionstruct(this.dimensions_strict.area_parameters);
				else
					areafun = this.areafun_strict;
				end
			else
				if isa(this.areafun_loose, 'GammaArea')
					areafun = control.design.gamma.area.GammaArea.fromfunctionstruct(this.dimensions_loose.area_parameters);
				else
					areafun = this.areafun_loose;
				end
			end
			R = {this.R_opt(:, :, idx), this.K_opt(:, :, idx), this.F_opt(:, :, idx)};
			if nargout >= 2
				[has, areamargin] = control.design.gamma.hasallpolesinarea(system, R, areafun, false, tolerance);
			else
				has = control.design.gamma.hasallpolesinarea(system, R, areafun, false, tolerance);
			end
		end

		function [has, P, feasibility] = hasallpolesinareaLPV(this, strict, tolerance, idx, byid)
			%HASALLPOLESINAREALPV return if all poles of the solution are in the specified area whe the area is treate as LMI region and the multiple models are the vertices of a convex polyhedron
			%	Input:
			%		this:				instance
			%		strict:				indicator, if areafunctions of strict or loose problem formulation should be used or GammaArea to use as pole area
			%		tolerance:			tolerance for checking
			%		idx:				indices of solutions to check
			%		byid:				indicator, if indices should be interpreted as ids
			%	Output
			%		has:				indicator if all systems have all poles in the specified area i.e. the resulting LMI problem is feasible
			%		P:					lyapunov matrix as solution for the LMI problem
			%		feasibilitymargin:	primal and dual feasibility measures for LMI
			if nargin <= 1
				strict = true;
			end
			if nargin <= 2
				tolerance = 0;
			end
			if nargin <= 4
				byid = false;
			end
			if nargin <= 3
				idx = (1:size(this.ids, 1))';
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					if byid
						idx = this.ids(end, 1);
					else
						idx = size(this.ids, 1);
					end
				end
				if strcmpi(idx, 'all')
					idx = 1:size(this.ids, 1);
					byid = false;
				end
			end
			if byid
				idxid = ismember(idx, this.ids);
				if isempty(idxid)
					error('control:design:gamma:solution:input', 'ID %d does not exist in solution set.', idx);
				end
				idx = idxid;
			end
			if any(idx <= 0) || any(idx > length(this.ids))
				error('control:design:gamma:solution:input', 'ID must be a positive integer smaller than number of solutions in current set.');
			end
			if ~configuration.optimization.hasYALMIP()
				error('control:design:gamma:solution:input', 'YALMIP toolbox is needed to solve LPV pole area test, but was not found.');
			end
			system = this.systems;
			if islogical(strict)
				if strict
					if isa(this.areafun_strict, 'GammaArea')
						areafun = control.design.gamma.area.GammaArea.fromfunctionstruct(this.dimensions_strict.area_parameters);
					else
						areafun = this.areafun_strict;
					end
				else
					if isa(this.areafun_loose, 'GammaArea')
						areafun = control.design.gamma.area.GammaArea.fromfunctionstruct(this.dimensions_loose.area_parameters);
					else
						areafun = this.areafun_loose;
					end
				end
			else
				if isa(strict, 'control.design.gamma.area.GammaArea')
					areafun = strict;
				elseif control.design.gamma.area.GammaArea.isfunctionstruct(strict)
					areafun = control.design.gamma.area.GammaArea.fromfunctionstruct(strict);
				else
					error('control:design:gamma:solution:input', 'Area function must be of type ''logical'' for saved areas and of type ''control.design.gamma.area.GammaArea'' for user defined areas.');
				end
			end
			areafun = control.design.gamma.area.GammaArea.unique(areafun);
			[LMI_region_L, LMI_region_M, haslmiregion] = areafun.toLMIregion();
			if ~all(haslmiregion(:))
				warning('control:design:gamma:solution:input', 'Not all areas have a LMI representation.');
			end
			Ropt = this.R_opt(:, :, idx);
			lmifeasibility = NaN(size(Ropt, 3), 2);
			solved = false(size(Ropt, 3), 1);
			P = cell(size(Ropt, 3), 1);
			for hh = 1:size(Ropt, 3) %#ok<FORPF> no parfor because of use of sdpvars in optimization
				if any(any(isnan(Ropt(:, :, hh))))
					warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
					continue;
				end
				R = Ropt(:, :, hh);
				A = cat(3, system.A);
				B = cat(3, system.B);
				C = cat(3, system.C);
				Acl = A - mtimes3d(mtimes3d(B, R), C);
				% scale elements of Acl to prevent Polyhedron class from removing too narrow (small) vertices
				meanAcl = mean(Acl, 3);
				extremevals = cat(3, min(Acl, [], 3), max(Acl, [], 3));
				extremevals(:, :, 1) = extremevals(:, :, 1) - meanAcl;
				extremevals(:, :, 2) = extremevals(:, :, 2) - meanAcl;
				iseq = extremevals(:, :, 1) == extremevals(:, :, 2);
				for ii = 1:size(iseq, 1)
					for jj = 1:size(iseq, 2)
						if iseq(ii, jj)
							extremevals(ii, jj, 1) = -1;
							extremevals(ii, jj, 2) = 1;
						end
					end
				end
				%Acl = (Acl - meanAcl)./(extremevals(:, :, 2) - extremevals(:, :, 1));
				A_use_poly_reduce = NaN(size(A, 3), size(A, 1)*size(A, 2));
				for ii = 1:size(Acl, 3)
					A_use_poly_reduce(ii, :) = reshape((Acl(:, :, ii) - meanAcl)./(extremevals(:, :, 2) - extremevals(:, :, 1)), 1, size(A, 1)*size(A, 2));
				end
				% convert to Polyhedron to get minimal representation of vertices (i.e. remove inner points)
				polyreduce = Polyhedron(A_use_poly_reduce);
				A_use_poly = polyreduce.V;
				Acl = NaN(size(A, 1), size(A, 2), size(A_use_poly, 1));
				for ii = 1:size(A_use_poly, 1)
					Acl(:, :, ii) = reshape(A_use_poly(ii, :), size(A, 1), size(A, 2)).*(extremevals(:, :, 2) - extremevals(:, :, 1)) + meanAcl;
				end
				%Acl = Acl.*(extremevals(:, :, 2) - extremevals(:, :, 1)) + meanAcl;
				P_lyap = rolmipvar(size(Acl, 1), size(Acl, 2), 'P', 'sym', size(Acl, 3), 1);
				P{hh, 1} = P_lyap;
				if size(Acl, 3) == 1
					A_vertices = {Acl};
				else
					A_vertices = mat2cell(Acl, size(Acl, 1), size(Acl, 2), ones(size(Acl, 3), 1));
				end
				A_vertices = reshape(A_vertices, [], 1);
				A = rolmipvar(cat(2, A_vertices{:}), 'A', size(Acl, 3), 1);
				% set up LMI feasibility problem for area and systems
				%tic
				PA = P_lyap*A;
				if ~all(real(LMI_region_L(:)) == 0)
					lyapunov = kron(real(LMI_region_L), P_lyap) + kron(LMI_region_M, PA) + kron(LMI_region_M', PA');% < 0
				else
					lyapunov = kron(LMI_region_M, PA) + kron(LMI_region_M', PA');% < 0
				end
				LMIs = lyapunov <= -tolerance*eye(size(lyapunov, 1));
				%toc
				% solve LMI feasibility problem for parameter dependent system
				%tic
				result = optimize(LMIs, [], sdpsettings('verbose', 1, 'solver', 'mosek'));
				solved(hh, 1) = any(result.problem == [-2, 0, 3]) && ~any(result.problem == [1, 4, 6, 8, 12, 15, 19]);
				%toc
				% check solution
				[primal, dual] = checkset(LMIs);
				lmifeasibility(hh, :) = [max([min(primal(:), [], 'omitnan'), -Inf]), max([min(dual(:), [], 'omitnan'), -Inf])];
			end
			feasibility = lmifeasibility;
			has = all(feasibility >= -eps, 2) & solved;
		end

		function [info] = stepinfo(this, systemnom, T, idx, F)
			%STEPINFO get information about solution and the step response
			%	Input:
			%		this:		instance
			%		systemnom:	nominal system to get dimension from for calculation of prefilter matrix
			%		T:			sampling time for system
			%		idx:		index of solution in current SolutionSet
			%	Output:
			%		info:		structure with information about the solution and the step response
			epsilon = 0.05;
			if nargin <= 3 && size(this.ids, 1) == 1
				idx = 1;
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					idx = size(this.ids, 1);
				end
			end
			if ~isscalar(idx)
				error('control:design:gamma:solution:plot', 'Index to plot must be scalar.');
			end
			if ~isscalar(T)
				error('control:design:gamma:solution:plot', 'Sampling time must be scalar.');
			end
			if isempty(T)
				T = -1;
			end
			[Ropt, Jopt, optiminfo, R0, solveroptions, ~, solcomment] = this.get(idx, false);
			if (~isempty(Ropt{1}) && any(isnan(Ropt{1}(:)))) || (~isempty(Ropt{2}) && any(isnan(Ropt{2}(:)))) || (~isempty(Ropt{3}) && any(isnan(Ropt{3}(:))))
				warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
				return;
			end
			hasstrict = this.hasallpolesinarea(true, solveroptions.ConstraintTolerance, idx, false);
			hasloose = this.hasallpolesinarea(false, solveroptions.ConstraintTolerance, idx, false);
			if nargin <= 4
				if ~isempty(Ropt{3}) && ~all(Ropt{3}(:) == 0)
					F = Ropt{3};
				else
					F = this.controller.prefilterpattern(Ropt, systemnom, T);
					if isa(this.controller, 'control.design.outputfeedback.PIDDirectOutputFeedback') || isa(this.controller, 'control.design.outputfeedback.PIDRealDirectOutputFeedback')
						F(1) = F(1) + Ropt{1}(1, 2);
					elseif isa(this.controller, 'control.design.outputfeedback.PDDirectOutputFeedback') || isa(this.controller, 'control.design.outputfeedback.PDRealDirectOutputFeedback')
						F(1) = F(1) + Ropt{1}(1, 1);
					end
				end
			else
				if size(F, 1) ~= size(Ropt{1}, 1)
					error('control:design:gamma:solution:plot', 'Prefilter matrix must have %d rows.', size(Ropt{1}, 1));
				end
			end
			Ropt = Ropt{1};
			isdiscrete = control.design.outputfeedback.OutputFeedback.isdiscreteT(T);
			if isdiscrete
				t = (0:T:5).';
				if size(F, 2) == 2
					w = [
						ones(length(t), 1),	[
							zeros(1, size(F, 2) > 1);
							ones(length(t) - 1, size(F, 2) > 1)
						]
					];
				else
					w = [
						ones(length(t), 1),	[
							zeros(1, size(F, 2) - 1);
							ones(length(t) - 1, size(F, 2) - 1)
						]
					];
				end
			else
				t = linspace(0, 5, 200);
				if size(F, 2) == 2
					w = [
						ones(length(t), 1),	[
							ones(1, size(F, 2) > 1);
							zeros(length(t) - 1, size(F, 2) > 1)
						]
					];
				else
					w = [
						ones(length(t), 1),	[
							ones(1, size(F, 2) - 1);
							zeros(length(t) - 1, size(F, 2) - 1)
						]
					];
				end
			end
			system = this.systems;
			for hh = 1:size(Ropt, 3) %#ok<FORPF> no parfor because of inner parfor
				if any(any(isnan(Ropt(:, :, hh))))
					warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
					continue;
				end
				ise = NaN(length(system), 1);% integral square error
				iae = NaN(size(ise));% integral area error
				itse = NaN(length(ise));% integral time square error
				itae = NaN(size(ise));% integral time area error
				isp = NaN(size(ise));% integral square power
				stat = NaN(size(ise));
				t_settling = NaN(size(ise));
				t_rising = NaN(size(ise));
				overshoot = NaN(size(ise));
				allstable = false(size(ise));
				R = Ropt(:, :, hh);
				optiminfohh = optiminfo(hh, 1);
				Jopthh = Jopt(hh, 1);
				R0hh = R0(:, :, hh);
				if isempty(solcomment)
					solcommenthh = '';
				elseif ischar(solcomment)
					solcommenthh = solcomment;
				else
					solcommenthh = solcomment{hh, 1};
				end
				parfor ii = 1:length(system)
					A = system(ii).A;
					B = system(ii).B;
					C = system(ii).C;
					D = system(ii).D;

					Acl = A - B*R*C;
					lambda = eig(Acl);
					if isdiscrete
						stable = ~any(abs(lambda) - 1 > 0);
					else
						stable = ~any(real(lambda) > 0);
					end
					allstable(ii, 1) = stable;
					Bcl = B*F;

					Gcl = ss(Acl, Bcl, [
						C - D*R*C;
						-R*C
					], [
						D*F;
						F
					], T);
					y = lsim(Gcl, w, t);
					isp(ii, 1) = trapz(t, y(:, size(C, 1) + 1)).^2/t(end);
					iae(ii, 1) = trapz(t, abs((y(:, 1) - w(:, 1))))/t(end);
					ise(ii, 1) = trapz(t, (y(:, 1) - w(:, 1)).^2)/t(end);
					itae(ii, 1) = trapz(t, t.*abs((y(:, 1) - w(:, 1))))/t(end)^2;
					itse(ii, 1) = trapz(t, t.*(y(:, 1) - w(:, 1)).^2)/t(end)^2;
					stat(ii, 1) = y(end, 1); %#ok<PFOUS> stationary value is only used if the system is stable
					overshoot(ii, 1) = max(y(:, 1)) - 1; %#ok<PFOUS> overshoot value is only used if the system is stable
					if ~stable
						idxepsilon = find(y(:, 1) >= (1 - epsilon), 1, 'first');
						if ~isempty(idxepsilon)
							t_rising(ii, 1) = t(idxepsilon, 1);
						else
							t_rising(ii, 1) = Inf;
						end
						t_settling(ii, 1) = NaN; %#ok<PFOUS> settling time value is only used if the system is stable
					else
						res = stepinfo(y(:, 1), t, y(end, 1), 'SettlingTimeThreshold', epsilon, 'RiseTimeLimits', [0, 1 - epsilon]);
						t_settling(ii, 1) = res.SettlingTime;
						t_rising(ii, 1) = res.RiseTime;
						%idxepsilon = find(y(:, 1) > (1 + epsilon) | y(:, 1) < (1 - epsilon), 1, 'last');
						%if ~isempty(idxepsilon)
						%	t_settling(ii, 1) = t(idxepsilon, 1);
						%else
						%	t_settling(ii, 1) = Inf;
						%end
					end
				end
				stat(~allstable) = [];
				overshoot(~allstable) = [];
				%ise(~allstable) = [];
				t_settling(~allstable) = [];
				%t_rising(~allstable) = [];
				stat(isnan(stat)) = [];
				overshoot(isnan(overshoot)) = [];
				ise(isnan(ise)) = [];
				iae(isnan(iae)) = [];
				itse(isnan(itse)) = [];
				itae(isnan(itae)) = [];
				isp(isnan(isp)) = [];
				t_settling(isnan(t_settling)) = [];
				t_rising(isnan(t_rising)) = [];
				if all(isnan(stat(:)))
					info_stat = NaN(1, 4);
				else
					info_stat = [mean(stat), min(stat), max(stat), var(stat)];
				end
				if all(isnan(overshoot(:)))
					info_overshoot = NaN(1, 4);
				else
					info_overshoot = [mean(overshoot), min(overshoot), max(overshoot), var(overshoot)];
				end
				if all(isnan(isp(:)))
					info_isp = NaN(1, 4);
				else
					info_isp = [mean(isp), min(isp), max(isp), var(isp)];
				end
				if all(isnan(iae(:)))
					info_iae = NaN(1, 4);
				else
					info_iae = [mean(iae), min(iae), max(iae), var(iae)];
				end
				if all(isnan(ise(:)))
					info_ise= NaN(1, 4);
				else
					info_ise = [mean(ise), min(ise), max(ise), var(ise)];
				end
				if all(isnan(itae(:)))
					info_itae = NaN(1, 4);
				else
					info_itae = [mean(itae), min(itae), max(itae), var(itae)];
				end
				if all(isnan(itse(:)))
					info_itse = NaN(1, 4);
				else
					info_itse = [mean(itse), min(itse), max(itse), var(itse)];
				end
				if all(isnan(t_settling(:)))
					info_t_settling = NaN(1, 4);
				else
					info_t_settling = [mean(t_settling), min(t_settling), max(t_settling), var(t_settling)];
				end
				if all(isnan(t_rising(:)))
					info_t_rising = NaN(1, 4);
				else
					info_t_rising = [mean(t_rising), min(t_rising), max(t_rising), var(t_rising)];
				end
				info(hh, 1) = struct(...
					'stat',					info_stat,...
					'overshoot',			info_overshoot,...
					'isp',					info_isp,...
					'iae',					info_iae,...
					'ise',					info_ise,...
					'itae',					info_itae,...
					'itse',					info_itse,...
					't_settling',			info_t_settling,...
					't_rising',				info_t_rising,...
					'info',					optiminfohh,...
					'stable',				all(allstable(:)),...
					'strictpolesinarea',	hasstrict(hh, 1),...
					'loosepolesinarea',		hasloose(hh, 1),...
					'R',					R,...
					'R_0',					R0hh,...
					'J',					Jopthh,...
					'comment',				solcommenthh...
				);
			end
		end

		function [z, p, k] = zpkdata(this, systemnom, T, idx)
			%ZPKDATA get information about zeros, poles and gains of the solution
			%	Input:
			%		this:		instance
			%		systemnom:	nominal system to get dimension from for calculation of prefilter matrix
			%		T:			sampling time for system
			%		idx:		index of solution in current SolutionSet
			%	Output:
			%		z:			zeros of all systems used for calculating the solution
			%		p:			poles of all systems used for calculating the solution
			%		k:			gain of all systems used for calculating the solution
			if nargin <= 3 && size(this.ids, 1) == 1
				idx = 1;
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					idx = size(this.ids, 1);
				end
			end
			if ~isscalar(idx)
				error('control:design:gamma:solution:plot', 'Index to plot must be scalar.');
			end
			if ~isscalar(T)
				error('control:design:gamma:solution:plot', 'Sampling time must be scalar.');
			end
			if isempty(T)
				T = -1;
			end
			Ropt = this.get(idx, false);
			if (~isempty(Ropt{1}) && any(isnan(Ropt{1}(:)))) || (~isempty(Ropt{2}) && any(isnan(Ropt{2}(:)))) || (~isempty(Ropt{3}) && any(isnan(Ropt{3}(:))))
				warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
				return;
			end
			if ~isempty(Ropt{3}) && ~all(Ropt{3}(:) == 0)
				F = Ropt{3};
			else
				F = this.controller.prefilterpattern(Ropt, systemnom, T);
				if isa(this.controller, 'control.design.outputfeedback.PIDDirectOutputFeedback') || isa(this.controller, 'control.design.outputfeedback.PIDRealDirectOutputFeedback')
					F(1) = F(1) + Ropt{1}(1, 2);
				elseif isa(this.controller, 'control.design.outputfeedback.PDDirectOutputFeedback') || isa(this.controller, 'control.design.outputfeedback.PDRealDirectOutputFeedback')
					F(1) = F(1) + Ropt{1}(1, 1);
				end
			end
			Ropt = Ropt{1};
			system = this.systems;
			dimn = zeros(length(system), 1);
			dimp = size(Ropt, 1);
			dimq = size(Ropt, 2);
			parfor ii = 1:length(system)
				dimn(ii, 1) = size(system(ii).A, 1);
			end
			dimn = max(dimn(:));
			z = NaN(size(Ropt, 3), length(system), dimq + dimp, size(F, 2), dimn) + 1i*NaN;
			p = NaN(size(Ropt, 3), length(system), dimq + dimp, size(F, 2), dimn) + 1i*NaN;
			k = NaN(size(Ropt, 3), length(system), dimq + dimp, size(F, 2));
			for hh = 1:size(Ropt, 3) %#ok<FORPF> no parfor because of inner parfor
				if any(any(isnan(Ropt(:, :, hh))))
					warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
					continue;
				end
				R = Ropt(:, :, hh);
				for ii = 1:length(system)
					A = system(ii).A;
					B = system(ii).B;
					C = system(ii).C;
					D = system(ii).D;

					Acl = A - B*R*C;
					Bcl = B*F;

					Gcl = ss(Acl, Bcl, [
						C - D*R*C;
						-R*C
					], [
						D*F;
						F
					], T);
					[zero, pole, gain] = zpkdata(Gcl);
					ztemp = z(hh, ii, :, :, :);
					ptemp = p(hh, ii, :, :, :);
					for jj = 1:dimq + dimp
						for kk = 1:size(F, 2)
							ztemp(1, 1, jj, kk, :) = [
								zero{jj, kk};
								NaN(dimn - size(zero{jj, kk}, 1), 1) + 1i*NaN
							];
							ptemp(1, 1, jj, kk, :) = [
								pole{jj, kk};
								NaN(dimn - size(pole{jj, kk}, 1), 1) + 1i*NaN
							];
						end
					end
					z(hh, ii, :, :, :) = ztemp;
					p(hh, ii, :, :, :) = ptemp;
					k(hh, ii, :, :) = gain;
				end
			end
		end

		function [R_opt, J_opt, info] = rerun(this, idx, R_0, solver)
			%RERUN rerun optimization for selected solution
			%	Input:
			%		this:	instance
			%		idx:	index of solution to run
			%		R_0:	inital value to use for rerun
			%		solver:	solver to use for rerun
			%	Output:
			%		R_opt:	optimal gain matrix
			%		J_opt:	optimal objective function value
			%		info:	output of optimization algorithm
			if nargin <= 1 && size(this.ids, 1) == 1
				idx = 1;
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					idx = size(this.ids, 1);
				end
			end
			if isempty(idx) || ~isscalar(idx)
				error('control:design:gamma:solution:input', 'Index must be a scalar finite number.');
			end
			if nargin <= 2
				[system, polearea, weight, Rfixed, R_0, solveroptions, objectiveoptions, Rbounds] = this.getproblem(idx, false);
			else
				[system, polearea, weight, Rfixed, ~, solveroptions, objectiveoptions, Rbounds] = this.getproblem(idx, false);
				if size(R_0, 1) ~= this.dimensions_strict.measurements
					error('control:design:gamma:solution:input', 'Initial gain must be a %dX%d matrix, not %dX%d.', this.dimensions_strict.controls, this.dimensions_strict.measurements, size(R_0, 1), size(R_0, 2));
				end
				if size(R_0, 2) ~= this.dimensions_strict.controls
					error('control:design:gamma:solution:input', 'Initial gain must be a %dX%d matrix, not %dX%d.', this.dimensions_strict.controls, this.dimensions_strict.measurements, size(R_0, 1), size(R_0, 2));
				end
			end
			if nargin >= 3
				if isa(solver, 'optimization.solver.Optimizer')
					solveroptions = optimization.options.OptionFactory.instance.options(solver, solveroptions);
				elseif isa(solver, 'optimization.options.Options')
					solveroptions = optimization.options.OptionFactory.instance.options(solveroptions, solver);
				end
			end
			[R_opt, J_opt, info] = control.design.gamma.gammasyn(system, polearea, weight, Rfixed, R_0, solveroptions, objectiveoptions, Rbounds);
		end

		function [figures] = plot(this, idx, systems, cursorfun)
			%PLOT plot a solution for the systems used in the optimization and the additional supplied systems
			%	Input:
			%		this:		instance
			%		idx:		index of solution in current SolutionSet
			%		systems:	additional systems to plot eigenvalues for
			%		cursorfun:	function pointer to use as data tip cursor function and transformation function to transform/add to set UserData
			%	Output:
			%		figures:	array with handles to figures for different plots
			if nargin <= 1 && size(this.ids, 1) == 1
				idx = 1;
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					idx = size(this.ids, 1);
				end
				if strcmpi(idx, 'all')
					idx = 1:size(this.ids, 1);
				end
			end
			if nargin <= 3
				cursorfun = {@control.design.gamma.cursordata, []};
			end
			if isfunctionhandle(cursorfun)
				datatipfun = cursorfun;
				datafun = [];
			elseif iscell(cursorfun)
				if numel(cursorfun) >= 2
					if isfunctionhandle(cursorfun{1})
						datatipfun = cursorfun{1};
					else
						error('control:design:gamma:solution:plot', 'Datatip function must be a function handle.');
					end
					if isfunctionhandle(cursorfun{2})
						datafun = cursorfun{2};
					elseif isempty(cursorfun{2})
						datafun = [];
					else
						error('control:design:gamma:solution:plot', 'Data transformation function must be a function handle.');
					end
				else
					if isfunctionhandle(cursorfun{1})
						datatipfun = cursorfun{1};
						datafun = [];
					else
						error('control:design:gamma:solution:plot', 'Datatip function must be a function handle.');
					end
				end
			else
				error('control:design:gamma:solution:plot', 'Datatip function must be a function handle or a cell array of function handles.');
			end
			if isempty(datatipfun) || ~isfunctionhandle(datatipfun)
				error('control:design:gamma:solution:plot', 'Datatip function must be a function handle.');
			end
			if nargin(datatipfun) < 2
				error('control:design:gamma:solution:plot', 'Datatip function must be a function handle with at least 2 input arguments.');
			end
			if ~isempty(datafun) && ~isfunctionhandle(datafun)
				error('control:design:gamma:solution:plot', 'Data transformation function must be a function handle.');
			end
			if ~isempty(datafun)
				if nargin(datafun) < 2
					error('control:design:gamma:solution:plot', 'Data transformation function must be a function handle with at least 2 input arguments.');
				end
			end
			%if ~isscalar(idx)
			%	error('control:design:gamma:solution:plot', 'Index to plot must be scalar.');
			%end
			[Ropt, ~, ~, ~, ~, ~, solcomment] = this.get(idx, false);
			if (~isempty(Ropt{1}) && all(isnan(Ropt{1}(:)))) || (~isempty(Ropt{2}) && all(isnan(Ropt{2}(:)))) || (~isempty(Ropt{3}) && all(isnan(Ropt{3}(:))))
				warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
				return;
			end
			areafuns_strict = control.design.gamma.area.GammaArea.unique(this.dimensions_strict.area_parameters);
			areafuns_loose = control.design.gamma.area.GammaArea.unique(this.dimensions_loose.area_parameters);
			figurehandle = cell(size(Ropt{1}, 3), 1);
			for hh = 1:size(Ropt{1}, 3) %#ok<FORPF> no parfor for figure operations
				if any(any(isnan(Ropt{1}(:, :, hh))))
					warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
					continue;
				end
				figurehandle{hh, 1} = figure('Name', ['ID:', num2str(this.ids(idx(hh), 1))]);
				if ischar(solcomment) && ~isempty(solcomment)
					title(escape_latex(solcomment));
				elseif iscell(solcomment) && ~isempty(solcomment{hh, 1})
					title(escape_latex(solcomment{hh, 1}));
				end
				hold('all');
				if nargin >= 3
					A = cat(3, systems.A);
					B = cat(3, systems.B);
					C = cat(3, systems.C);
					eigstest = eig3d(A - mtimes3d(mtimes3d(B, Ropt{1}(:, :, hh)), C), 'vector');
					systemcolor = gray(size(eigstest, 2));
					for ii = 1:size(eigstest, 1)
						handle = scatter(real(eigstest(ii, :)), imag(eigstest(ii, :)), [], systemcolor(:, 1:3), '*');
						set(handle, 'UserData', (1:size(systems, 1))');
					end
					if ~isempty(datafun)
						[systhis, sys] = feval(datafun, this.systems, systems);
						if ~isstruct(systhis) || ~isfield(systhis, 'systems')
							error('control:design:gamma:solution:plot', 'Data transformation function must return a structure with field ''systems''.');
						end
						if ~isstruct(sys) || ~isfield(sys, 'systems')
							error('control:design:gamma:solution:plot', 'Data transformation function must return a structure with field ''systems''.');
						end
						set(get(handle, 'Parent'), 'UserData', {sys, systhis});
					else
						set(get(handle, 'Parent'), 'UserData', {struct(...
							'systems',		systems...
						), struct(...
							'systems',		this.systems...
						)});
					end
					box = [
						min(real(eigstest(:))), max(real(eigstest(:))),	1i*min(imag(eigstest(:))), 1i*max(imag(eigstest(:)))
					];
				else
					box = zeros(0, 4);
				end
				cursorMode = datacursormode(figurehandle{hh, 1});
				set(cursorMode, 'UpdateFcn', datatipfun, 'Enable', 'on');
				systems = this.systems;
				colors = gray(length(systems));
				if iscell(systems)
					A = ssdata(systems{1});
					numbereig = size(A);
				elseif isstruct(systems)
					numbereig = size(systems(1).A, 1);
				else
					numbereig = 0;
				end
				eigssys = (1 + 1i)*NaN(numbereig, length(systems));
				for ii = 1:length(systems)
					A = systems(ii).A;
					B = systems(ii).B;
					C = systems(ii).C;
					szA = size(A, 1);
					eigssys(:, ii) = [
						eig(A - B*Ropt{1}(:, :, hh)*C);
						(1 + 1i)*NaN(size(eigssys, 1) - szA)
					];
				end
				handle = plot(real(eigssys), imag(eigssys), 'd', 'MarkerSize', 10, 'MarkerEdgeColor', 'k');
				set(handle, {'UserData'}, num2cell((1:size(systems, 1))'), {'MarkerFaceColor'}, mat2cell(colors(:, 1:3), ones(size(colors, 1), 1), 3));
				handle = plot(real(eigssys), imag(eigssys), '.', 'MarkerSize', 7, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0, 0, 0]);
				set(handle, {'UserData'}, num2cell((1:size(systems, 1))'));
				if nargin < 3
					if ~isempty(datafun)
						systhis = datafun(this.systems);
						if ~isstruct(systhis) || ~isfield(systhis, 'systems')
							error('control:design:gamma:solution:plot', 'Data transformation function must return a structure with field ''systems''.');
						end
						set(get(handle, 'Parent'), 'UserData', {[], systhis});
					else
						set(get(handle(1), 'Parent'), 'UserData', {[], struct(...
							'systems',		this.systems...
						)});
					end
				end
				if ~isempty(areafuns_strict)
					border = cat(1, areafuns_strict.plotborder([
						box;
						-1,	1,	-1i,	1i;
						min(real(eigssys(:))), max(real(eigssys(:))),	1i*min(imag(eigssys(:))), 1i*max(imag(eigssys(:)))
					], 500));
					plot(real(border).', imag(border).', 'r', 'LineWidth', 2);
				end
				if ~isempty(areafuns_loose)
					border = cat(1, areafuns_loose.plotborder([
						box;
						-1,	1,	-1i,	1i;
						min(real(eigssys(:))), max(real(eigssys(:))),	1i*min(imag(eigssys(:))), 1i*max(imag(eigssys(:)))
					], 500));
					plot(real(border).', imag(border).', 'b', 'LineWidth', 2);
				end
				grid('on');
				% TODO: plot only for discrete systems
				plot.zgrid();
				hold('off');
			end
			if nargout >= 1
				figures = [figurehandle{:}];
			end
		end

		function [figures] = plotexport(this, idx, plotareafun, plotgrid)
			%PLOTEXPORT plot a solution for the systems used in the optimization for exporting to tikz
			%	Input:
			%		this:			instance
			%		idx:			index of solution in current SolutionSet
			%		plotareafun:	plot borders of area functions
			%		plotgrid:		plot grid lines
			if nargin <= 2
				plotareafun = false;
			end
			if nargin <= 3
				plotgrid = false;
			end
			if nargin <= 1 && size(this.ids, 1) == 1
				idx = 1;
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					idx = size(this.ids, 1);
				end
				if strcmpi(idx, 'all')
					idx = 1:size(this.ids, 1);
				end
			end
			%if ~isscalar(idx)
			%	error('control:design:gamma:solution:plot', 'Index to plot must be scalar.');
			%end
			Ropt = this.get(idx, false);
			if (~isempty(Ropt{1}) && all(isnan(Ropt{1}(:)))) || (~isempty(Ropt{2}) && all(isnan(Ropt{2}(:)))) || (~isempty(Ropt{3}) && all(isnan(Ropt{3}(:))))
				warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
				return;
			end
			areafuns_strict = control.design.gamma.area.GammaArea.unique(this.dimensions_strict.area_parameters);
			areafuns_loose = control.design.gamma.area.GammaArea.unique(this.dimensions_loose.area_parameters);
			system = this.systems;
			Ropt = Ropt{1};
			figurehandle = cell(size(Ropt, 3), 1);
			for hh = 1:size(Ropt, 3) %#ok<FORPF> no parfor for figure operations
				if any(any(isnan(Ropt(:, :, hh))))
					warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
					continue;
				end
				figurehandle{hh, 1} = figure;
				hold('all');
				A = cat(3, system.A);
				B = cat(3, system.B);
				C = cat(3, system.C);
				eigstest = eig3d(A - mtimes3d(mtimes3d(B, Ropt(:, :, hh)), C), 'vector');
				systemcolor = gray(size(eigstest, 2));
				for ii = 1:size(eigstest, 1)
					scatter(real(eigstest(ii, :)), imag(eigstest(ii, :)), [], systemcolor(:, 1:3), '*');
				end
				box = [
					min(real(eigstest(:))), max(real(eigstest(:))),	1i*min(imag(eigstest(:))), 1i*max(imag(eigstest(:)))
				];
				if plotareafun
					if ~isempty(areafuns_strict)
						border = cat(1, areafuns_strict.plotborder([
							box;
							-1,	1,	-1i,	1i
						], 500));
						plot(real(border).', imag(border).', 'r');
					end
					if ~isempty(areafuns_loose)
						border = cat(1, areafuns_loose.plotborder([
							box;
							-1,	1,	-1i,	1i
						], 500));
						plot(real(border).', imag(border).', 'b');
					end
				end
				grid('on');
				if plotgrid
					plot.zgrid();
				end
				hold('off');
			end
			if nargout >= 1
				figures = [figurehandle{:}];
			end
		end

		function [figurehandle] = step(this, systemnom, T, idx, systems, F, plotvalues, precision)
			%STEP plot the step responses of a solution for the systems used in the optimization and the additional supplied systems
			%	Input:
			%		this:			instance
			%		systemnom:		nominal system to get dimension from for calculation of prefilter matrix
			%		T:				sampling time for system
			%		idx:			index of solution in current SolutionSet
			%		systems:		additional systems to plot eigenvalues for
			%		F:				prefilter matrix to use for simulation
			%		plotvalues:		cell array with indicator array for selecting measurements and controls for plotting
			%		precision:		precision the values are plotted in (Inf for no change)
			%	Output:
			%		figurehandle:	handle to figure
			if nargin <= 3 && size(this.ids, 1) == 1
				idx = 1;
			end
			if nargin <= 7
				precision = Inf;
			end
			if ischar(idx)
				if any(strcmpi(idx, {'end', 'last'}))
					idx = size(this.ids, 1);
				end
			end
			if ~isscalar(idx)
				error('control:design:gamma:solution:plot', 'Index to plot must be scalar.');
			end
			if isempty(T)
				T = -1;
			end
			if ~isscalar(T)
				error('control:design:gamma:solution:plot', 'Sampling time must be scalar.');
			end
			[Ropt, ~, ~, ~, ~, ~, solcomment] = this.get(idx, false);
			if (~isempty(Ropt{1}) && any(isnan(Ropt{1}(:)))) || (~isempty(Ropt{2}) && any(isnan(Ropt{2}(:)))) || (~isempty(Ropt{3}) && any(isnan(Ropt{3}(:))))
				warning('control:design:gamma:solution:plot', 'No valid solution could be found.');
				return;
			end
			number_controls = size(Ropt{1}, 1);
			number_measurements = size(Ropt{1}, 2);
			number_references = size(Ropt{3}, 2);
			isdiscrete = control.design.outputfeedback.OutputFeedback.isdiscreteT(T);
			if nargin <= 4
				plotadditional = false;
				calculateprefilter = true;
				plotvalues = {true(number_measurements, 1), true(number_controls, 1), true(number_references, 1)};
			else
				if nargin <= 5
					if isnumeric(systems)
						plotadditional = false;
						F = systems;
						calculateprefilter = false;
						plotvalues = {true(number_measurements, 1), true(number_controls, 1), true(number_references, 1)};
					elseif iscell(systems)
						plotadditional = false;
						plotvalues = systems;
						calculateprefilter = true;
					elseif islogical(systems)
						plotadditional = false;
						if size(systems, 1) == number_measurements
							plotvalues = {systems, true(number_controls, 1), true(number_references, 1)};
						else
							plotvalues = {true(number_measurements, 1), systems, true(number_references, 1)};
						end
						calculateprefilter = true;
					else
						plotadditional = true;
						calculateprefilter = true;
						plotvalues = {true(number_measurements, 1), true(number_controls, 1), true(number_references, 1)};
					end
				elseif nargin <= 6
					if iscell(F)
						plotvalues = F;
						if isnumeric(systems)
							plotadditional = false;
							F = systems;
							calculateprefilter = false;
						else
							plotadditional = true;
							calculateprefilter = true;
						end
					elseif islogical(F)
						if size(systems, 1) == number_measurements
							plotvalues = {F, true(number_controls, 1), true(number_references, 1)};
						else
							plotvalues = {true(number_measurements, 1), F, true(number_references, 1)};
						end
						if isnumeric(systems)
							plotadditional = false;
							F = systems;
							calculateprefilter = false;
						else
							plotadditional = true;
							calculateprefilter = true;
						end
					else
						plotadditional = true;
						calculateprefilter = true;
						plotvalues = {true(number_measurements, 1), true(number_controls, 1), true(number_references, 1)};
					end
				else
					plotadditional = ~isempty(systems);
					if islogical(F)
						calculateprefilter = F;
					else
						calculateprefilter = false;
					end
				end
			end
			if islogical(plotvalues)
				if size(plotvalues, 1) == number_measurements
					plotvalues = {plotvalues, true(number_controls, 1)};
				else
					plotvalues = {true(number_measurements, 1), plotvalues};
				end
			end
			if ~iscell(plotvalues)
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicator must be a cell array or logical.');
			end
			if length(plotvalues) < 2
				error('control:design:gamma:solution:plot', '2 Measurement and control plot indicators must be supplied.');
			end
			if length(plotvalues) < 3
				plotvalues = [plotvalues, {true(number_references, 1)}];
			end
			if ~isnumeric(precision) || ~isscalar(precision)
				error('control:design:gamma:solution:plot', 'Precision must be a numerical scalar.');
			end
			plot_measurements1 = size(plotvalues{1}, 1) == number_measurements;
			plot_controls1 = size(plotvalues{1}, 1) == number_controls;
			plot_measurements2 = size(plotvalues{2}, 1) == number_measurements;
			plot_controls2 = size(plotvalues{2}, 1) == number_controls;
			if size(plotvalues{1}, 2) ~= 1 || size(plotvalues{2}, 2) ~= 1 || size(plotvalues{3}, 2) ~= 1
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicators must be column vectors.');
			end
			if ~plot_measurements1 && ~plot_controls1
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicator size must match number of measurements and controls.');
			end
			if ~plot_measurements2 && ~plot_controls2
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicator size must match number of measurements and controls.');
			end
			if number_measurements ~= number_controls && plot_measurements1 && plot_measurements2
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicator size must match number of measurements and controls.');
			end
			if number_measurements ~= number_controls && plot_controls1 && plot_controls2
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicator size must match number of measurements and controls.');
			end
			if size(plotvalues{3}, 1) ~= number_references
				error('control:design:gamma:solution:plot', 'Measurement and control plot indicator size must match number of measurements and controls.');
			end
			if plot_measurements1
				use_measurements = plotvalues{1};
				use_controls = plotvalues{2};
				use_references = plotvalues{3};
			else
				use_measurements = plotvalues{2};
				use_controls = plotvalues{1};
				use_references = plotvalues{3};
			end
			if ~any([
				use_measurements;
				use_references;
				use_controls
			])
				error('control:design:gamma:solution:plot', 'At least one value must be plotted.');
			end
			if calculateprefilter
				if ~isempty(Ropt{3}) && ~all(Ropt{3}(:) == 0)
					F = Ropt{3};
				else
					F = this.controller.prefilterpattern(Ropt, systemnom, T);
					if isa(this.controller, 'control.design.outputfeedback.PIDDirectOutputFeedback') || isa(this.controller, 'control.design.outputfeedback.PIDRealDirectOutputFeedback')
						F(1) = F(1) + Ropt{1}(1, 2);
					elseif isa(this.controller, 'control.design.outputfeedback.PDDirectOutputFeedback') || isa(this.controller, 'control.design.outputfeedback.PDRealDirectOutputFeedback')
						F(1) = F(1) + Ropt{1}(1, 1);
					end
				end
			else
				if ~isnumeric(F)
					error('control:design:gamma:solution:plot', 'Prefilter matrix must be numeric.');
				end
				if size(F, 1) ~= number_controls
					error('control:design:gamma:solution:plot', 'Prefilter matrix must have %d rows.', number_controls);
				end
			end
			Ropt = Ropt{1};
			if plotadditional
				if ~isstruct(systems)
					error('control:design:gamma:solution:plot', 'Systems must be of type ''struct''.');
				end
				if any(~isfield(systems, {'A', 'B', 'C', 'D'}))
					error('control:design:gamma:solution:plot', 'Systems must have fields ''A'', ''B'', ''C'' and ''D''.');
				end
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				t = (0:T:5).';
				if isa(this.controller, 'control.design.outputfeedback.AbstractCouplingFeedback')
					w = [
						ones(length(t), size(F, 2) - this.controller.number_couplingconditions),	zeros(length(t), this.controller.number_couplingconditions)
					];
				else
					if size(F, 2) == 2
						w = [
							ones(length(t), 1),	[
								zeros(1, size(F, 2) > 1);
								ones(length(t) - 1, size(F, 2) > 1)
							]
						];
					else
						w = [
							ones(length(t), 1),	[
								zeros(1, size(F, 2) - 1);
								ones(length(t) - 1, size(F, 2) - 1)
							]
						];
					end
				end
			else
				t = linspace(0, 5, 200).';
				if isa(this.controller, 'control.design.outputfeedback.AbstractCouplingFeedback')
					w = [
						ones(length(t), size(F, 2) - this.controller.number_couplingconditions),	zeros(length(t), this.controller.number_couplingconditions)
					];
				else
					if size(F, 2) == 2
						w = [
							ones(length(t), 1),	[
								ones(1, size(F, 2) > 1);
								zeros(length(t) - 1, size(F, 2) > 1)
							]
						];
					else
						w = [
							ones(length(t), 1),	[
								ones(1, size(F, 2) - 1);
								zeros(length(t) - 1, size(F, 2) - 1)
							]
						];
					end
				end
			end
			if nargout >= 1
				figurehandle = figure('Name', ['ID:', num2str(this.ids(idx, 1))]);
			else
				figure('Name', ['ID:', num2str(this.ids(idx, 1))]);
			end
			nplot = number_controls + number_references + number_measurements;
			plotdisplay = [
				use_measurements;
				use_references;
				use_controls
			];
			plotdisplayidx = cumsum(plotdisplay);
			nplotdisplay = sum(plotdisplay);
			if plotadditional
				systemcolor = jet(length(systems));
				y = NaN(size(t, 1), nplot, length(systems));
				parfor ii = 1:length(systems)
					A = systems(ii).A;
					B = systems(ii).B;
					C = systems(ii).C;
					D = systems(ii).D;
				if isfield(systems(ii), 'C_ref')
					C_ref = systems(ii).C_ref;
				else
					C_ref = zeros(0, size(A, 1));
				end
				if isfield(systems(ii), 'D_ref')
					D_ref = systems(ii).D_ref;
				else
					D_ref = zeros(0, size(B, 2));
				end

					Acl = A - B*Ropt*C;
					Bcl = B*F;
				Ccl = [
						C - D*Ropt*C;
					C_ref - D_ref*Ropt*C;
						-Ropt*C
				];
				Dcl = [
						D*F;
					D_ref*F;
						F
				];

				if isdiscrete
					Gcl = ss(Acl, Bcl, Ccl, Dcl, T);
				else
					Gcl = ss(Acl, Bcl, Ccl, Dcl);
				end
					%y(:, (1:nplot) + nplot*(ii - 1)) = lsim(Gcl, w, t);
					y(:, :, ii) = lsim(Gcl, w, t);
				end
				y = roundtopreciision(reshape(y, size(t, 1), nplot*length(systems)), precision);
				for jj = 1:nplot %#ok<FORPF> no parfor for figure operations
					if plotdisplay(jj, 1)
						subplothandle = subplot(nplotdisplay, 1, plotdisplayidx(jj, 1));
						% https://de.mathworks.com/matlabcentral/answers/19815-explicitly-specifying-line-colors-when-plotting-a-matrix
						set(subplothandle, 'ColorOrder', systemcolor, 'NextPlot', 'replacechildren');
						if isdiscrete
							stairs(t, y(:, jj:nplot:end));
						else
							plot(t, y(:, jj:nplot:end));
						end
						hold('all');
						grid('on');
						if jj <= number_measurements
							ylabel(['$y_{', num2str(jj), '}$']);
					elseif jj <= number_measurements + number_references
						ylabel(['$y_{ref, ', num2str(jj - number_measurements), '}$']);
						else
						ylabel(['$u_{', num2str(jj - number_measurements - number_references), '}$']);
						end
						axis('tight');
					end
				end
			end
			systems = this.systems;
			systemcolor = jet(length(systems));
			y = NaN(size(t, 1), nplot, length(systems));
			parfor ii = 1:length(systems)
				A = systems(ii).A;
				B = systems(ii).B;
				C = systems(ii).C;
				D = systems(ii).D;
					if isfield(systems(ii), 'C_ref')
						C_ref = systems(ii).C_ref;
					else
						C_ref = zeros(0, size(A, 1));
					end
					if isfield(systems(ii), 'D_ref')
						D_ref = systems(ii).D_ref;
					else
						D_ref = zeros(0, size(B, 2));
					end

				Acl = A - B*Ropt*C;
				Bcl = B*F;
					Ccl = [
					C - D*Ropt*C;
						C_ref - D_ref*Ropt*C;
					-Ropt*C
					];
					Dcl = [
					D*F;
						D_ref*F;
					F
					];

					if isdiscrete
						Gcl = ss(Acl, Bcl, Ccl, Dcl, T);
					else
						Gcl = ss(Acl, Bcl, Ccl, Dcl);
					end
				%y(:, (1:nplot) + nplot*(ii - 1)) = lsim(Gcl, w, t);
				y(:, :, ii) = lsim(Gcl, w, t);
			end
			y = roundtoprecision(reshape(y, size(t, 1), nplot*length(systems)), precision);
			subplothandles = zeros(nplotdisplay, 0);
			subplothandles(1, 1) = subplot(nplotdisplay, 1, 1);
			for jj = 1:nplot %#ok<FORPF> no parfor for figure operations
				if plotdisplay(jj, 1)
					subplothandles(plotdisplayidx(jj, 1), 1) = subplot(nplotdisplay, 1, plotdisplayidx(jj, 1));
					% https://de.mathworks.com/matlabcentral/answers/19815-explicitly-specifying-line-colors-when-plotting-a-matrix
					set(subplothandles(plotdisplayidx(jj, 1), 1), 'ColorOrder', systemcolor, 'NextPlot', 'replacechildren');
					if isdiscrete
						stairs(t, y(:, jj:nplot:end));
					else
						plot(t, y(:, jj:nplot:end));
					end
					hold('all');
					grid('on');
					if jj <= number_measurements
						ylabel(['$y_{', num2str(jj), '}$']);
						elseif jj <= number_measurements + number_references
							ylabel(['$y_{ref, ', num2str(jj - number_measurements), '}$']);
					else
							ylabel(['$u_{', num2str(jj - number_measurements - number_references), '}$']);
					end
					axis('tight');
				end
			end
			if ~isempty(solcomment)
				title(subplothandles(1, 1), escape_latex(solcomment));
			end
			linkaxes(subplothandles, 'x');
		end

		function [len] = count(this)
			%COUNT return number of solutions
			%	Input:
			%		this:	instance
			%	Output:
			%		len:	number of solutions
			len = size(this.J_opt, 1);
		end

		function [name] = getfilename(this)
			%GETFILENAME return filename for SolutionSet
			%	Input:
			%		this:	instance
			%	Output:
			%		name:	filename for instance
			name = this.filename();
		end

		function [] = save(this)
			%SAVE save solutions
			%	Input:
			%		this:	instance
			save(fullfile(this.SAVEPATH, this.filename()), '-mat', 'this');
		end

		function [] = saveas(this, prefix, noobject)
			%SAVEAS save solution to file with prefix
			%	Input:
			%		this:		instance
			%		prefix:		prefix for filename
			%		noobject:	indicator, if problem settings ahould be incorporated into the filename
			if nargin <= 2
				noobject = false;
			end
			if ~islogical(noobject)
				error('control:design:gamma:solution:save', 'Indicator for incorporation of  problem settings must be of type ''logical''.');
			end
			if ~ischar(prefix) || ~isempty(strfind(prefix, '/')) || ~isempty(strfind(prefix, '\'))
				error('control:design:gamma:solution:save', 'Filename is invalid.');
			end
			if prefix(end) ~= '_'
				prefix = [prefix, '_'];
			end
			save(fullfile(this.SAVEPATH, [prefix, this.filename(noobject)]), '-mat', 'this');
		end

		function [] = addComment(this, comment)
			%ADDCOMMENT add a comment to the solution set
			%	Input:
			%		this:		instance
			%		comment:	comment to add
			if ~ischar(comment)
				if iscellstr(comment)
					comment = strjoin(comment, sprintf('\n'));
				else
					error('control:design:gamma:solution:input', 'Comment must be of type ''char''.');
				end
			end
			this.comment = [this.comment, comment];
		end

		function [] = clearComment(this)
			%CLEARCOMMENT remove comment from solution set
			%	Input:
			%		this:	instance
			this.comment = '';
		end

		function [] = setComment(this, comment)
			%SETCOMMENT set comment for solution set
			%	Input:
			%		this:		instance
			%		comment:	comment to set
			if ~ischar(comment)
				if iscellstr(comment)
					comment = strjoin(comment, sprintf('\n'));
				else
					error('control:design:gamma:solution:input', 'Comment must be of type ''char''.');
				end
			end
			this.comment = comment;
		end

		function [] = addSolutionComment(this, comment, idx, byid)
			%ADDSOLUTIONCOMMENT add a comment to the solution
			%	Input:
			%		this:		instance
			%		comment:	comment to add
			%		idx:		indices of solutions to add comment for
			%		byid:		indicator, if indices should be interpreted as ids
			if nargin <= 3
				idx = size(this.ids, 1);
			end
			if byid
				idxid = ismember(idx, this.ids);
				if isempty(idxid)
					error('control:design:gamma:solution:input', 'ID %d does not exist in solution set.', idx);
				end
				idx = idxid;
			end
			if any(idx <= 0) || any(idx > length(this.ids))
				error('control:design:gamma:solution:input', 'ID must be a positive integer smaller than number of solutions in current set.');
			end
			if ~ischar(comment)
				if iscellstr(comment)
					if length(idx) == 1
						comment = {strjoin(comment, sprintf('\n'))};
					else
						if size(comment, 1) == length(idx)
							error('control:design:gamma:solution:input', 'Number of comments must match number of IDs.');
						end
					end
				else
					error('control:design:gamma:solution:input', 'Comment must be of type ''char''.');
				end
				this.solutioncomment(idx, 1) = comment;
			else
				this.solutioncomment{idx, 1} = repmat({comment}, length(idx), 1);
			end
		end

		function [] = clearSolutionComment(this, idx, byid)
			%CLEARCOMMENT remove comment from solution
			%	Input:
			%		this:	instance
			%		idx:	indices of solutions to add comment for
			%		byid:	indicator, if indices should be interpreted as ids
			if nargin <= 3
				idx = size(this.ids, 1);
			end
			if byid
				idxid = ismember(idx, this.ids);
				if isempty(idxid)
					error('control:design:gamma:solution:input', 'ID %d does not exist in solution set.', idx);
				end
				idx = idxid;
			end
			if any(idx <= 0) || any(idx > length(this.ids))
				error('control:design:gamma:solution:input', 'ID must be a positive integer smaller than number of solutions in current set.');
			end
			this.solutioncomment{idx, 1} = repmat({''}, length(idx), 1);
		end
	end
end

function [name] = calculate_filename(controller, p, q, q_dot, r, systems, areafun_strict, areafun_loose, weight_strict, weight_loose, dimensions_strict, dimensions_loose, bounds, noproblemsettings)
	%CALCULATE_FILENAME filename without prefix and extension for saving and loading solutions
	%	Input:
	%		controller:			controller to calculate name for
	%		p:					number of controls
	%		q:					number of measurements
	%		q_dot:				number of derivative measurements
	%		r:					number of references
	%		systems:			structure with system matrices of systems to take into consideration
	%		areafun_strict:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for strict optimization of areafunctions
	%		areafun_loose:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for loose optimization of areafunctions
	%		weight_strict:		weighting matrix with number of systems columns and number of pole area border functions rows for strict optimization of areafunctions
	%		weight_loose:		weighting matrix with number of systems columns and number of pole area border functions rows for loose optimization of areafunctions
	%		dimensions_strict:	structure with information about dimensions of the variables and systems and fixed gain parameters for strict optimization of areafunctions
	%		dimensions_loose:	structure with information about dimensions of the variables and systems and fixed gain parameters for loose optimization of areafunctions
	%		number_states_all:	vector with orders of all systems
	%		bounds:				structure with information about bounded gain parameters for optimization
	%		noproblemsettings:	indicator, if problem settings like systems and objective function parametrization should be included in filename
	%	Output:
	%		name:		filename
	if nargin <= 13
		noproblemsettings = false;
	end
	controllername = baseclassname(controller);
	if noproblemsettings
		name = ['Controller_', controllername, '_', num2str(p), '_', num2str(q), '_', num2str(q_dot), '_', num2str(r)];
	else
		strict = dimensions_strict;
		loose = dimensions_loose;
		names = fieldnames(strict.area_parameters);
		namesloose = fieldnames(loose.area_parameters);
		if ~isequal(sort(names), sort(namesloose))
			% DataHash(rmfield(this.dimensions_strict, 'models')) is 50 times slower than the reshaping code below because of area_parameters field, which contains a huge structure array, maybe convert it to a single structure with double matrices with a mex file and then calculate the hash
			warning('control:design:gamma:solution:input', 'Fieldnames of area parameters are not equal, there must be a programming error somewhere in the area function argument handling.');
			dimensionstrictHash = DataHash(rmfield(strict, 'models'));
			dimensionlooseHash = DataHash(rmfield(loose, 'models'));
		else
			area_parameters_strict = strict.area_parameters;
			area_parameters_strict_new = cell2struct(cell(size(names, 1), 1), names, 1);
			area_parameters_loose = loose.area_parameters;
			area_parameters_loose_new = cell2struct(cell(size(names, 1), 1), names, 1);
			for ii = 1:size(names, 1) %#ok<FORPF> parfor can not be used because of dynamic structure reference
				areaparameters = {area_parameters_strict.(names{ii})};
				areaparameterssize = cellfun(@size, areaparameters, 'UniformOutput', false);
				sz = size(area_parameters_strict);
				if ~isempty(areaparameters)
					len = cellfun(@length, areaparameterssize, 'UniformOutput', true);
					if all(len(1) == len) && len(1) == 2
						areaparameterssize = cat(1, areaparameterssize{:});
						if all(areaparameterssize(1) == areaparameterssize(:, 1)) && all(areaparameterssize(2) == areaparameterssize(:, 2)) && areaparameterssize(2) == 1
							areaparameters = [areaparameters{:}];
						else
							maxdim = max(areaparameterssize(:, 2));
							areaparameters = cellfun(@(x) [x, NaN(size(x, 1), maxdim - size(x, 2))], 'UniformOutput', false);
							areaparameters = cat(1, areaparameters{:});
							sz = [sz(1), sz(2), maxdim];
						end
					else
						warning('control:design:gamma:solution:input', 'Area parameters must not be multidimensional matrices, there must be a programming error somewhere in the area function argument handling.');
					end
				end
				area_parameters_strict_new.(names{ii}) = reshape(areaparameters, sz);
				areaparameters = {area_parameters_loose.(names{ii})};
				areaparameterssize = cellfun(@size, areaparameters, 'UniformOutput', false);
				sz = size(area_parameters_loose);
				if ~isempty(areaparameters)
					len = cellfun(@length, areaparameterssize, 'UniformOutput', true);
					if all(len(1) == len) && len(1) == 2
						areaparameterssize = cat(1, areaparameterssize{:});
						if all(areaparameterssize(1) == areaparameterssize(:, 1)) && all(areaparameterssize(2) == areaparameterssize(:, 2)) && areaparameterssize(2) == 1
							areaparameters = [areaparameters{:}];
						else
							maxdim = max(areaparameterssize(:, 2));
							areaparameters = cellfun(@(x) [x, NaN(size(x, 1), maxdim - size(x, 2))], areaparameters, 'UniformOutput', false);
							areaparameters = cat(1, areaparameters{:});
							sz = [sz(1), sz(2), maxdim];
						end
					else
						warning('control:design:gamma:solution:input', 'Area parameters must not be multidimensional matrices, there must be a programming error somewhere in the area function argument handling.');
					end
				end
				area_parameters_loose_new.(names{ii}) = reshape(areaparameters, sz);
			end
			strict.area_parameters = area_parameters_strict_new;
			loose.area_parameters = area_parameters_loose_new;
			dimensionstrictHash = DataHash(rmfield(strict, 'models'));
			dimensionlooseHash = DataHash(rmfield(loose, 'models'));
		end
		strictHash = DataHash([DataHash(areafun_strict), DataHash(weight_strict), dimensionstrictHash, DataHash(bounds)]);
		looseHash = DataHash([DataHash(areafun_loose), DataHash(weight_loose), dimensionlooseHash, DataHash(bounds)]);
		name = ['Controller_', controllername, '_', num2str(p), '_', num2str(q), '_', num2str(q_dot), '_', num2str(r), '_', strictHash, '_', looseHash, '_', DataHash(systems)];
	end
end