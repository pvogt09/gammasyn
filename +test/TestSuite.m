classdef TestSuite < handle
	%TESTSUITE class for running test cases
	
	properties(Constant=true)
		% ignore pattern for folders that should not be searched for test cases
		EXCLUDE = {
			'.git';
			'.svn';
			'.hg';
			'slprj';
			'autogen';
			'emcprj';
			'lightspeed';
			'yalmip';
			'matlab2tikz';
			'codegen';
			'Ergebnisse';
			'temp';
			'private'
		};
		% search pattern for test cases
		PATTERN = '*Test.m'
	end
	
	properties(Access=protected)
		% found test cases
		tests,
		% information about test failure
		passed,
		% test results
		results
	end
	
	methods(Static=true)
		function [] = assert(a, identifier, message, varargin)
			%ASSERT assert that condition a is fulfilled
			%	Input:
			%		a:			condition to test
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			assert(a, identifier, message, varargin{:});
		end
		
		function [] = assertSameSize(a, b, identifier, message, varargin)
			%ASSERTSAMESIZE assert that a and b are same size variables
			%	Input:
			%		a:			variable to test
			%		b:			expected variable
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			if nargin <= 3
				message = 'A and B must be same size.';
			end
			assert(ndims(a) == ndims(b) && all(size(a) == size(b)), identifier, message, varargin{:});
		end
		
		function [] = assertEqual(a, b, identifier, message, varargin)
			%ASSERTEQUAL assert that a and b are equal
			%	Input:
			%		a:			variable to test
			%		b:			expected variable
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			test.TestSuite.assertSameSize(a, b, identifier);
			if nargin <= 3
				message = 'A and B must be equal.';
			end
			if isstruct(a) && isstruct(b)
				temp = isequal(sortstruct(a), sortstruct(b));
			elseif xor(isstruct(a), isstruct(b))
				assert(false, identifier, message, varargin{:});
			elseif iscell(a) && iscell(b)
				for ii = 1:numel(a)
					test.TestSuite.assertEqual(a{ii}, b{ii}, identifier, message, varargin{:});
				end
				temp = true;
			elseif xor(iscell(a), iscell(b))
				assert(false, identifier, message, varargin{:});
			elseif isnumeric(a) && isnumeric(b)
				temp = a == b;
			elseif isfunctionhandle(a) && isfunctionhandle(b)
				temp = nargin(a) == nargin(b);
			elseif xor(isfunctionhandle(a), isfunctionhandle(b))
				assert(false, identifier, message, varargin{:});
			else
				temp = a == b;
			end
			assert(all(temp(:)), identifier, message, varargin{:});
		end
		
		function [] = assertEqualNaN(a, b, identifier, message, varargin)
			%ASSERTEQUALNAN assert that a and b are equal with equal NaN
			%	Input:
			%		a:			variable to test
			%		b:			expected variable
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			test.TestSuite.assertSameSize(a, b, identifier);
			if nargin <= 3
				message = 'A and B must be equal.';
			end
			if isstruct(a) && isstruct(b)
				temp = isequaln(sortstruct(a), sortstruct(b));
			elseif xor(isstruct(a), isstruct(b))
				assert(false, identifier, message, varargin{:});
			elseif iscell(a) && iscell(b)
				for ii = 1:numel(a)
					test.TestSuite.assertEqualNaN(a{ii}, b{ii}, identifier, message, varargin{:});
				end
				temp = true;
			elseif xor(iscell(a), iscell(b))
				assert(false, identifier, message, varargin{:});
			elseif isnumeric(a) && isnumeric(b)
				temp = a == b;
				temp(isnan(a) & isnan(b)) = true;
			elseif isfunctionhandle(a) && isfunctionhandle(b)
				temp = nargin(a) == nargin(b);
			elseif xor(isfunctionhandle(a), isfunctionhandle(b))
				assert(false, identifier, message, varargin{:});
			else
				temp = a == b;
				temp(isnan(a) & isnan(b)) = true;
			end
			assert(all(temp(:)), identifier, message, varargin{:});
		end
		
		function [] = assertNotEqual(a, b, identifier, message, varargin)
			%ASSERTNOTEQUAL assert that a and b are not equal
			%	Input:
			%		a:			variable to test
			%		b:			expected variable
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			test.TestSuite.assertSameSize(a, b, identifier);
			if nargin <= 3
				message = 'A and B must not be equal.';
			end
			temp = a ~= b;
			assert(all(temp(:)), identifier, message, varargin{:});
		end
		
		function [] = assertEqualTolerance(a, b, tol, identifier, message, varargin)
			%ASSERTEQUALTOLERANCE assert that a and b are equal according to specified tolerance
			%	Input:
			%		a:			variable to test
			%		b:			expected variable
			%		tol:		tolerance for equality test
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			test.TestSuite.assertSameSize(a, b, identifier);
			if nargin <= 3
				message = sprintf('A and B must be equal within tolerance %f.', tol);
			end
			temp = abs(a - b);
			assert(all(temp(:) < tol), identifier, message, varargin{:});
		end
		
		function [] = assertLessThan(a, b, identifier, message, varargin)
			%ASSERTLESSTHAN assert that a is less than b
			%	Input:
			%		a:			variable to test
			%		b:			expected variable
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			test.TestSuite.assertSameSize(a, b, identifier);
			if nargin <= 3
				message = 'A must be less than B.';
			end
			temp = a < b;
			assert(all(temp(:)), identifier, message, varargin{:});
		end
		
		function [] = assertFieldnames(a, b, identifier, message, varargin)
			%ASSERTFIELDNAMES assert that a and b have the same fieldnames if b is a struct or that a has the fieldnames specified in b
			%	Input:
			%		a:			variable to test
			%		b:			fieldnames or structure with fieldnames a has to have
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			if isstruct(b)
				names = fieldnames(b);
			else
				if ischar(b)
					names = {b};
				elseif iscellstr(b)
					names = b;
				elseif isobject(b)
					names = properties(b);
				else
					names = char(b);
				end
			end
			if nargin <= 3
				message = ['A must have fields ''', strjoin(names, ''', '''), ''''];
			end
			test.TestSuite.assert(isstruct(a), identifier, message);
			temp = fieldnames(a);
			assert(all(ismember(names, temp)), identifier, message, varargin{:});
		end
		
		function [] = assertWarning(code, warning, identifier, message, varargin)
			%ASSERTWARNING assert that a warning is thrown
			%	Input:
			%		code:		code to test for warning
			%		warning:	expected warning identifier
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			if nargin <= 3
				message = sprintf('A warning of type ''%s'' must be thrown.', warning);
			end
			[oldmsgstr, oldmsgid] = lastwarn('', '');
			evalin('caller', code);
			[~, msgid] = lastwarn();
			assert(strcmp(msgid, warning), identifier, message, varargin{:});
			lastwarn(oldmsgstr, oldmsgid);
		end
		
		function [] = assertException(code, exception, identifier, message, varargin)
			%ASSERTEXCEPTION assert that an exception is thrown
			%	Input:
			%		code:		code to test for exception
			%		exception:	expected exception identifier
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			if nargin <= 3
				message = sprintf('An exception of type ''%s'' must be thrown.', exception);
			end
			haderror = false;
			try
				evalin('caller', code);
			catch e
				if ~ischar(exception) || ~strcmpi(exception, 'any')
					assert(strcmp(e.identifier, exception), identifier, message, varargin{:});
				end
				haderror = true;
			end
			if ~haderror
				assert(false, identifier, message, varargin{:});
			end
		end
		
		function [] = assertNoWarning(code, identifier, message, varargin)
			%ASSERTNOWARNING assert that no warning is thrown
			%	Input:
			%		code:		code to test for warning
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			if nargin <= 3
				message = 'A warning of type ''%s'' must not be thrown.';
			end
			[oldmsgstr, oldmsgid] = lastwarn('', '');
			evalin('caller', code);
			[~, msgid] = lastwarn();
			lastwarn(oldmsgstr, oldmsgid);
			if ~isempty(msgid)
				if nargin <= 3
					assert(false, identifier, message, msgid);
				else
					assert(false, identifier, message, varargin{:});
				end
			end
		end
		
		function [] = assertNoException(code, identifier, message, varargin)
			%ASSERTEXCEPTION assert that no exception is thrown
			%	Input:
			%		code:		code to test for exception
			%		identifier:	condition identifier string
			%		message:	condition message string
			%		varargin:	replacement arguments for message string
			if nargin <= 3
				message = 'An excpetion of type ''%s'' must not be thrown.\nerrormessage:\n\t\t%s';
			end
			try
				evalin('caller', code);
			catch e
				if nargin <= 3
					assert(false, identifier, message, e.identifier, strrep(e.getReport('extended'), sprintf('\n'), sprintf('\n\t\t')));
				else
					assert(false, identifier, message, varargin{:});
				end
			end
		end
		
		function [hadallowedexception] = assertNoExceptionExcept(code, except, identifier, message, varargin)
			%ASSERTEXCEPTIONEXCEPT assert that no exception except for exceptions of the specified type is thrown
			%	Input:
			%		code:					code to test for exception
			%		except:					error identifiers that may be thrown
			%		identifier:				condition identifier string
			%		message:				condition message string
			%		varargin:				replacement arguments for message string
			%	Output:
			%		hadallowedexception:	indicator if allowed exception was thrown
			if nargin <= 3
				message = 'An excpetion of type ''%s'' must not be thrown.\nerrormessage:\n\t\t%s';
			end
			hadallowedexception = false;
			try
				evalin('caller', code);
			catch e
				if ~any(strcmpi(e.identifier, except))
					if nargin <= 3
						assert(false, identifier, message, e.identifier, strrep(e.getReport('extended'), sprintf('\n'), sprintf('\n\t\t')));
					else
						assert(false, identifier, message, varargin{:});
					end
				end
				hadallowedexception = true;
			end
		end
		
		function [hadallowedexception] = assertNoExceptionExceptStack(code, except, exceptstack, identifier, message, varargin)
			%ASSERTEXCEPTIONEXCEPTSTACK assert that no exception except for exceptions of the specified type with matching stack is thrown
			%	Input:
			%		code:			code to test for exception
			%		except:			error identifiers that may be thrown
			%		exceptstack:	list of files and lines, where exceptions from the except list are allowed
			%		identifier:		condition identifier string
			%		message:		condition message string
			%		varargin:		replacement arguments for message string
			if nargin <= 4
				message = 'An excpetion of type ''%s'' must not be thrown.\nerrormessage:\n\t\t%s';
			end
			if ischar(except)
				except = {except};
			end
			if ~iscell(except) || (iscell(exceptstack) && size(except, 1) ~= size(exceptstack, 1)) || (isstruct(exceptstack) && (size(except, 1) ~= length(exceptstack) || ~all(isfield(exceptstack, {'file', 'line'}))))
				error('test:suite', 'Exceptionstack must match exceptions and must be of type ''cell'' or ''struct''.');
			end
			hadallowedexception = false;
			try
				evalin('caller', code);
			catch e
				matches = strcmpi(e.identifier, except);
				if ~any(matches)
					if nargin <= 4
						assert(false, identifier, message, e.identifier, strrep(e.getReport('extended'), sprintf('\n'), sprintf('\n\t\t')));
					else
						assert(false, identifier, message, varargin{:});
					end
				else
					idx = find(matches);
					allowederror = false(length(idx), 1);
					for ii = 1:length(idx)
						stack = e.stack;
						for jj = 1:length(stack)
							if isstruct(exceptstack)
								if strcmpi(stack(jj).file, exceptstack(idx(ii)).file)
									if ischar(exceptstack(idx(ii)).line) && strcmpi(exceptstack(idx(ii)).line, 'any')
										allowederror(ii, 1) = true;
										break;
									elseif isnumeric(exceptstack(idx(ii)).line) && exceptstack(idx(ii)).line == stack(jj).line
										allowederror(ii, 1) = true;
										break;
									end
								end
							else
								if strcmpi(stack(jj).file, exceptstack{idx(ii), 1})
									if ischar(exceptstack{idx(ii), 1}) && strcmpi(exceptstack{idx(ii), 1}, 'any')
										allowederror(ii, 1) = true;
										break;
									elseif isnumeric(exceptstack{idx(ii), 1}) && exceptstack{idx(ii), 1} == stack(jj).line
										allowederror(ii, 1) = true;
										break;
									end
								end
							end
						end
					end
					if nargin <= 4
						assert(all(allowederror), identifier, message, e.identifier, strrep(e.getReport('extended'), sprintf('\n'), sprintf('\n\t\t')));
					else
						assert(all(allowederror), identifier, message, varargin{:});
					end
					hadallowedexception = any(allowederror);
				end
			end
		end
	end
	
	methods
		function [this] = TestSuite(path, exclude)
			%TESTSUITE create new test suite
			%	Input:
			%		path:		path to search for test cases
			%		exclude:	ignore pattern for folders that should not be searched for tests
			%	Output:
			%		this:		instance
			if nargin <= 1
				exclude = this.EXCLUDE;
			end
			if ~ischar(path)
				error('test:suite', 'Search path must be a string.');
			end
			if ischar(exclude)
				exclude = {exclude};
			end
			if ~iscell(exclude)
				error('test:suite', 'Ignore pattern must be a string.');
			end
			exclude = union(exclude, this.EXCLUDE);
			this.tests = findTests(path, exclude, this.PATTERN);
			this.passed = repmat(identifier.TestResult.SKIPPED, size(this.tests, 1), 1);
			this.results = cell(size(this.tests, 1), 2);
		end
		
		function [tests] = gettests(this)
			%GETTESTS return test case files
			%	Input:
			%		this:	instance
			%	Output:
			%		tests:	test case files
			tests = this.tests;
		end
		
		function [] = run(this, display)
			%RUN run tests
			%	Input:
			%		this:		instance
			%		display:	show test progress
			if nargin <= 1
				display = false;
			end
			if ~islogical(display)
				error('test:run', 'Progress flag must be of type ''logical''.');
			end
			testvariable = this.tests;
			wait = Progress(size(testvariable, 1), 'Unittests', display);
			for i = 1:size(testvariable, 1)
				if wait.iscancelled()
					break;
				end
				try
					retval = eval([testvariable{i}, '(true)']);
					result = [];
				catch ebase
					if strcmp(ebase.identifier, 'MATLAB:scriptNotAFunction')
						try
							run([testvariable{i}, '.m']);
							retval = identifier.TestResult.PASSED;
							result = [];
						catch e
							result = e;
							retval = identifier.TestResult.FAILED;
						end
					else
						result = ebase;
						retval = identifier.TestResult.FAILED;
					end
				end
				this.results{i, 1} = retval;
				this.results{i, 2} = result;
				this.passed(i) = retval;
				wait.step('Test %d/%d');
			end
			clear wait;
		end

		function [] = rerun_failed(this, display)
			%RERUN_FAILED run failed tests again
			%	Input:
			%		this:		instance
			%		display:	show test progress
			if nargin <= 1
				display = false;
			end
			if ~islogical(display)
				error('test:run', 'Progress flag must be of type ''logical''.');
			end
			testvariable = this.tests;
			idx = ~[this.passed.ispassed];
			if ~isempty(idx)
				testvariable = testvariable(idx);
				pass = this.passed(idx);
				res = this.results(idx, :);
				wait = Progress(size(testvariable, 1), 'Unittests', display);
				for i = 1:size(testvariable, 1)
					if wait.iscancelled()
						break;
					end
					TheTest = testvariable{i};
					try
						retval = eval([TheTest, '(true)']);
						result = [];
					catch e
						result = e;
						retval = identifier.TestResult.FAILED;
					end
					res{i, 1} = retval;
					res{i, 2} = result;
					pass(i) = identifier.TestResult.getresult(retval);
					wait.step('Test %d/%d');
				end
				clear wait;
				this.results(idx, :) = res;
				this.passed(idx) = pass;
			end
		end
		
		function [summary] = summary(this, print)
			%SUMMARY show summary of test results
			%	Input:
			%		this:		instance
			%		display:	indicator whether test result should be printed
			%	Output:
			%		summary:	summary of test results
			if nargin <= 1
				print = true;
			end
			summary = sprintf('Test results for %d tests:\n', size(this.tests, 1));
			failed = ~[this.passed.ispassed];
			summary= [summary, sprintf('Passed: %d, Skipped: %d, Failed: %d\n', sum(double(this.passed == identifier.TestResult.PASSED)), sum(double(this.passed == identifier.TestResult.SKIPPED)), sum(double(failed)))];
			idx = 1:size(failed, 1);
			if any(failed)
				summary = [summary, 'Failed Tests:\n'];
				for i = idx(failed)
					summary = [summary, sprintf(' Test ''%s''\n', this.tests{i})];
					summary = [summary, sprintf('   Result: %s\n', this.passed(i).tostring)];
					if ~isempty(this.results{i, 2})
						summary = [summary, sprintf('   Errormessage: %s\n', strrep(this.results{i, 2}.getReport(), sprintf('\n'), sprintf('\n    ')))];
					else
						summary = [summary, sprintf('   Errormessage: none\n')];
					end
				end
				summary= [summary, sprintf('Passed: %d, Skipped: %d, Failed: %d\n', sum(this.passed == identifier.TestResult.PASSED), sum(this.passed == identifier.TestResult.SKIPPED), sum(failed))];
			end
			if print
				disp(summary);
			end
			if nargout <= 0
				clear summary;
			end
		end
	end
end