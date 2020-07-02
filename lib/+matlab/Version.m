classdef(Enumeration) Version < handle
	%class to represent a version of matlab
	
	enumeration
		% currently running version
		CURRENT();
		% R2019B
		R2019B('R2019B', 42, 9.7,	10.0);
		% R2019A
		R2019A('R2019A', 41, 9.6,	9.3);
		% R2018B
		R2018B('R2018B', 40, 9.5,	9.2);
		% R2018A
		R2018A('R2018A', 39, 9.4,	9.1);
		% R2017B
		R2017B('R2017B', 38, 9.3,	9.0);
		% R2017A
		R2017A('R2017A', 37, 9.2,	8.9);
		% R2016B
		R2016B('R2016B', 36, 9.1,	8.8);
		% R2016A
		R2016A('R2016A', 35, 9.0,	8.7);
		% R2015B
		R2015B('R2015B', 34, 8.6,	8.6);
		% R2015A
		R2015A('R2015A', 33, 8.5,	8.5);
		% R2014B
		R2014B('R2014B', 32, 8.4,	8.4);
		% R2014A
		R2014A('R2014A', 31, 8.3,	8.3);
		% R2013B
		R2013B('R2013B', 30, 8.2,	8.2);
		% R2013A
		R2013A('R2013A', 29, 8.1,	8.1);
		% R2012B
		R2012B('R2012B', 28, 8.0,	8.0);
		% R2012A
		R2012A('R2012A', 27, 7.14,	7.9);
		% R2011B
		R2011B('R2011B', 26, 7.13,	7.8);
		% R2011A
		R2011A('R2011A', 25, 7.12,	7.7);
		% R2010B
		R2010B('R2010B', 24, 7.11,	7.6);
		% R2010A
		R2010A('R2010A', 23, 7.10,	7.5);
		% R2009B
		R2009B('R2009B', 22, 7.9,	7.4);
		% R2009A
		R2009A('R2009A', 21, 7.8,	7.3);
		% R2008B
		R2008B('R2008B', 20, 7.7,	7.2);
		% R2008A
		R2008A('R2008A', 19, 7.6,	7.1);
		% R2007B
		R2007B('R2007B', 18, 7.5,	7.0);
		% R2007A
		R2007A('R2007A', 17, 7.4,	6.6);
		% R2006B
		R2006B('R2006B', 16, 7.3,	6.5);
		% R2006A
		R2006A('R2006A', 15, 7.2,	6.4);
	end
	
	properties(SetAccess=private)
		% name of matlab version
		name,
		% number of matlab version
		number,
		% version number of matlab version
		matlabversion,
		% version number of simulink version
		simulinkversion
	end
	
	methods
		function [isless] = less(this, that)
			%LESS return, if version is less than given version
			%	Input:
			%		this:			instance
			%		that:			version to compare to, if none is given, current version is used for comparison
			%	Output:
			%		isless:			indicator, if version is less than the given version or current version, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			isless = this.matlabversion < that.matlabversion;
		end
		
		function [islessoreq] = lessorequal(this, that)
			%LESSOREQUAL return, if version is less or equal than given version
			%	Input:
			%		this:			instance
			%		that:			version to compare to, if none is given, current version is used for comparison
			%	Output:
			%		islessoreq:		indicator, if version is less than or equal the given version or current version, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			islessoreq = this.matlabversion <= that.matlabversion;
		end
		
		function [isless] = lesssimulink(this, that)
			%LESSSIMULINK return, if simulinkversion is less than given simulinkversion
			%	Input:
			%		this:			instance
			%		that:			simulinkversion to compare to, if none is given, current simulinkversion is used for comparison
			%	Output:
			%		isless:			indicator, if simulinkversion is less than the given simulinkversion or current simulinkversion, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			isless = this.simulinkversion < that.simulinkversion;
		end
		
		function [islessoreq] = lessorequalsimulink(this, that)
			%LESSOREQUALSIMULINK return, if simulinkversion is less or equal than given simulinkversion
			%	Input:
			%		this:			instance
			%		that:			simulinkversion to compare to, if none is given, current simulinkversion is used for comparison
			%	Output:
			%		islessoreq:		indicator, if simulinkversion is less than or equal the given simulinkversion or current simulinkversion, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			islessoreq = this.simulinkversion <= that.simulinkversion;
		end
		
		function [isless] = lt(this, that)
			%LT return, if version is less than given version
			%	Input:
			%		this:			instance
			%		that:			version to compare to, if none is given, current version is used for comparison
			%	Output:
			%		isless:			indicator, if version is less than the given version or current version, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			isless = this.less(that);
		end
		
		function [islessoreq] = le(this, that)
			%LE return, if version is less or equal than given version
			%	Input:
			%		this:			instance
			%		that:			version to compare to, if none is given, current version is used for comparison
			%	Output:
			%		islessoreq:		indicator, if version is less than or equal the given version or current version, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			islessoreq = this.lessorequal(that);
		end
		
		function [isgreater] = gt(this, that)
			%GT return, if version is greater than given version
			%	Input:
			%		this:			instance
			%		that:			version to compare to, if none is given, current version is used for comparison
			%	Output:
			%		isgreater:		indicator, if version is greater than the given version or current version, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			isgreater = ~this.lessorequal(that);
		end
		
		function [isgreateroreq] = ge(this, that)
			%GE return, if version is greater or equal than given version
			%	Input:
			%		this:			instance
			%		that:			version to compare to, if none is given, current version is used for comparison
			%	Output:
			%		isgreateroreq:	indicator, if version is greater than or equal the given version or current version, if no version to compare to is supplied
			if nargin <= 1
				that = matlab.Version.CURRENT();
			end
			isgreateroreq = ~this.less(that);
		end
	end
	
	methods(Access=private)
		function [this] = Version(name, number, matlabversion, simulinkversion)
			%VERSION create new version object
			%	Input:
			%		name:				name of matlab release
			%		number:				number of matlab release
			%		matlabversion:		version number of matlab release
			%		simulinkversion:	version number of simulink release
			if nargin == 0
				version = ver;
				names = {version.Name};
				matlab = strcmpi(names, 'MATLAB');
				simulink = strcmpi(names, 'SIMULINK');
				if ~any(matlab) || ~any(simulink)
					error('matlab:version', 'Error in parsing version strings for current matlab version.');
				end
				this.name = upper(strrep(strrep(version(matlab).Release, '(', ''), ')', ''));
				this.number = 0;
				this.matlabversion = str2double(version(matlab).Version);
				this.simulinkversion = str2double(version(simulink).Version);
			else
				this.name = name;
				this.number = number;
				this.matlabversion = matlabversion;
				this.simulinkversion = simulinkversion;
			end
		end
	end
end