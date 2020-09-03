classdef OptionFactory < handle
	%OPTIONFACTORY create optimization options for different solvers

	properties(Access=private)
		% map of registered solvers
		map
	end

	methods(Static=true)
		function [this] = instance()
			%INSTANCE return singleton instance for factory
			%	Output:
			%		this:	instance
			persistent object;
			if isempty(object)
				object = optimization.options.OptionFactory();
			end
			this = object;
		end
	end

	methods(Access=private)
		function [this] = OptionFactory()
			%OPTIONFACTORY create new option factory
			%	Output:
			%		this:	instance
			this.map = containers.Map('KeyType', 'char', 'ValueType', 'any');
			this.registerSolver(optimization.solver.Optimizer.FMINCON,			'optimization.options.fmincon');
			this.registerSolver(optimization.solver.Optimizer.FMINCONGLOBAL,	'optimization.options.fminconglobal');
			this.registerSolver(optimization.solver.Optimizer.FMINSEARCH,		'optimization.options.fminsearch');
			this.registerSolver(optimization.solver.Optimizer.FMINUNC,			'optimization.options.fminunc');
			this.registerSolver(optimization.solver.Optimizer.FMINUNCGLOBAL,	'optimization.options.fminuncglobal');
			this.registerSolver(optimization.solver.Optimizer.SNOPT,			'optimization.options.snopt');
			this.registerSolver(optimization.solver.Optimizer.IPOPT,			'optimization.options.ipopt');
			this.registerSolver(optimization.solver.Optimizer.GA,				'optimization.options.ga');
			this.registerSolver(optimization.solver.Optimizer.PARTICLESWARM,	'optimization.options.particleswarm');
			this.registerSolver(optimization.solver.Optimizer.PATTERNSEARCH,	'optimization.options.patternsearch');
			this.registerSolver(optimization.solver.Optimizer.SIMULANNEAL,		'optimization.options.simulanneal');
			this.registerSolver(optimization.solver.Optimizer.NLOPTCON,			'optimization.options.nloptcon');
			this.registerSolver(optimization.solver.Optimizer.NLOPTUNC,			'optimization.options.nloptunc');
			this.registerSolver(optimization.solver.Optimizer.NLOPTCONGLOBAL,	'optimization.options.nloptconglobal');
			this.registerSolver(optimization.solver.Optimizer.NLOPTUNCGLOBAL,	'optimization.options.nloptuncglobal');
			this.registerSolver(optimization.solver.Optimizer.FMINIMAX,			'optimization.options.fminimax');
			this.registerSolver(optimization.solver.Optimizer.PPPBOX,			'optimization.options.pppbox');
			this.registerSolver(optimization.solver.Optimizer.SLPGS,			'optimization.options.slpgs');
			this.registerSolver(optimization.solver.Optimizer.SQPGS,			'optimization.options.sqpgs');
			this.registerSolver(optimization.solver.Optimizer.SCBFGS,			'optimization.options.scbfgs');
			this.registerSolver(optimization.solver.Optimizer.KSOPT,			'optimization.options.ksopt');
			this.registerSolver(optimization.solver.Optimizer.SOLVOPT,			'optimization.options.solvopt');
		end
	end

	methods
		function [] = registerSolver(this, solver, classname)
			%REGISTERSOLVER register new solver
			%	Input:
			%		this:		instance
			%		solver:		name of solver to register
			%		classname:	name of class to represent solver options for solver
			if nargin < 3 || isempty(solver) || isempty(classname)
				error('optimization:options:factory', 'Missing input arguments for method registerSolver.');
			end
			if ischar(solver)
				if this.map.isKey(solver)
					error('optimization:options:factory', 'Solver ''%s'' is already registered.', solver);
				end
			elseif isa(solver, 'optimization.solver.Optimizer')
				if this.map.isKey(solver.algorithmname)
					error('optimization:options:factory', 'Solver ''%s'' is already registered.', solver.algorithmname);
				end
				solver = solver.algorithmname;
			else
				error('optimization:options:factory', 'Undefined function or variable registerSolver for input arguments of type ''%s''.', class(solver));
			end
			if ~ischar(classname)
				if isa(classname, 'meta.class')
					classname = classname.Name;
				else
					error('optimization:options:factory', 'Undefined function or variable registerSolver for input arguments of type ''%s''.', class(solver));
				end
			end
			if issubclassof(classname, ?optimization.options.Options)
				this.map(solver) = str2func(classname);
			else
				error('optimization:options:factory', 'Registered option class ''%s'' is no subclass of ''%s''.', class(solver), 'optimization.options.Options');
			end
		end

		function [option] = options(this, solver, varargin)
			%OPTIONS return options for specified solver
			%	Input:
			%		this:		instance
			%		solver:		solver name
			%		varargin:	arguments to pass to solver options constructor
			%	Output:
			%		option:		corresponding options for solver
			if ischar(solver)
				if ~this.map.isKey(solver)
					error('optimization:options:factory', 'Solver ''%s'' is not registered.', solver);
				end
				createCallback = this.map(solver);
			elseif isa(solver, 'optimization.solver.Optimizer')
				if ~this.map.isKey(solver.algorithmname)
					error('optimization:options:factory', 'Solver ''%s'' is not registered.', solver.algorithmname);
				end
				createCallback = this.map(solver.algorithmname);
			elseif isa(solver, 'optimization.options.Options')
				if ~this.map.isKey(solver.Solver.algorithmname)
					error('optimization:options:factory', 'Solver ''%s'' is not registered.', solver.algorithmname);
				end
				if nargin <= 2
					% TODO: if deep copy is needed, remove this or let Options inherit from matlab.mixin.Copyable
					option = solver;
					return;
				end
				createCallback = this.map(solver.Solver.algorithmname);
				varargin = [{solver}, varargin];
			elseif isstruct(solver) && isfield(solver, 'Solver')
				option = this.options(solver.Solver, solver);
				return;
			elseif isstruct(solver) && isfield(solver, 'solver')
				option = this.options(solver.solver, solver);
				return;
			else
				error('optimization:options:factory', 'Undefined function or variable options for input arguments of type ''%s''.', class(solver));
			end
			option = createCallback(varargin{:});
		end
	end
end