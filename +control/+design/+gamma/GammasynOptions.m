classdef GammasynOptions < handle
	%GAMMASYNOPTIONS class for storing options for objective functions and other parameters for gammasyn optimization runs

	properties(Constant=true, Hidden=true)
		% prototype for options used for code generation
		PROTOTYPE_CODEGEN = struct(...
			'usecompiled',			false,...
			'numthreads',			uint32(configuration.matlab.numthreads()),...
			'type',					GammaJType.ZERO,...
			'weight',				0,...
			'allowvarorder',		false,...
			'eigenvaluederivative',	GammaEigenvalueDerivativeType.getDefaultValue(),...
			'eigenvaluefilter',		GammaEigenvalueFilterType.getDefaultValue(),...
			'eigenvalueignoreinf',	false,...
			'decouplingcontrol',	struct(...
				'tf_structure',					[],...
				'decouplingstrategy',			GammaDecouplingStrategy.getDefaultValue(),...
				'sortingstrategy_decoupling',	GammaDecouplingconditionSortingStrategy.getDefaultValue(),...
				'weight_decoupling',			1,...
				'weight_prefilter',				1,...
				'tolerance_decoupling',			0,...
				'tolerance_prefilter',			0,...
				'solvesymbolic',				true,...
				'round_equations_to_digits',	NaN,...
				'allowoutputdecoupling',		false...
			),...
			'objective',			struct(...
				'preventNaN',		false,...
				'kreisselmeier',	struct(...
					'rho',				20,...
					'max',				(0)...
				),...
				'lyapunov',			struct(...
					'Q',				[]...
				),...
				'normgain',			struct(...
					'R',		[],...
					'R_shift',	[],...
					'K',		[],...
					'K_shift',	[],...
					'F',		[],...
					'F_shift',	[]...
				)...
			)...
		);
	end

	properties(Constant=true)
		% prototype for options with default values
		PROTOTYPE = struct(...
			'usecompiled',			false,...
			'numthreads',			uint32(configuration.matlab.numthreads()),...
			'type',					GammaJType.getDefaultValue(),...
			'weight',				1,...
			'allowvarorder',		false,...
			'eigenvaluederivative',	GammaEigenvalueDerivativeType.getDefaultValue(),...
			'eigenvaluefilter',		GammaEigenvalueFilterType.getDefaultValue(),...
			'eigenvalueignoreinf',	false,...
			'decouplingcontrol',	struct(...
				'tf_structure',					[],...
				'decouplingstrategy',			GammaDecouplingStrategy.getDefaultValue(),...
				'sortingstrategy_decoupling',	GammaDecouplingconditionSortingStrategy.getDefaultValue(),...
				'weight_decoupling',			1,...
				'weight_prefilter',				1,...
				'tolerance_decoupling',			NaN,...
				'tolerance_prefilter',			NaN,...
				'solvesymbolic',				false,...
				'round_equations_to_digits',	NaN,...
				'allowoutputdecoupling',		false...
			),...
			'objective',			struct(...
				'preventNaN',		false,...
				'kreisselmeier',	struct(...
					'rho',				NaN,...
					'max',				NaN...
				),...
				'lyapunov',			struct(...
					'Q',				[]...
				),...
				'normgain',			struct(...
					'R',		[],...
					'R_shift',	[],...
					'K',		[],...
					'K_shift',	[],...
					'F',		[],...
					'F_shift',	[]...
				)...
			),...
			'allownegativeweight',		false,...
			'strategy',					GammaSolutionStrategy.getDefaultValue(),...
			'errorhandler',				GammaErrorHandler.getDefaultValue(),...
			'errorhandler_function',	[],...
			'system', struct(...
				'usereferences',		true,...
				'usemeasurements_xdot',	true,...
				'samples',				struct(),...
				'Blocks',				struct()...
			)...
		);
	end

	properties(Dependent=true)
		% indicator if generated code should be used
		usecompiled = [];
		% number of parallel threads to use
		numthreads = [];
		% type of objective function(s) to use
		type = [];
		% weight for the used objective functions
		weight = [];
		% indicator if systems with different numbers of states are allowed
		allowvarorder = [];
		% method for calculation of eigenvalue derivatives to use
		eigenvaluederivative = [];
		% filter for eigenvalues to use
		eigenvaluefilter = [];
		% indicator for ignoring infinte eigenvalues to use
		eigenvalueignoreinf = [];
		% options for decoupling controller design
		decouplingcontrol = [];
		% options for objective functions
		objective = [];
		% indicator if negative weights are allowed
		allownegativeweight = [];
		% strategy for solution to use
		strategy = [];
		% type of error handler to use
		errorhandler = [];
		% error handler function to use in case of GammaErrorHandler.USER
		errorhandler_function = [];
		% options for multiple models
		system = [];
	end

	properties(Dependent=true)
		% indicator if references should be used
		usereferences = [];
		% indicator if derivative measurements should be used
		usemeasurements_xdot = [];
		% structure with number of samples for genmat system blocks
		samples = [];
		% structure with number of samples for genmat system blocks
		Blocks = [];
		% structure of transfer matrix
		tf_structure = [];
		% strategy for decoupling controller design
		decouplingstrategy = [];
		% strategy for sorting decoupling conditions
		sortingstrategy_decoupling = [];
		% weight for decoupling controller design constraints
		weight_decoupling = [];
		% weight for prefilter decoupling controller design constraints
		weight_prefilter = [];
		% tolerance for decoupling controller design constraints
		tolerance_decoupling = [];
		% tolerance for regularization of prefilter in decoupling controller design
		tolerance_prefilter = [];
		% setting to perform calculations for decoupling constraints symbolically
		solvesymbolic = [];
		% setting to how many decimal places systems of equations that have to be solved are rounded in case of numerical difficulties
		round_equations_to_digits = [];
		% indicator wheter ouput feedback is allowed for decoupling controller design
		allowoutputdecoupling = [];
		% indicator if NaN should be prevented in objective functions
		preventNaN = [];
		% kreisselmeier objective parameter rho
		rho = [];
		% kreisselmeier objective parameter max
		max = [];
		% lyapunov objective parameter Q
		Q = [];
		% normgain objective parameter R
		R = [];
		% normgain objective parameter R_shift
		R_shift = [];
		% normgain objective parameter K
		K = [];
		% normgain objective parameter K_shift
		K_shift = [];
		% normgain objective parameter F
		F = [];
		% normgain objective parameter F_shift
		F_shift = [];
	end

	properties(Access=protected)
		% indicator if generated code should be used (internal)
		usecompiled_internal = [];
		% number of parallel threads to use (internal)
		numthreads_internal = [];
		% type of objective function(s) to use (internal)
		type_internal = [];
		% weight for the used objective functions (internal)
		weight_internal = [];
		% indicator if systems with different numbers of states are allowed (internal)
		allowvarorder_internal = [];
		% method for calculation of eigenvalue derivatives to use (internal)
		eigenvaluederivative_internal = [];
		% filter for eigenvalues to use (internal)
		eigenvaluefilter_internal = [];
		% indicator for ignoring infinite eigenvalues to use (internal)
		eigenvalueignoreinf_internal = [];
		% options for decoupling controller design (internal)
		decouplingcontrol_internal = [];
		% options for objective functions (internal)
		objective_internal = [];
		% indicator if negative weights are allowed (internal)
		allownegativeweight_internal = [];
		% strategy for solution to use (internal)
		strategy_internal = [];
		% type of error handler to use (internal)
		errorhandler_internal = [];
		% error handler function to use in case of GammaErrorHandler.USER (internal)
		errorhandler_function_internal = [];
		% options for multiple models (internal)
		system_internal = [];
	end

	properties(Access=protected)
		% indicator if user set 'usecompiled' option
		usecompiled_user = false;
		% indicator if user set 'numthreads' option
		numthreads_user = false;
		% indicator if user set 'type' option
		type_user = false;
		% indicator if user set 'weight' option
		weight_user = false;
		% indicator if user set 'allowvarorder' option
		allowvarorder_user = false;
		% indicator if user set 'eigenvaluederivative' option
		eigenvaluederivative_user = false;
		% indicator if user set 'eigenvaluefilter' option
		eigenvaluefilter_user = false;
		% indicator if user set 'eigenvalueignoreinf' option
		eigenvalueignoreinf_user = false;
		% indicator if user set 'decouplingcontrol' option
		decouplingcontrol_user = struct(...
			'tf_structure',					false,...
			'decouplingstrategy',			false,...
			'sortingstrategy_decoupling',	false,...
			'weight_decoupling',			false,...
			'weight_prefilter',				false,...
			'tolerance_decoupling',			false,...
			'tolerance_prefilter',			false,...
			'solvesymbolic',				false,...
			'round_equations_to_digits',	false,...
			'allowoutputdecoupling',			false...
		);
		% indicator if user set 'objective' option
		objective_user = struct(...
			'preventNaN',		false,...
			'kreisselmeier',	struct(...
				'rho',	false,...
				'max',	false...
			),...
			'lyapunov',			struct(...
				'Q',	false...
			),...
			'normgain', struct(...
				'R',		false,...
				'R_shift',	false,...
				'K',		false,...
				'K_shift',	false,...
				'F',		false,...
				'F_shift',	false...
			)...
		);
		% indicator if user set 'allownegativeweight' option
		allownegativeweight_user = false;
		% indicator if user set 'strategy' option
		strategy_user = false;
		% indicator if user set 'error_handler' option
		errorhandler_user = false;
		% indicator if user set 'errorhandler_function' option
		errorhandler_function_user = false;
		% indicator if user set 'system' option
		system_user = struct(...
			'usereferences',		false,...
			'usemeasurements_xdot',	false,...
			'samples',				false,...
			'Blocks',				false...
		);
	end

	properties(Access=protected, Transient=true)% avoid saving "constant" properties containing property name mappings to files
		% user setable options for optimization
		options;
		%structoptions;
	end

	methods
		function [this] = GammasynOptions(varargin)
			%GAMMASYNOPTIONS class for representing options to be used with gammasyn
			%	Input:
			%		varargin:	GammasynOptions object or structure to copy values from optionally in first argument and key value list of options to set with paths as cellstrings
			%	Output:
			%		this:		instance
			this.options = this.available_options();
			this.use_default();
			if nargin == 0
				return;
			end
			if nargin == 1
				this.useoptions(varargin{1});
			else
				this.useoptions(varargin{1}, varargin{2:end});
			end
		end

		function [usecompiled] = get.usecompiled(this)
			%USECOMPILED getter for usecompiled property
			%	Input:
			%		this:			instance
			%	Output:
			%		usecompiled:	indicator if compiled code should be used
			usecompiled = this.usecompiled_internal;
		end

		function [numthreads] = get.numthreads(this)
			%NUMTHREADS getter for numthreads property
			%	Input:
			%		this:		instance
			%	Output:
			%		numthreads:	number of parallel threads to use
			numthreads = this.numthreads_internal;
		end

		function [type] = get.type(this)
			%TYPE getter for objective type property
			%	Input:
			%		this:		instance
			%	Output:
			%		type:		objective type to use
			type = this.type_internal;
		end

		function [weight] = get.weight(this)
			%WEIGHT getter for objective weight property
			%	Input:
			%		this:		instance
			%	Output:
			%		weight:		objective weight to use
			weight = this.weight_internal;
		end

		function [allowvarorder] = get.allowvarorder(this)
			%ALLOWVARORDER getter for indicator for systems with different order
			%	Input:
			%		this:				instance
			%	Output:
			%		allowvarorder:		indicator if systems of different order are allowed
			allowvarorder = this.allowvarorder_internal;
		end

		function [eigenvaluederivative] = get.eigenvaluederivative(this)
			%EIGENVALUEDERIVATIVE getter for type of eigenvalue derivative method to use
			%	Input:
			%		this:						instance
			%	Output:
			%		eigenvaluederivative:		type of eigenvalue derivative method to use
			eigenvaluederivative = this.eigenvaluederivative_internal;
		end

		function [eigenvaluefilter] = get.eigenvaluefilter(this)
			%EIGENVALUEFILTER getter for type of filter method to use for eigenvalues
			%	Input:
			%		this:					instance
			%	Output:
			%		eigenvaluefilter:		type of filter method to use for eigenvalues
			eigenvaluefilter = this.eigenvaluefilter_internal;
		end

		function [eigenvalueignoreinf] = get.eigenvalueignoreinf(this)
			%EIGENVALUEIGNOREINF getter for indicator wheter infinite eigenvalues are ignored
			%	Input:
			%		this:					instance
			%	Output:
			%		eigenvalueignoreinf:	indicator wheter infinite eigenvalues are ignored
			eigenvalueignoreinf = this.eigenvalueignoreinf_internal;
		end

		function [decouplingcontrol] = get.decouplingcontrol(this)
			%DECOUPLINGCONTROL getter for decoupling controller design settings
			%	Input:
			%		this:				instance
			%	Output:
			%		decouplingcontrol:	structure with decoupling controller design settings
			if isstruct(this.decouplingcontrol_internal)
				decouplingcontrol = this.decouplingcontrol_internal;
			else
				decouplingcontrol = struct();
			end
		end

		function [tf_structure] = get.tf_structure(this)
			%TF_STRUCTURE getter for structure of the transfer matrix
			%	Input:
			%		this:			instance
			%	Output:
			%		tf_structure:	structure of the transfer matrix
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'tf_structure')
				tf_structure = this.decouplingcontrol_internal.tf_structure;
			else
				tf_structure = [];
			end
		end

		function [decouplingstrategy] = get.decouplingstrategy(this)
			%DECOUPLINGSTRATEGY getter for decoupling controller design strategy
			%	Input:
			%		this:				instance
			%	Output:
			%		decouplingstrategy:	decoupling controller design strategy
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'decouplingstrategy')
				decouplingstrategy = this.decouplingcontrol_internal.decouplingstrategy;
			else
				decouplingstrategy = [];
			end
		end

		function [sortingstrategy_decoupling] = get.sortingstrategy_decoupling(this)
			%SORTINGSTRATEGY_DECOUPLING getter for sorting strategy for decoupling conditions
			%	Input:
			%		this:						instance
			%	Output:
			%		sortingstrategy_decoupling:	decoupling controller design strategy
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'sortingstrategy_decoupling')
				sortingstrategy_decoupling = this.decouplingcontrol_internal.sortingstrategy_decoupling;
			else
				sortingstrategy_decoupling = [];
			end
		end

		function [weight_decoupling] = get.weight_decoupling(this)
			%WEIGHT_DECOUPLING getter for weight of decoupling controller design constraints
			%	Input:
			%		this:				instance
			%	Output:
			%		weight_decoupling:	weight for decoupling controller design constraints
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'weight_decoupling')
				weight_decoupling = this.decouplingcontrol_internal.weight_decoupling;
			else
				weight_decoupling = [];
			end
		end

		function [weight_prefilter] = get.weight_prefilter(this)
			%WEIGHT_PREFILTER getter for weight of prefilter decoupling controller design constraints
			%	Input:
			%		this:				instance
			%	Output:
			%		weight_prefilter:	weight for prefilter decoupling controller design constraints
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'weight_prefilter')
				weight_prefilter = this.decouplingcontrol_internal.weight_prefilter;
			else
				weight_prefilter = [];
			end
		end

		function [tolerance_decoupling] = get.tolerance_decoupling(this)
			%TOLERANCE_DECOUPLING getter for tolerance of decoupling controller design constraints
			%	Input:
			%		this:				instance
			%	Output:
			%		tolerance_decoupling:	tolerance for decoupling controller design constraints
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'tolerance_decoupling')
				tolerance_decoupling = this.decouplingcontrol_internal.tolerance_decoupling;
			else
				tolerance_decoupling = [];
			end
		end

		function [tolerance_prefilter] = get.tolerance_prefilter(this)
			%TOLERANCE_PREFILTER getter for tolerance of prefilter regularization constraint
			%	Input:
			%		this:				instance
			%	Output:
			%		tolerance_prefilter:	tolerance for decoupling controller design constraints
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'tolerance_prefilter')
				tolerance_prefilter = this.decouplingcontrol_internal.tolerance_prefilter;
			else
				tolerance_prefilter = [];
			end
		end

		function [solvesymbolic] = get.solvesymbolic(this)
			%SOLVESYMBOLIC getter for setting to perform calculations for decoupling constraints symbolically
			%	Input:
			%		this:			instance
			%	Output:
			%		solvesymbolic:	setting to perform calculations for decoupling constraints symbolically
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'solvesymbolic')
				solvesymbolic = this.decouplingcontrol_internal.solvesymbolic;
			else
				solvesymbolic = [];
			end
		end

		function [round_equations_to_digits] = get.round_equations_to_digits(this)
			%ROUND_EQUATIONS_TO_DIGITS getter for setting to how many decimal places systems of equations that have to be solved are rounded in case of numerical difficulties
			%	Input:
			%		this:						instance
			%	Output:
			%		round_equations_to_digits:	setting for rounding to number of decimal places
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'round_equations_to_digits')
				round_equations_to_digits = this.decouplingcontrol_internal.round_equations_to_digits;
			else
				round_equations_to_digits = [];
			end
		end

		function [allowoutputdecoupling] = get.allowoutputdecoupling(this)
			%ALLOWOUPUTDECOUPLING getter for indicator whether ouput feedback is allowed for decoupling control
			%	Input:
			%		this:					instance
			%	Output:
			%		allowoutputdecoupling:	indicator whether output feedback is allowed for decoupling control
			if isstruct(this.decouplingcontrol_internal) && isfield(this.decouplingcontrol_internal, 'allowoutputdecoupling')
				allowoutputdecoupling = this.decouplingcontrol_internal.allowoutputdecoupling;
			else
				allowoutputdecoupling = [];
			end
		end

		function [objective] = get.objective(this)
			%OBJECTIVE getter for objectiveoptions
			%	Input:
			%		this:		instance
			%	Output:
			%		objective:	structure with objectiveoptions
			if isstruct(this.objective_internal)
				objective = this.objective_internal;
			else
				objective = struct();
			end
		end

		function [preventnan] = get.preventNaN(this)
			%PREVENTNAN getter for indicator if objective function should not retun NaN
			%	Input:
			%		this:			instance
			%	Output:
			%		preventnan:		indicator if objective function should not return NaN
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'preventNaN')
				preventnan = this.objective_internal.preventNaN;
			else
				preventnan = [];
			end
		end

		function [rho] = get.rho(this)
			%RHO getter for kreisselmeier objectiveoption rho
			%	Input:
			%		this:			instance
			%	Output:
			%		rho:			kreisselmeier objective parameter rho
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'kreisselmeier')
				if isstruct(this.objective_internal.kreisselmeier) && isfield(this.objective_internal.kreisselmeier, 'rho')
					rho = this.objective_internal.kreisselmeier.rho;
				else
					rho = [];
				end
			else
				rho = [];
			end
		end

		function [max] = get.max(this)
			%MAX getter for kreisselmeier objectiveoption max
			%	Input:
			%		this:			instance
			%	Output:
			%		max:			kreisselmeier objective parameter max
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'kreisselmeier')
				if isstruct(this.objective_internal.kreisselmeier) && isfield(this.objective_internal.kreisselmeier, 'max')
					max = this.objective_internal.kreisselmeier.max;
				else
					max = [];
				end
			else
				max = [];
			end
		end

		function [Q] = get.Q(this)
			%Q getter for lyapunov objectiveoption Q
			%	Input:
			%		this:			instance
			%	Output:
			%		Q:				lyapunov objective parameter Q
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'lyapunov')
				if isstruct(this.objective_internal.lyapunov) && isfield(this.objective_internal.lyapunov, 'Q')
					Q = this.objective_internal.lyapunov.Q;
				else
					Q = [];
				end
			else
				Q = [];
			end
		end

		function [R] = get.R(this)
			%R getter for normgain objectiveoption R
			%	Input:
			%		this:			instance
			%	Output:
			%		R:				normgain objective parameter R
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'normgain')
				if isstruct(this.objective_internal.normgain) && isfield(this.objective_internal.normgain, 'R')
					R = this.objective_internal.normgain.R;
				else
					R = [];
				end
			else
				R = [];
			end
		end

		function [R_shift] = get.R_shift(this)
			%R_SHIFT getter for normgain objectiveoption R_shift
			%	Input:
			%		this:			instance
			%	Output:
			%		R_shift:		normgain objective parameter R_shift
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'normgain')
				if isstruct(this.objective_internal.normgain) && isfield(this.objective_internal.normgain, 'R_shift')
					R_shift = this.objective_internal.normgain.R_shift;
				else
					R_shift = [];
				end
			else
				R_shift = [];
			end
		end

		function [K] = get.K(this)
			%K getter for normgain objectiveoption K
			%	Input:
			%		this:			instance
			%	Output:
			%		K:				normgain objective parameter K
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'normgain')
				if isstruct(this.objective_internal.normgain) && isfield(this.objective_internal.normgain, 'K')
					K = this.objective_internal.normgain.K;
				else
					K = [];
				end
			else
				K = [];
			end
		end

		function [K_shift] = get.K_shift(this)
			%K_SHIFT getter for normgain objectiveoption K_shift
			%	Input:
			%		this:			instance
			%	Output:
			%		K_shift:		normgain objective parameter K_shift
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'normgain')
				if isstruct(this.objective_internal.normgain) && isfield(this.objective_internal.normgain, 'K_shift')
					K_shift = this.objective_internal.normgain.K_shift;
				else
					K_shift = [];
				end
			else
				K_shift = [];
			end
		end

		function [F] = get.F(this)
			%F getter for normgain objectiveoption F
			%	Input:
			%		this:			instance
			%	Output:
			%		F:				normgain objective parameter F
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'normgain')
				if isstruct(this.objective_internal.normgain) && isfield(this.objective_internal.normgain, 'F')
					F = this.objective_internal.normgain.F;
				else
					F = [];
				end
			else
				F = [];
			end
		end

		function [F_shift] = get.F_shift(this)
			%F_SHIFT getter for normgain objectiveoption F_shift
			%	Input:
			%		this:			instance
			%	Output:
			%		F_shift:		normgain objective parameter F_shift
			if isstruct(this.objective_internal) && isfield(this.objective_internal, 'normgain')
				if isstruct(this.objective_internal.normgain) && isfield(this.objective_internal.normgain, 'F_shift')
					F_shift = this.objective_internal.normgain.F_shift;
				else
					F_shift = [];
				end
			else
				F_shift = [];
			end
		end

		function [allownegativeweight] = get.allownegativeweight(this)
			%ALLOWNEGATIVEWEIGHT getter for indicator if negative pole area weights are allowed
			%	Input:
			%		this:					instance
			%	Output:
			%		allownegativeweight:	indicator if negative pole area weights are allowed
			allownegativeweight = this.allownegativeweight_internal;
		end

		function [strategy] = get.strategy(this)
			%STRATEGY getter for solver strategy
			%	Input:
			%		this:			instance
			%	Output:
			%		strategy:		strategy for solving problem
			strategy = this.strategy_internal;
		end

		function [errorhandler] = get.errorhandler(this)
			%ERRORHANDLER getter for error handler
			%	Input:
			%		this:			instance
			%	Output:
			%		errorhandler:	type of error handler to use
			errorhandler = this.errorhandler_internal;
		end

		function [errorhandler_function] = get.errorhandler_function(this)
			%ERRORHANDLER_FUNCTION getter for error handler function
			%	Input:
			%		this:					instance
			%	Output:
			%		errorhandler_function:	error handler function to use
			errorhandler_function = this.errorhandler_function_internal;
		end

		function [system] = get.system(this)
			%SYSTEM getter for systemoptions
			%	Input:
			%		this:		instance
			%	Output:
			%		system:		structure with systemoptions
			if isstruct(this.system_internal)
				system = this.system_internal;
			else
				system = struct();
			end
		end

		function [usereferences] = get.usereferences(this)
			%USEREFERENCES getter for systemoption usereferences
			%	Input:
			%		this:					instance
			%	Output:
			%		usereferences:			indicator if references should be used
			if isstruct(this.system_internal) && isfield(this.system_internal, 'usereferences')
				usereferences = this.system_internal.usereferences;
			else
				usereferences = [];
			end
		end

		function [usemeasurements_xdot] = get.usemeasurements_xdot(this)
			%USEMEASUREMENTS_XDOT getter for systemoption usemeasurements_xdot
			%	Input:
			%		this:					instance
			%	Output:
			%		usemeasurements_xdot:	indicator if derivative measurements should be used
			if isstruct(this.system_internal) && isfield(this.system_internal, 'usemeasurements_xdot')
				usemeasurements_xdot = this.system_internal.usemeasurements_xdot;
			else
				usemeasurements_xdot = [];
			end
		end

		function [samples] = get.samples(this)
			%SAMPLES getter for systemoption samples
			%	Input:
			%		this:		instance
			%	Output:
			%		samples:	structure with number of samples
			if isstruct(this.system_internal) && isfield(this.system_internal, 'samples')
				samples = this.system_internal.samples;
			else
				samples = [];
			end
		end

		function [Blocks] = get.Blocks(this)
			%BLOCKS getter for systemoption Blocks
			%	Input:
			%		this:		instance
			%	Output:
			%		Blocks:		structure with number of samples
			Blocks = this.samples;
		end

		function [] = set.usecompiled(this, usecompiled)
			%USECOMPILED setter for usecompiled property
			%	Input:
			%		this:			instance
			%		usecompiled:	indicator if compiled code should be used
			this.usecompiled_internal = this.checkProperty('usecompiled', usecompiled);
			this.usecompiled_user = true;
		end

		function [] = set.numthreads(this, numthreads)
			%NUMTHREADS setter for numthreads property
			%	Input:
			%		this:		instance
			%		numthreads:	number of parallel threads to use
			this.numthreads_internal = this.checkProperty('numthreads', numthreads);
			this.numthreads_user = true;
		end

		function [] = set.type(this, type)
			%TYPE setter for objective type property
			%	Input:
			%		this:		instance
			%		type:		objective type to use
			this.type_internal = this.checkProperty('type', type);
			this.type_user = true;
		end

		function [] = set.weight(this, weight)
			%WEIGHT setter for objective weight property
			%	Input:
			%		this:		instance
			%		weight:		objective weight to use
			this.weight_internal = this.checkProperty('weight', weight);
			this.weight_user = true;
		end

		function [] = set.allowvarorder(this, allowvarorder)
			%ALLOWVARORDER setter for indicator for systems with different order
			%	Input:
			%		this:				instance
			%		allowvarorder:		indicator if systems of different order are allowed
			this.allowvarorder_internal = this.checkProperty('allowvarorder', allowvarorder);
			this.allowvarorder_user = true;
		end

		function [] = set.eigenvaluederivative(this, eigenvaluederivative)
			%EIGENVALUEDERIVATIVE setter for type of eigenvalue derivative method to use
			%	Input:
			%		this:						instance
			%		eigenvaluederivative:		type of eigenvalue derivative method to use
			this.eigenvaluederivative_internal = this.checkProperty('eigenvaluederivative', eigenvaluederivative);
			this.eigenvaluederivative_user = true;
		end

		function [] = set.eigenvaluefilter(this, eigenvaluefilter)
			%EIGENVALUEFILTER setter for type of filter method to use for eigenvalues
			%	Input:
			%		this:					instance
			%		eigenvaluefilter:		type of filter method to use for eigenvalues
			this.eigenvaluefilter_internal = this.checkProperty('eigenvaluefilter', eigenvaluefilter);
			this.eigenvaluefilter_user = true;
		end

		function [] = set.eigenvalueignoreinf(this, eigenvalueignoreinf)
			%EIGENVALUEIGNOREINF setter for indicator to ignore inifite eigenvalues
			%	Input:
			%		this:					instance
			%		eigenvalueignoreinf:	indicator to ignore inifite eigenvalues
			this.eigenvalueignoreinf_internal = this.checkProperty('eigenvalueignoreinf', eigenvalueignoreinf);
			this.eigenvalueignoreinf_user = true;
		end

		function [] = set.decouplingcontrol(this, decouplingcontrol)
			%OBJECTIVE setter for decoupling controller design options
			%	Input:
			%		this:				instance
			%		decouplingcontrol:	structure with decoupling controller design options
			this.useoptions(struct('decouplingcontrol', decouplingcontrol));
		end

		function [] = set.tf_structure(this, tf_structure)
			%TF_STRUCTURE setter for the structure of the transfer matrix
			%	Input:
			%		this:			instance
			%		tf_structure:	structure of the transfer matrix
			this.decouplingcontrol_internal.tf_structure = this.checkProperty('tf_structure', tf_structure);
			this.decouplingcontrol_user.tf_structure = true;
		end

		function [] = set.decouplingstrategy(this, strategy)
			%DECOUPLINGSTRATEGY setter for decoupling controller design strategy
			%	Input:
			%		this:		instance
			%		strategy:	decoupling controller design strategy
			this.decouplingcontrol_internal.decouplingstrategy = this.checkProperty('decouplingstrategy', strategy);
			this.decouplingcontrol_user.decouplingstrategy = true;
		end

		function [] = set.sortingstrategy_decoupling(this, strategy)
			%SORTINGSTRATEGY_DECOUPLING setter for decoupling condition sorting strategy
			%	Input:
			%		this:		instance
			%		strategy:	decoupling condition sorting strategy
			this.decouplingcontrol_internal.sortingstrategy_decoupling = this.checkProperty('sortingstrategy_decoupling', strategy);
			this.decouplingcontrol_user.sortingstrategy_decoupling = true;
		end

		function [] = set.weight_decoupling(this, weight_decoupling)
			%WEIGHT_DECOUPLING setter for weight of decoupling condition constraints
			%	Input:
			%		this:				instance
			%		weight_decoupling:	weight of decoupling condition constraints
			this.decouplingcontrol_internal.weight_decoupling = this.checkProperty('weight_decoupling', weight_decoupling);
			this.decouplingcontrol_user.weight_decoupling = true;
		end

		function [] = set.weight_prefilter(this, weight_prefilter)
			%WEIGHT_PREFILTER setter for weight of prefilter decoupling condition constraints
			%	Input:
			%		this:				instance
			%		weight_prefilter:	weight of prefilter decoupling condition constraints
			this.decouplingcontrol_internal.weight_prefilter = this.checkProperty('weight_prefilter', weight_prefilter);
			this.decouplingcontrol_user.weight_prefilter = true;
		end

		function [] = set.tolerance_decoupling(this, tolerance_decoupling)
			%TOLERNACE_DECOUPLING setter for tolerance of decoupling condition constraints
			%	Input:
			%		this:				instance
			%		tolerance_decoupling:	tolerance of decoupling condition constraints
			this.decouplingcontrol_internal.tolerance_decoupling = this.checkProperty('tolerance_decoupling', tolerance_decoupling);
			this.decouplingcontrol_user.tolerance_decoupling = true;
		end

		function [] = set.tolerance_prefilter(this, tolerance_prefilter)
			%TOLERNACE_PREFILTER setter for tolerance of prefilter regularization constraint
			%	Input:
			%		this:				instance
			%		tolerance_prefilter:	tolerance of prefilter regularization constraint
			this.decouplingcontrol_internal.tolerance_prefilter = this.checkProperty('tolerance_prefilter', tolerance_prefilter);
			this.decouplingcontrol_user.tolerance_prefilter = true;
		end

		function [] = set.solvesymbolic(this, solvesymbolic)
			%SOLVESYMBOLIC setter for tolerance of prefilter regularization constraint
			%	Input:
			%		this:			instance
			%		solvesymbolic:	indicator if symbolic calculations should be used
			this.decouplingcontrol_internal.solvesymbolic = this.checkProperty('solvesymbolic', solvesymbolic);
			this.decouplingcontrol_user.solvesymbolic = true;
		end

		function [] = set.round_equations_to_digits(this, round_equations_to_digits)
			%ROUND_EQUATIONS_TO_DIGITS setter for tolerance of prefilter regularization constraint
			%	Input:
			%		this:						instance
			%		round_equations_to_digits:	indicator to how many decimal places systems of equations should be rounded in case of numerical difficulties
			this.decouplingcontrol_internal.round_equations_to_digits = this.checkProperty('round_equations_to_digits', round_equations_to_digits);
			this.decouplingcontrol_user.round_equations_to_digits = true;
		end

		function [] = set.allowoutputdecoupling(this, allowoutputdecoupling)
			%ALLOWOUPUTDECOUPLING setter for indicator whether output feedback is allowed for decoupling control
			%	Input:
			%		this:					instance
			%		allowoutputdecoupling:	indicator whether output feedback is allowed for decoupling control
			this.decouplingcontrol_internal.allowoutputdecoupling = this.checkProperty('allowoutputdecoupling', allowoutputdecoupling);
			this.decouplingcontrol_user.allowoutputdecoupling = true;
		end

		function [] = set.objective(this, objective)
			%OBJECTIVE setter for objectiveoptions
			%	Input:
			%		this:		instance
			%		objective:	structure with objectiveoptions
			this.useoptions(struct('objective', objective));
		end

		function [] = set.preventNaN(this, preventNaN)
			%PREVENTNAN setter for indicator to prevent NaN in objective function
			%	Input:
			%		this:			instance
			%		preventNaN:		indicator if NaN should be prevented in objective function
			this.objective_internal.preventNaN = this.checkProperty('preventNaN', preventNaN);
			this.objective_user.preventNaN = true;
		end

		function [] = set.rho(this, rho)
			%RHO setter for kreisselmeiser objectiveoption rho
			%	Input:
			%		this:			instance
			%		rho:			kreisselmeier objectiveoption rho
			this.objective_internal.kreisselmeier.rho = this.checkProperty('rho', rho);
			this.objective_user.kreisselmeier.rho = true;
		end

		function [] = set.max(this, max)
			%MAX setter for kreisselmeiser objectiveoption max
			%	Input:
			%		this:			instance
			%		max:			kreisselmeier objectiveoption max
			this.objective_internal.kreisselmeier.max = this.checkProperty('max', max);
			this.objective_user.kreisselmeier.max = true;
		end

		function [] = set.Q(this, Q)
			%Q setter for lyapunov objectiveoption Q
			%	Input:
			%		this:			instance
			%		Q:				lyapunov objectiveoption Q
			this.objective_internal.lyapunov.Q = this.checkProperty('Q', Q);
			this.objective_user.lyapunov.Q = true;
		end

		function [] = set.R(this, R)
			%R setter for normgain objectiveoption R
			%	Input:
			%		this:			instance
			%		R:				normgain objectiveoption R
			this.objective_internal.normgain.R = this.checkProperty('R', R);
			this.objective_user.normgain.R = true;
		end

		function [] = set.R_shift(this, R_shift)
			%R_SHIFT setter for normgain objectiveoption R_shift
			%	Input:
			%		this:			instance
			%		R_shift:		normgain objectiveoption R_shift
			this.objective_internal.normgain.R_shift = this.checkProperty('R_shift', R_shift);
			this.objective_user.normgain.R_shift = true;
		end

		function [] = set.K(this, K)
			%K setter for normgain objectiveoption K
			%	Input:
			%		this:			instance
			%		K:				normgain objectiveoption K
			this.objective_internal.normgain.K = this.checkProperty('K', K);
			this.objective_user.normgain.K = true;
		end

		function [] = set.K_shift(this, K_shift)
			%K_SHIFT setter for normgain objectiveoption K_shift
			%	Input:
			%		this:			instance
			%		K_shift:		normgain objectiveoption K_shift
			this.objective_internal.normgain.K_shift = this.checkProperty('K_shift', K_shift);
			this.objective_user.normgain.K_shift = true;
		end

		function [] = set.F(this, F)
			%F setter for normgain objectiveoption F
			%	Input:
			%		this:			instance
			%		F:				normgain objectiveoption F
			this.objective_internal.normgain.F = this.checkProperty('F', F);
			this.objective_user.normgain.F = true;
		end

		function [] = set.F_shift(this, F_shift)
			%F_SHIFT setter for normgain objectiveoption F_shift
			%	Input:
			%		this:			instance
			%		F_shift:		normgain objectiveoption F_shift
			this.objective_internal.normgain.F_shift = this.checkProperty('F_shift', F_shift);
			this.objective_user.normgain.F_shift = true;
		end

		function [] = set.allownegativeweight(this, allownegativeweight)
			%ALLOWNEGATIVEWEIGHT setter for indicator if negative pole area weights are allowed
			%	Input:
			%		this:					instance
			%		allownegativeweight:	indicator if negative pole area weights are allowed
			this.allownegativeweight_internal = this.checkProperty('allownegativeweight', allownegativeweight);
			this.allownegativeweight_user = true;
		end

		function [] = set.strategy(this, strategy)
			%STRATEGY setter for solver strategy
			%	Input:
			%		this:			instance
			%		strategy:		strategy for solving problem
			this.strategy_internal = this.checkProperty('strategy', strategy);
			this.strategy_user = true;
		end

		function [] = set.errorhandler(this, errorhandler)
			%ERRORHANDLER setter for error handler
			%	Input:
			%		this:			instance
			%		errorhandler:	type of error handler to use
			this.errorhandler_internal = this.checkProperty('errorhandler', errorhandler);
			this.errorhandler_user = true;
		end

		function [] = set.errorhandler_function(this, errorhandler_function)
			%ERRORHANDLER_FUNCTION setter for error handler function
			%	Input:
			%		this:					instance
			%		errorhandler_function:	error handler function to use
			this.errorhandler_function_internal = this.checkProperty('errorhandler_function', errorhandler_function);
			this.errorhandler_function_user = true;
		end

% 		function [] = set.system_internal(this, system)
% 			%SYSTEM setter for systemoptions
% 			%	Input:
% 			%		this:		instance
% 			%		system:		structure with systemoptions
% 			if isempty(system)
% 				this.system_internal = struct(...
% 					'usereferences',		true,...
% 					'usemeasurements_xdot',	true,...
% 					'samples',				struct(),...
% 					'Blocks',				struct()...
% 				);
% 				this.system_user = struct(...
% 					'usereferences',		false,...
% 					'usemeasurements_xdot',	false,...
% 					'samples',				false,...
% 					'Blocks',				false...
% 				);
% 				return;
% 			end
% 			if ~isstruct(system)
% 				error();
% 			end
% 			names = fieldnames(system);
% 			if isfield(system, 'usereferences')
% 				this.system_internal.usereferences = this.checkProperty('usereferences', system.usereferences);
% 				this.system_user.usereferences = true;
% 				names(strcmp('usereferences', names)) = [];
% 			end
% 			if isfield(system, 'usemeasurements_xdot')
% 				this.system_internal.usemeasurements_xdot = this.checkProperty('usemeasurements_xdot', system.usemeasurements_xdot);
% 				this.system_user.usemeasurements_xdot = true;
% 				names(strcmp('usemeasurements_xdot', names)) = [];
% 			end
% 			if isfield(system, 'samples')
% 				this.system_internal.samples = this.checkProperty('samples', system.samples);
% 				this.system_user.samples = true;
% 				names(strcmp('samples', names)) = [];
% 			end
% 			if isfield(system, 'Blocks')
% 				this.system_internal.Blocks = this.checkProperty('Blocks', system.Blocks);
% 				this.system_user.Blocks = true;
% 				names(strcmp('Blocks', names)) = [];
% 			end
% 			if ~isempty(names)
% 				error();
% 			end
% 		end

		function [] = set.system(this, system)
			%SYSTEM setter for systemoptions
			%	Input:
			%		this:		instance
			%		system:		structure with systemoptions
			this.useoptions(struct('system', system));
		end

		function [] = set.usereferences(this, usereferences)
			%USEREFERENCES setter for systemoption usereferences
			%	Input:
			%		this:					instance
			%		usereferences:			indicator if references should be used
			this.system_internal.usereferences = this.checkProperty('usereferences', usereferences);
			this.system_user.usereferences = true;
		end

		function [] = set.usemeasurements_xdot(this, usemeasurements_xdot)
			%USEMEASUREMENTS_XDOT setter for systemoption usemeasurements_xdot
			%	Input:
			%		this:					instance
			%		usemeasurements_xdot:	indicator if derivative measurements should be used
			this.system_internal.usemeasurements_xdot = this.checkProperty('usemeasurements_xdot', usemeasurements_xdot);
			this.system_user.usemeasurements_xdot = true;
		end

		function [] = set.samples(this, samples)
			%SAMPLES setter for systemoption samples
			%	Input:
			%		this:		instance
			%		samples:	structure with number of samples
			this.system_internal.samples = this.checkProperty('samples', samples);
			this.system_user.samples = true;
		end

		function [] = set.Blocks(this, Blocks)
			%BLOCKS setter for systemoption Blocks
			%	Input:
			%		this:		instance
			%		Blocks:		structure with number of samples
			this.samples = Blocks;
		end
	end

	methods(Access=protected)
		function [options] = available_options(~)
			%AVAILABLE_OPTIONS cell array with information about available options
			%	Input:
			%		this:	instance
			%	Output:
			%		option:	cell array with information about options
			options = {
				% name,							isroot,	codegen,	path,								catchall
				'usecompiled',					true,	true,		{},									false;
				'numthreads',					true,	true,		{},									false;
				'type',							true,	true,		{},									false;
				'weight',						true,	true,		{},									false;
				'allowvarorder',				true,	true,		{},									false;
				'eigenvaluederivative',			true,	true,		{},									false;
				'eigenvaluefilter',				true,	true,		{},									false;
				'eigenvalueignoreinf',			true,	true,		{},									false;
				'tf_structure',					false,	true,		{'decouplingcontrol'},				false;
				'decouplingstrategy',			false,	true,		{'decouplingcontrol'},				false;
				'sortingstrategy_decoupling',	false,	true,		{'decouplingcontrol'},				false;
				'weight_decoupling',			false,	true,		{'decouplingcontrol'},				false;
				'weight_prefilter',				false,	true,		{'decouplingcontrol'},				false;
				'tolerance_decoupling',			false,	true,		{'decouplingcontrol'},				false;
				'tolerance_prefilter',			false,	true,		{'decouplingcontrol'},				false;
				'solvesymbolic',				false,	true,		{'decouplingcontrol'},				false;
				'round_equations_to_digits',	false,	true,		{'decouplingcontrol'},				false;
				'allowoutputdecoupling',		false,	true,		{'decouplingcontrol'},				false;
				'preventNaN',					false,	true,		{'objective'},						false;
				'rho',							false,	true,		{'objective', 'kreisselmeier'},		false;
				'max',							false,	true,		{'objective', 'kreisselmeier'},		false;
				'Q',							false,	true,		{'objective', 'lyapunov'},			false;
				'R',							false,	true,		{'objective', 'normgain'},			false;
				'R_shift',						false,	true,		{'objective', 'normgain'},			false;
				'K',							false,	true,		{'objective', 'normgain'},			false;
				'K_shift',						false,	true,		{'objective', 'normgain'},			false;
				'F',							false,	true,		{'objective', 'normgain'},			false;
				'F_shift',						false,	true,		{'objective', 'normgain'},			false;
				'allownegativeweight',			true,	false,		{},									false;
				'strategy',						true,	false,		{},									false;
				'errorhandler',					true,	false,		{},									false;
				'errorhandler_function',		true,	false,		{},									false;
				'usereferences',				false,	false,		{'system'},							false;
				'usemeasurements_xdot',			false,	false,		{'system'},							false;
				'samples',						false,	false,		{'system'},							true;
				'Blocks',						false,	false,		{'system'},							true;
			};
			rootpath = cellfun(@getfirst, options(:, 4), 'UniformOutput', false);
			options(:, end + 1) = rootpath;
		end

		function [] = alldefault(this)
			%ALLDEFAULT set indicator for all options to 'not set by user'
			%	Input:
			%		this:	instance
			this.usecompiled_user = false;
			this.numthreads_user = false;
			this.type_user = false;
			this.weight_user = false;
			this.allowvarorder_user = false;
			this.eigenvaluederivative_user = false;
			this.eigenvaluefilter_user = false;
			this.eigenvalueignoreinf_user = false;
			this.decouplingcontrol_user = struct(...
				'tf_structure',					false,...
				'decouplingstrategy',			false,...
				'sortingstrategy_decoupling',	false,...
				'weight_decoupling',			false,...
				'weight_prefilter',				false,...
				'tolerance_decoupling',			false,...
				'tolerance_prefilter',			false,...
				'solvesymbolic',				false,...
				'round_equations_to_digits',	false,...
				'allowoutputdecoupling',		false...
			);
			this.objective_user = struct(...
				'preventNaN',		false,...
				'kreisselmeier',	struct(...
					'rho',	false,...
					'max',	false...
				),...
				'lyapunov',	struct(...
					'Q',	false...
				),...
				'normgain', struct(...
					'R',		false,...
					'R_shift',	false,...
					'K',		false,...
					'K_shift',	false,...
					'F',		false,...
					'F_shift',	false...
				)...
			);
			this.allownegativeweight_user = false;
			this.strategy_user = false;
			this.errorhandler_user = false;
			this.errorhandler_function_user = false;
			this.system_user = struct(...
				'usereferences',		false,...
				'usemeasurements_xdot',	false,...
				'samples',				false,...
				'Blocks',				false...
			);
		end

		function [] = use_default(this)
			%USE_DEFAULT use default option for all options
			%	Input:
			%		this:	instance
			proto = this.PROTOTYPE;
			this.usecompiled_internal = proto.usecompiled;
			this.numthreads_internal = proto.numthreads;
			this.type_internal = proto.type;
			this.weight_internal = proto.weight;
			this.allowvarorder_internal = proto.allowvarorder;
			this.eigenvaluederivative_internal = proto.eigenvaluederivative;
			this.eigenvaluefilter_internal = proto.eigenvaluefilter;
			this.eigenvalueignoreinf_internal = proto.eigenvalueignoreinf;
			this.decouplingcontrol_internal = struct(...
				'tf_structure',					proto.decouplingcontrol.tf_structure,...
				'decouplingstrategy',			proto.decouplingcontrol.decouplingstrategy,...
				'sortingstrategy_decoupling',	proto.decouplingcontrol.sortingstrategy_decoupling,...
				'weight_decoupling',			proto.decouplingcontrol.weight_decoupling,...
				'weight_prefilter',				proto.decouplingcontrol.weight_prefilter,...
				'tolerance_decoupling',			proto.decouplingcontrol.tolerance_decoupling,...
				'tolerance_prefilter',			proto.decouplingcontrol.tolerance_prefilter,...
				'solvesymbolic',				proto.decouplingcontrol.solvesymbolic,...
				'round_equations_to_digits',	proto.decouplingcontrol.round_equations_to_digits,...
				'allowoutputdecoupling',		proto.decouplingcontrol.allowoutputdecoupling...
			);
			this.objective_internal = struct(...
				'preventNaN',		proto.objective.preventNaN,...
				'kreisselmeier',	struct(...
					'rho',	proto.objective.kreisselmeier.rho,...
					'max',	proto.objective.kreisselmeier.max...
				),...
				'lyapunov',	struct(...
					'Q',	proto.objective.lyapunov.Q...
				),...
				'normgain', struct(...
					'R',		proto.objective.normgain.R,...
					'R_shift',	proto.objective.normgain.R_shift,...
					'K',		proto.objective.normgain.K,...
					'K_shift',	proto.objective.normgain.K_shift,...
					'F',		proto.objective.normgain.F,...
					'F_shift',	proto.objective.normgain.F_shift...
				)...
			);
			this.allownegativeweight_internal = proto.allownegativeweight;
			this.strategy_internal = proto.strategy;
			this.errorhandler_internal = proto.errorhandler;
			this.errorhandler_function_internal = proto.errorhandler_function;
			this.system_internal = struct(...
				'usereferences',		proto.system.usereferences,...
				'usemeasurements_xdot',	proto.system.usemeasurements_xdot,...
				'samples',				struct(),...
				'Blocks',				struct()...
			);
			this.alldefault();
		end

		function [] = use_options(this, options)
			%USE_OPTIONS set options to values in supplied data structure with options, where non existent options are ignored
			%	Input:
			%		this:		instance
			%		options:	structure with option or ObjectiveOptions object to copy
			names = this.options;
			if isstruct(options)
				names_toplevel = names(:, 1);
				names_toplevel(~[names{:, 2}], 1) = names(~[names{:, 2}], 6);
				name = fieldnames(options);
				if isempty(name)
					return;
				end
				[properties, idx] = ismember(names_toplevel, name);
				foundnames = names(properties, :);
				%idx = idx(properties);
				for ii = 1:size(foundnames, 1)
					if isempty(foundnames{ii, 4})
						this.(foundnames{ii, 1}) = options.(foundnames{ii, 1});
					else
						default_set = [];
						value = this.([foundnames{ii, 6}, '_internal']);
						if ~isstruct(value)
							value = struct();
						end
						setvalue = options.(foundnames{ii, 6});
						if ~isstruct(setvalue)
							continue;
						end
						tempstruct = value;
						tempstruct_set = setvalue;
						path = [foundnames{ii, 4}(2:end), foundnames{ii, 1}];
						hierarchy = cell(length(path), 2);
						wasadded = false;
						wasfound = false;
						for jj = 1:length(path)
							if isstruct(tempstruct_set) && isfield(tempstruct_set, path{jj})
								hierarchy{jj, 2} = tempstruct_set;
								tempstruct_set = tempstruct_set.(path{jj});
								if jj == length(path)
									default_set = tempstruct_set;
									wasfound = true;
								end
							elseif isobject(tempstruct_set) && isprop(tempstruct_set, path{jj})
								hierarchy{jj, 2} = tempstruct_set;
							else
								if jj == length(path) && isstruct(tempstruct_set) && ~isfield(tempstruct_set, path{jj})
									continue;
								end
								if isstruct(tempstruct_set) && ~wasadded
									hierarchy{jj, 2} = tempstruct_set;
									wasadded = true;
									tempstruct_set = struct();
								elseif isobject(tempstruct_set) && ~wasadded
									hierarchy{jj, 2} = tempstruct_set;
									wasadded = true;
								elseif jj ~= length(path)
									hierarchy{jj, 2} = struct();
									tempstruct_set = struct();
								else
									continue;
								end
							end
							if isstruct(tempstruct) && isfield(tempstruct, path{jj})
								hierarchy{jj, 1} = tempstruct;
								tempstruct = tempstruct.(path{jj});
							elseif isobject(tempstruct) && isprop(tempstruct, path{jj})
								hierarchy{jj, 1} = tempstruct;
							else
								if isstruct(tempstruct) && ~wasadded
									hierarchy{jj, 1} = tempstruct;
									wasadded = true;
									tempstruct = struct();
								elseif isobject(tempstruct) && ~wasadded
									hierarchy{jj, 1} = tempstruct;
									wasadded = true;
								elseif jj ~= length(path)
									hierarchy{jj, 1} = struct();
									tempstruct = struct();
								end
							end
						end
						if ~wasfound
							continue;
						end
						tempstruct = default_set;
						for jj = size(hierarchy, 1):-1:1
							hierarchy{jj, 1}.(path{jj}) = tempstruct;
							tempstruct = hierarchy{jj, 1};
						end
						this.([foundnames{ii, 6}, '_internal']) = tempstruct;
						this.([foundnames{ii, 6}, '_user']) = setstruct(this.([foundnames{ii, 4}{1}, '_user']), [foundnames{ii, 4}(2:end), foundnames{ii, 1}], true);
					end
				end
			elseif isa(options, 'control.design.gamma.GammasynOptions')
				this.useoptions(options);
			else
				error('control:design:gamma:GammasynOptions:input', 'Options must be of type ''struct'' or ''ObjectiveOptions'', not ''%s''.', class(options));
			end
		end

		function [value, validvalue, errmsg, errid] = checkProperty(this, name, value)
			%CHECKPROPERTY set a value for an objective option
			%	Input:
			%		this:			instance
			%		name:			name of option to set
			%		value:			value to set
			%	Output:
			%		value:			value to set
			%		validvalue:		indicator, if value is valid
			%		errmsg:			error message, if value is not valid
			%		errid:			error identifier, if value is not valid
			if isempty(value) && isnumeric(value)
				validvalue = true;
				errmsg = '';
				errid = '';
				return;
			end
			if ischar(value)
				value = deblank(value);
			end
			[value, validvalue, errmsg, errid] = objectiveoptions_checkoption(this, name, value);
			if ~validvalue
				ME = MException(errid, '%s', errmsg);
				throwAsCaller(ME);
			end
		end

% 		function [value] = convertProperty(this, name, value, optionstype, replacechar)
% 			%CONVERTPROPERTY convert a property to the specified option type
% 			%	Input:
% 			%		this:			instance
% 			%		name:			name of the option to convert
% 			%		value:			value of the option to convert
% 			%		optionstype:	type of option set, the value is returned for
% 			%		replacechar:	indicator, if char options should be replaced by corresponding numerical values
% 			%	Output:
% 			%		value:			value to set
% 			if nargin <= 4
% 				replacechar = ~isempty(this.NumberVariables);
% 			end
% 			if ischar(value)
% 				value = lower(deblank(value));
% 			end
% 			replaceallowed = any(strcmpi(name, this.VARIABLESIZEALLOWED));
% 			if replaceallowed && replacechar && (isempty(this.NumberVariables) || isnan(this.NumberVariables))
% 				error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
% 			end
% 			value = convertfield(this, name, value, optionstype, replacechar && replaceallowed);
% 		end

		function [options] = get_user(this)
			%GET_USER get all options set by th user
			%	Input:
			%		this:		instance
			%	Output:
			%		options:	structure with options set by user
			% TODO: move to temporary variable
			options = struct();
			available_options = this.options;
			names = available_options;
			for ii = 1:size(names, 1) %#ok<FORPF> no parfor because order of fields matters
				if isempty(names{ii, 4})
					if this.([names{ii, 1}, '_user'])
						options.(names{ii, 1}) = this.([names{ii, 1}, '_internal']);
					end
				else
					if hasstructpath(this.([names{ii, 6}, '_user']), [names{ii, 4}(2:end), names{ii, 1}])
						if getstruct(this.([names{ii, 6}, '_user']), [names{ii, 4}(2:end), names{ii, 1}])
							options = isset(options, [names{ii, 4}, names{ii, 1}], getstruct(this.([names{ii, 4}{1}, '_internal']), [names{ii, 4}(2:end), names{ii, 1}]));
						end
					end
				end
			end
		end
	end

	methods
		function [options] = struct(this)
			%STRUCT convert object to structure
			%	Input:
			%		this:	instance
			%	Output:
			%		opions:	structure representation of the object
			% TODO: move to temporary variable
			options = struct();
			available_options = this.options;
			names = available_options;
			for ii = 1:size(names, 1) %#ok<FORPF> no parfor because order of fields matters
				if isempty(names{ii, 4})
					options.(names{ii, 1}) = [];
				else
					options = isset(options, [names{ii, 4}, names{ii, 1}], []);
				end
			end
			options(numel(this)) = options;
			for jj = 1:numel(this) %#ok<FORPF> no parfor because of struct indexing
				for ii = 1:size(names, 1)
					if isempty(names{ii, 4})
						options(jj).(names{ii, 1}) = this(jj).([names{ii, 1}, '_internal']);
					else
						options(jj) = setstruct(options(jj), [names{ii, 4}, names{ii, 1}], getstruct(this(jj).([names{ii, 4}{1}, '_internal']), [names{ii, 4}(2:end), names{ii, 1}]));
					end
				end
			end
		end

		function [options] = codegenstruct(this)
			%CODEGENSTRUCT convert object to structure used for code generation
			%	Input:
			%		this:	instance
			%	Output:
			%		opions:	structure representation of the object
			% TODO: move to temporary variable
			options = struct();
			available_options = this.options;
			names = available_options([available_options{:, 3}], :);
			for ii = 1:size(names, 1) %#ok<FORPF> no parfor because order of fields matters
				if isempty(names{ii, 4})
					options.(names{ii, 1}) = this.([names{ii, 1}, '_internal']);
				else
					options = isset(options, [names{ii, 4}, names{ii, 1}], getstruct(this.([names{ii, 4}{1}, '_internal']), [names{ii, 4}(2:end), names{ii, 1}]));
				end
			end
		end

		function [options] = userstruct(this)
			%USERSTRUCT convert object to structure with only fields set by user
			%	Input:
			%		this:	instance
			%	Output:
			%		opions:	structure representation of the object
			% TODO: move to temporary variable?
			options = this.get_user();
		end

		function [this] = subsasgn(this, sub, varargin)
			%SUBSASGN overload subsasgn method to allow for indexing into nested options with argument checking
			%	Input:
			%		this:		instance
			%		sub:		substruct for indexing
			%		varargin:	value to set
			%	Output:
			%		this:		instance
			narginchk(2 + 1, 2 + numel(this));
			assignall = false;
			if strcmp(sub(1).type, '()') && ~strcmp(sub(1).subs, ':') && length(sub) > 1 && strcmp(sub(2).type, '.')
				instance = subsref(this, sub(1));
				oldsub = sub(1);
				sub(1) = [];
			elseif strcmp(sub(1).type, '()') && strcmp(sub(1).subs, ':') && length(sub) > 1 && strcmp(sub(2).type, '.')
				instance = this;
				oldsub = sub(1);
				sub(1) = [];
				assignall = true;
			else
				instance = this;
				oldsub = [];
			end
			narginchk(2 + 1, 2 + numel(instance));
			if strcmp(sub(1).type, '.')
				proto = instance(1).PROTOTYPE;
				for ii = 1:numel(instance)
					% dot indexing for properties
					if strcmpi(sub(1).subs, 'system')
						% 'system' property requested
						systemnames = {
							'usereferences';
							'usemeasurements_xdot';
							'samples';
							'Blocks'
						};
						if length(sub) > 1
							% deeper indexing requested
							if strcmp(sub(2).type, '()')
								% this.system(...) indexing type
								if length(sub(2).subs) == 1
									% this.system(:) and this.system(1) are ok, others are converted to error by call to subsref
									if ischar(sub(2).subs{1}) && strcmpi(sub(2).subs{1}, ':')
										sub(2) = [];
									elseif isnumeric(sub(2).subs{1}) && isscalar(sub(2).subs{1}) && sub(2).subs{1} == 1
										sub(2) = [];
									else
										try
											subsref(instance(ii).system_internal, sub(2:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''system''.');
									end
								else
									% this.system(:, :, ...) and this.system(1, 1, ....) are ok, others are converted to error by call to subsref
									iscolon = cellfun(@(x) ischar(x) && strcmpi(x, ':'), sub(2).subs, 'UniformOutput', true);
									isone = cellfun(@(x) isscalar(x) && x == 1, sub(2).subs, 'UniformOutput', true);
									if all(iscolon(:))
										sub(2) = [];
									elseif all(isone(:))
										sub(2) = [];
									else
										try
											subsref(instance(ii).system_internal, sub(2:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''system''.');
									end
								end
							end
						end
						if length(sub) > 1
							% setting options under this.system requested
							if strcmp(sub(2).type, '.') && any(strcmpi(sub(2).subs, systemnames))
								% setting this.system.option
								names = {sub(2).subs};
								if strcmp(sub(2).subs, 'usereferences')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).system_internal.usereferences;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).system_internal.usereferences = instance(ii).checkProperty('usereferences', prop);
									instance(ii).system_user.usereferences = true;
									names(strcmp('usereferences', names)) = [];
								end
								if strcmp(sub(2).subs, 'usemeasurements_xdot')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).system_internal.usemeasurements_xdot;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).system_internal.usemeasurements_xdot = instance(ii).checkProperty('usemeasurements_xdot', prop);
									instance(ii).system_user.usemeasurements_xdot = true;
									names(strcmp('usemeasurements_xdot', names)) = [];
								end
								if strcmp(sub(2).subs, 'samples')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).system_internal.samples;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).system_internal.samples = instance(ii).checkProperty('samples', prop);
									instance(ii).system_user.samples = true;
									names(strcmp('samples', names)) = [];
								end
								if strcmp(sub(2).subs, 'Blocks')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).system_internal.Blocks;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).system_internal.Blocks = instance(ii).checkProperty('Blocks', prop);
									instance(ii).system_user.Blocks = true;
									names(strcmp('Blocks', names)) = [];
								end
								if ~isempty(names)
									error('control:design:gamma:GammasynOptions:input', 'Option ''system'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
								end
							else
								% indexing into this.system with {} or ()
								try
									subsref(instance(ii).system_internal, sub(2:end));
								catch e
									error(e.identifier, e.message);
								end
								error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''system''.');
							end
						else
							% assigning to whole this.system structure (partially)
							if isempty(varargin{ii})
								instance(ii).system_internal = struct(...
									'usereferences',		proto.system.usereferences,...
									'usemeasurements_xdot',	proto.system.usemeasurements_xdot,...
									'samples',				proto.system.samples,...
									'Blocks',				proto.system.Blocks...
								);
								instance(ii).system_user = struct(...
									'usereferences',		false,...
									'usemeasurements_xdot',	false,...
									'samples',				false,...
									'Blocks',				false...
								);
								return;
							end
							if ~isstruct(varargin{ii}) || ~isscalar(varargin{ii})
								error('control:design:gamma:GammasynOptions:input', 'Option ''system'' must be a scalar structure.');
							end
							names = fieldnames(varargin{ii});
							if isfield(varargin{ii}, 'usereferences')
								instance(ii).system_internal.usereferences = instance(ii).checkProperty('usereferences', varargin{ii}.usereferences);
								instance(ii).system_user.usereferences = true;
								names(strcmp('usereferences', names)) = [];
							end
							if isfield(varargin{ii}, 'usemeasurements_xdot')
								instance(ii).system_internal.usemeasurements_xdot = instance(ii).checkProperty('usemeasurements_xdot', varargin{ii}.usemeasurements_xdot);
								instance(ii).system_user.usemeasurements_xdot = true;
								names(strcmp('usemeasurements_xdot', names)) = [];
							end
							if isfield(varargin{ii}, 'samples')
								instance(ii).system_internal.samples = instance(ii).checkProperty('samples', varargin{ii}.samples);
								instance(ii).system_user.samples = true;
								names(strcmp('samples', names)) = [];
							end
							if isfield(varargin{ii}, 'Blocks')
								instance(ii).system_internal.Blocks = instance(ii).checkProperty('Blocks', varargin{ii}.Blocks);
								instance(ii).system_user.Blocks = true;
								names(strcmp('Blocks', names)) = [];
							end
							if ~isempty(names)
								error('control:design:gamma:GammasynOptions:input', 'Option ''system'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
							end
						end
					elseif strcmpi(sub(1).subs, 'decouplingcontrol')
						% 'system' property requested
						decouplingcontrolnames = {
							'tf_structure';
							'decouplingstrategy';
							'sortingstrategy_decoupling';
							'weight_decoupling';
							'weight_prefilter';
							'tolerance_decoupling';
							'tolerance_prefilter';
							'solvesymbolic';
							'round_equations_to_digits';
							'allowoutputdecoupling'
						};
						if length(sub) > 1
							% deeper indexing requested
							if strcmp(sub(2).type, '()')
								% this.system(...) indexing type
								if length(sub(2).subs) == 1
									% this.system(:) and this.system(1) are ok, others are converted to error by call to subsref
									if ischar(sub(2).subs{1}) && strcmpi(sub(2).subs{1}, ':')
										sub(2) = [];
									elseif isnumeric(sub(2).subs{1}) && isscalar(sub(2).subs{1}) && sub(2).subs{1} == 1
										sub(2) = [];
									else
										try
											subsref(instance(ii).decouplingcontrol_internal, sub(2:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''decouplingcontrol''.');
									end
								else
									% this.system(:, :, ...) and this.system(1, 1, ....) are ok, others are converted to error by call to subsref
									iscolon = cellfun(@(x) ischar(x) && strcmpi(x, ':'), sub(2).subs, 'UniformOutput', true);
									isone = cellfun(@(x) isscalar(x) && x == 1, sub(2).subs, 'UniformOutput', true);
									if all(iscolon(:))
										sub(2) = [];
									elseif all(isone(:))
										sub(2) = [];
									else
										try
											subsref(instance(ii).decouplingcontrol_internal, sub(2:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''decouplingcontrol''.');
									end
								end
							end
						end
						if length(sub) > 1
							% setting options under this.system requested
							if strcmp(sub(2).type, '.') && any(strcmpi(sub(2).subs, decouplingcontrolnames))
								% setting this.system.option
								names = {sub(2).subs};
								if strcmp(sub(2).subs, 'tf_structure')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.tf_structure;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.tf_structure = instance(ii).checkProperty('tf_structure', prop);
									instance(ii).decouplingcontrol_user.tf_structure = true;
									names(strcmp('tf_structure', names)) = [];
								end
								if strcmp(sub(2).subs, 'decouplingstrategy')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.decouplingstrategy;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.decouplingstrategy = instance(ii).checkProperty('decouplingstrategy', prop);
									instance(ii).decouplingcontrol_user.decouplingstrategy = true;
									names(strcmp('decouplingstrategy', names)) = [];
								end
								if strcmp(sub(2).subs, 'sortingstrategy_decoupling')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.sortingstrategy_decoupling;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.sortingstrategy_decoupling = instance(ii).checkProperty('sortingstrategy_decoupling', prop);
									instance(ii).decouplingcontrol_user.sortingstrategy_decoupling = true;
									names(strcmp('sortingstrategy_decoupling', names)) = [];
								end
								if strcmp(sub(2).subs, 'weight_decoupling')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.weight_decoupling;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.weight_decoupling = instance(ii).checkProperty('weight_decoupling', prop);
									instance(ii).decouplingcontrol_user.weight_decoupling = true;
									names(strcmp('weight_decoupling', names)) = [];
								end
								if strcmp(sub(2).subs, 'weight_prefilter')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.weight_prefilter;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.weight_prefilter = instance(ii).checkProperty('weight_prefilter', prop);
									instance(ii).decouplingcontrol_user.weight_prefilter = true;
									names(strcmp('weight_prefilter', names)) = [];
								end
								if strcmp(sub(2).subs, 'tolerance_decoupling')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.tolerance_decoupling;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.tolerance_decoupling = instance(ii).checkProperty('tolerance_decoupling', prop);
									instance(ii).decouplingcontrol_user.tolerance_decoupling = true;
									names(strcmp('tolerance_decoupling', names)) = [];
								end
								if strcmp(sub(2).subs, 'tolerance_prefilter')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.tolerance_prefilter;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.tolerance_prefilter = instance(ii).checkProperty('tolerance_prefilter', prop);
									instance(ii).decouplingcontrol_user.tolerance_prefilter = true;
									names(strcmp('tolerance_prefilter', names)) = [];
								end
								if strcmp(sub(2).subs, 'solvesymbolic')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.solvesymbolic;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.solvesymbolic = instance(ii).checkProperty('solvesymbolic', prop);
									instance(ii).decouplingcontrol_user.solvesymbolic = true;
									names(strcmp('solvesymbolic', names)) = [];
								end
								if strcmp(sub(2).subs, 'round_equations_to_digits')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.round_equations_to_digits;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.round_equations_to_digits = instance(ii).checkProperty('round_equations_to_digits', prop);
									instance(ii).decouplingcontrol_user.round_equations_to_digits = true;
									names(strcmp('round_equations_to_digits', names)) = [];
								end
								if strcmp(sub(2).subs, 'allowoutputdecoupling')
									if length(sub) > 2
										% further indexing into option
										tmp = instance(ii).decouplingcontrol_internal.allowoutputdecoupling;
										if ~iscell(tmp) && strcmpi(sub(3).type, '{}')
											% matlab R2015B crashes if cell index assignment is used for numerical values
											error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object. ');
										else
											prop = subsasgn(tmp, sub(3:end), varargin{ii});
										end
									else
										% setting option directly
										prop = varargin{ii};
									end
									instance(ii).decouplingcontrol_internal.allowoutputdecoupling = instance(ii).checkProperty('allowoutputdecoupling', prop);
									instance(ii).decouplingcontrol_user.allowoutputdecoupling = true;
									names(strcmp('allowoutputdecoupling', names)) = [];
								end
								if ~isempty(names)
									error('control:design:gamma:GammasynOptions:input', 'Option ''decouplingcontrol'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
								end
							else
								% indexing into this.decouplingcontrol with {} or ()
								try
									subsref(instance(ii).decouplingcontrol_internal, sub(2:end));
								catch e
									error(e.identifier, e.message);
								end
								error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''decouplingcontrol''.');
							end
						else
							% assigning to whole this.system structure (partially)
							if isempty(varargin{ii})
								instance(ii).decouplingcontrol_internal = struct(...
									'tf_structure',					proto.decouplingcontrol.tf_structure,...
									'decouplingstrategy',			proto.decouplingcontrol.decouplingstrategy,...
									'sortingstrategy_decoupling',	proto.decouplingcontrol.sortingstrategy_decoupling,...
									'weight_decoupling',			proto.decouplingcontrol.weight_decoupling,...
									'weight_prefilter',				proto.decouplingcontrol.weight_prefilter,...
									'tolerance_decoupling',			proto.decouplingcontrol.tolerance_decoupling,...
									'tolerance_prefilter',			proto.decouplingcontrol.tolerance_prefilter,...
									'solvesymbolic',				proto.decouplingcontrol.solvesymbolic,...
									'round_equations_to_digits',	proto.decouplingcontrol.round_equations_to_digits,...
									'allowoutputdecoupling',		proto.decouplingcontrol.allowoutputdecoupling...
								);
								instance(ii).decouplingcontrol_user = struct(...
									'tf_structure',					false,...
									'decouplingstrategy',			false,...
									'sortingstrategy_decoupling',	false,...
									'weight_decoupling',			false,...
									'weight_prefilter',				false,...
									'tolerance_decoupling',			false,...
									'tolerance_prefilter',			false,...
									'solvesymbolic',				false,...
									'round_equations_to_digits',	false,...
									'allowoutputdecoupling',		false...
								);
								return;
							end
							if ~isstruct(varargin{ii}) || ~isscalar(varargin{ii})
								error('control:design:gamma:GammasynOptions:input', 'Option ''decouplingcontrol'' must be a scalar structure.');
							end
							names = fieldnames(varargin{ii});
							if isfield(varargin{ii}, 'tf_structure')
								instance(ii).decouplingcontrol_internal.tf_structure = instance(ii).checkProperty('tf_structure', varargin{ii}.tf_structure);
								instance(ii).decouplingcontrol_user.tf_structure = true;
								names(strcmp('tf_structure', names)) = [];
							end
							if isfield(varargin{ii}, 'decouplingstrategy')
								instance(ii).sdecouplingcontrol_internal.decouplingstrategy = instance(ii).checkProperty('decouplingstrategy', varargin{ii}.decouplingstrategy);
								instance(ii).decouplingcontrol_user.decouplingstrategy = true;
								names(strcmp('decouplingstrategy', names)) = [];
							end
							if isfield(varargin{ii}, 'sortingstrategy_decoupling')
								instance(ii).sdecouplingcontrol_internal.sortingstrategy_decoupling = instance(ii).checkProperty('sortingstrategy_decoupling', varargin{ii}.sortingstrategy_decoupling);
								instance(ii).decouplingcontrol_user.sortingstrategy_decoupling = true;
								names(strcmp('sortingstrategy_decoupling', names)) = [];
							end
							if isfield(varargin{ii}, 'weight_decoupling')
								instance(ii).decouplingcontrol_internal.weight_decoupling = instance(ii).checkProperty('weight_decoupling', varargin{ii}.weight_decoupling);
								instance(ii).decouplingcontrol_user.weight_decoupling = true;
								names(strcmp('weight_decoupling', names)) = [];
							end
							if isfield(varargin{ii}, 'weight_prefilter')
								instance(ii).decouplingcontrol_internal.weight_prefilter = instance(ii).checkProperty('weight_prefilter', varargin{ii}.weight_prefilter);
								instance(ii).decouplingcontrol_user.weight_prefilter = true;
								names(strcmp('weight_prefilter', names)) = [];
							end
							if isfield(varargin{ii}, 'tolerance_decoupling')
								instance(ii).decouplingcontrol_internal.tolerance_decoupling = instance(ii).checkProperty('tolerance_decoupling', varargin{ii}.tolerance_decoupling);
								instance(ii).decouplingcontrol_user.tolerance_decoupling = true;
								names(strcmp('tolerance_decoupling', names)) = [];
							end
							if isfield(varargin{ii}, 'tolerance_prefilter')
								instance(ii).decouplingcontrol_internal.tolerance_prefilter = instance(ii).checkProperty('tolerance_prefilter', varargin{ii}.tolerance_prefilter);
								instance(ii).decouplingcontrol_user.tolerance_prefilter = true;
								names(strcmp('tolerance_prefilter', names)) = [];
							end
							if isfield(varargin{ii}, 'solvesymbolic')
								instance(ii).decouplingcontrol_internal.solvesymbolic = instance(ii).checkProperty('solvesymbolic', varargin{ii}.solvesymbolic);
								instance(ii).decouplingcontrol_user.solvesymbolic = true;
								names(strcmp('solvesymbolic', names)) = [];
							end
							if isfield(varargin{ii}, 'round_equations_to_digits')
								instance(ii).decouplingcontrol_internal.round_equations_to_digits = instance(ii).checkProperty('round_equations_to_digits', varargin{ii}.round_equations_to_digits);
								instance(ii).decouplingcontrol_user.round_equations_to_digits = true;
								names(strcmp('round_equations_to_digits', names)) = [];
							end
							if isfield(varargin{ii}, 'allowoutputdecoupling')
								instance(ii).decouplingcontrol_internal.allowoutputdecoupling = instance(ii).checkProperty('allowoutputdecoupling', varargin{ii}.allowoutputdecoupling);
								instance(ii).decouplingcontrol_user.allowoutputdecoupling = true;
								names(strcmp('allowoutputdecoupling', names)) = [];
							end
							if ~isempty(names)
								error('control:design:gamma:GammasynOptions:input', 'Option ''decouplingcontrol'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
							end
						end
					elseif strcmpi(sub(1).subs, 'objective')
						% 'objective' property requested
						objectivenames = {
							'preventNaN';
							'kreisselmeier';
							'lyapunov';
							'normgain'
						};
						objectivenames_all = {
							'rho';
							'max';
							'Q';
							'R';
							'R_shift';
							'K';
							'K_shift';
							'F';
							'F_shift'
						};
						if length(sub) > 1
							% deeper indexing requested
							if strcmp(sub(2).type, '()')
								% this.objective(...) indexing type
								if length(sub(2).subs) == 1
									% this.objective(:) and this.objective(1) are ok, others are converted to error by call to subsref
									if ischar(sub(2).subs{1}) && strcmpi(sub(2).subs{1}, ':')
										sub(2) = [];
									elseif isnumeric(sub(2).subs{1}) && isscalar(sub(2).subs{1}) && sub(2).subs{1} == 1
										sub(2) = [];
									else
										try
											subsref(instance(ii).objective_internal, sub(2:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''objective''.');
									end
								else
									% this.objective(:, :, ...) and this.objective(1, 1, ....) are ok, others are converted to error by call to subsref
									iscolon = cellfun(@(x) ischar(x) && strcmpi(x, ':'), sub(2).subs, 'UniformOutput', true);
									isone = cellfun(@(x) isscalar(x) && x == 1, sub(2).subs, 'UniformOutput', true);
									if all(iscolon(:))
										sub(2) = [];
									elseif all(isone(:))
										sub(2) = [];
									else
										try
											subsref(instance(ii).objective_internal, sub(2:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''objective''.');
									end
								end
							end
						end
						if length(sub) > 1
							% setting options under this.objective requested
							if strcmp(sub(2).type, '.') && any(strcmpi(sub(2).subs, objectivenames))
								% setting this.objective.option
								if length(sub) > 2
									% deeper indexing requested
									if strcmp(sub(3).type, '()')
										% this.objective.option(...) indexing type
										if length(sub(3).subs) == 1
											% this.objective.option(:) and this.objective.option(1) are ok, others are converted to error by call to subsref
											if ischar(sub(3).subs{1}) && strcmpi(sub(3).subs{1}, ':')
												sub(3) = [];
											elseif isnumeric(sub(3).subs{1}) && isscalar(sub(3).subs{1}) && sub(3).subs{1} == 1
												sub(3) = [];
											else
												try
													subsref(instance(ii).objective_internal.(sub(2).subs), sub(3:end));
												catch e
													error(e.identifier, e.message);
												end
												error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''objective.%s''.', sub(2).subs);
											end
										else
											% this.objective.option(:, :, ...) and this.objective.option(1, 1, ....) are ok, others are converted to error by call to subsref
											iscolon = cellfun(@(x) ischar(x) && strcmpi(x, ':'), sub(3).subs, 'UniformOutput', true);
											isone = cellfun(@(x) isscalar(x) && x == 1, sub(3).subs, 'UniformOutput', true);
											if all(iscolon(:))
												sub(3) = [];
											elseif all(isone(:))
												sub(3) = [];
											else
												try
													subsref(instance(ii).objective_internal.(sub(2).subs), sub(3:end));
												catch e
													error(e.identifier, e.message);
												end
												error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''objective.%s''.', sub(2).subs);
											end
										end
									end
								end
								if length(sub) > 2
									if strcmp(sub(3).type, '.') && any(strcmpi(sub(3).subs, objectivenames_all))
										names = {sub(3).subs};
										if strcmp(sub(2).subs, 'kreisselmeier')
											if strcmp(sub(3).subs, 'rho')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.kreisselmeier.rho;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.kreisselmeier.rho = instance(ii).checkProperty('rho', prop);
												instance(ii).objective_user.kreisselmeier.rho = true;
												names(strcmp('rho', names)) = [];
											end
											if strcmp(sub(3).subs, 'max')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.kreisselmeier.max;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.kreisselmeier.max = instance(ii).checkProperty('max', prop);
												instance(ii).objective_user.kreisselmeier.max = true;
												names(strcmp('max', names)) = [];
											end
										end
										if strcmp(sub(2).subs, 'lyapunov')
											if strcmp(sub(3).subs, 'Q')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.lyapunov.Q;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.lyapunov.Q = instance(ii).checkProperty('Q', prop);
												instance(ii).objective_user.lyapunov.Q = true;
												names(strcmp('Q', names)) = [];
											end
										end
										if strcmp(sub(2).subs, 'normgain')
											if strcmp(sub(3).subs, 'R')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.normgain.R;
													prop = subsasgn(tmp, sub(4:end), varargin{ii});
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.normgain.R = instance(ii).checkProperty('R', prop);
												instance(ii).objective_user.normgain.R = true;
												names(strcmp('R', names)) = [];
											end
											if strcmp(sub(3).subs, 'R_shift')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.normgain.R_shift;
													prop = subsasgn(tmp, sub(4:end), varargin{ii});
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.normgain.R_shift = instance(ii).checkProperty('R_shift', prop);
												instance(ii).objective_user.normgain.R_shift = true;
												names(strcmp('R_shift', names)) = [];
											end
											if strcmp(sub(3).subs, 'K')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.normgain.K;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.normgain.K = instance(ii).checkProperty('K', prop);
												instance(ii).objective_user.normgain.K = true;
												names(strcmp('K', names)) = [];
											end
											if strcmp(sub(3).subs, 'K_shift')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.normgain.K_shift;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.normgain.K_shift = instance(ii).checkProperty('K_shift', prop);
												instance(ii).objective_user.normgain.K_shift = true;
												names(strcmp('K_shift', names)) = [];
											end
											if strcmp(sub(3).subs, 'F')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.normgain.F;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.normgain.F = instance(ii).checkProperty('F', prop);
												instance(ii).objective_user.normgain.F = true;
												names(strcmp('F', names)) = [];
											end
											if strcmp(sub(3).subs, 'F_shift')
												if length(sub) > 3
													% further indexing into option
													tmp = instance(ii).objective_internal.normgain.F_shift;
													if ~iscell(tmp) && strcmpi(sub(4).type, '{}')
														% matlab R2015B crashes if cell index assignment is used for numerical values
														error('control:design:gamma:GammasynOptions:input', 'Cell contents assignment to a non-cell array object.');
													else
														prop = subsasgn(tmp, sub(4:end), varargin{ii});
													end
												else
													% setting option directly
													prop = varargin{ii};
												end
												instance(ii).objective_internal.normgain.F_shift = instance(ii).checkProperty('F_shift', prop);
												instance(ii).objective_user.normgain.F_shift = true;
												names(strcmp('F_shift', names)) = [];
											end
										end
										if ~isempty(names)
											error('control:design:gamma:GammasynOptions:input', 'Option ''system'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
										end
									else
										% indexing into this.system with {} or ()
										try
											subsref(instance(ii).objective_internal.(sub(2).subs), sub(3:end));
										catch e
											error(e.identifier, e.message);
										end
										error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''objective''.');
									end
								else
									names = {sub(2).subs};
									if strcmp(sub(2).subs, 'preventNaN')
										if isempty(varargin{ii})
											instance(ii).objective_internal.preventNaN = proto.objective.preventNaN;
											instance(ii).objective_user.preventNaN = false;
											return;
										end
										instance(ii).objective_internal.preventNaN = instance(ii).checkProperty('preventNaN', varargin{ii});
										instance(ii).objective_user.preventNaN = true;
										names(strcmp('preventNaN', names)) = [];
									end
									if strcmp(sub(2).subs, 'kreisselmeier')
										if isempty(varargin{ii})
											instance(ii).objective_internal.kreisselmeier = struct(...
												'rho',	proto.objective.kreisselmeier.rho,...
												'max',	proto.objective.kreisselmeier.max...
											);
											instance(ii).objective_user.kreisselmeier = struct(...
												'rho',	false,...
												'max',	false...
											);
											return;
										end
										if ~isstruct(varargin{ii}) || ~isscalar(varargin{ii})
											error('control:design:gamma:GammasynOptions:input', 'Option ''objective.%s'' must be a scalar structure.', sub(2).subs);
										end
										if isstruct(varargin{ii}) && isscalar(varargin{ii})
											names_2 = fieldnames(varargin{ii});
											if isfield(varargin{ii}, 'rho')
												instance(ii).objective_internal.kreisselmeier.rho = instance(ii).checkProperty('rho', varargin{ii}.rho);
												instance(ii).objective_user.kreisselmeier.rho = true;
												names_2(strcmp('rho', names_2)) = [];
											end
											if isfield(varargin{ii}, 'max')
												instance(ii).objective_internal.kreisselmeier.max = instance(ii).checkProperty('max', varargin{ii}.max);
												instance(ii).objective_user.kreisselmeier.max = true;
												names_2(strcmp('max', names_2)) = [];
											end
											if ~isempty(names_2)
												error('control:design:gamma:GammasynOptions:input', 'Option ''objective.kreisselmeier'' does not have field%s ''%s''.', iftern(length(names_2) > 1, 's', ''), strjoin(names_2, ''', '''));
											end
										else
											error('control:design:gamma:GammasynOptions:input', 'Option ''objective.kreisselmeier'' must be a scalar structure.');
										end
										names(strcmp('kreisselmeier', names)) = [];
									end
									if strcmp(sub(2).subs, 'lyapunov')
										if isempty(varargin{ii})
											instance(ii).objective_internal.lyapunov = struct(...
												'Q',	proto.objective.lyapunov.Q...
											);
											instance(ii).objective_user.lyapunov = struct(...
												'Q',	false...
											);
											return;
										end
										if ~isstruct(varargin{ii}) || ~isscalar(varargin{ii})
											error('control:design:gamma:GammasynOptions:input', 'Option ''objective.%s'' must be a scalar structure.', sub(2).subs);
										end
										if isstruct(varargin{ii}) && isscalar(varargin{ii})
											names_2 = fieldnames(varargin{ii});
											if isfield(varargin{ii}, 'Q')
												instance(ii).objective_internal.lyapunov.Q = instance(ii).checkProperty('Q', varargin{ii}.Q);
												instance(ii).objective_user.lyapunov.Q = true;
												names_2(strcmp('Q', names_2)) = [];
											end
											if ~isempty(names_2)
												error('control:design:gamma:GammasynOptions:input', 'Option ''objective.lyapunov'' does not have field%s ''%s''.', iftern(length(names_2) > 1, 's', ''), strjoin(names_2, ''', '''));
											end
										else
											error('control:design:gamma:GammasynOptions:input', 'Option ''objective.lyapunov'' must be a scalar structure.');
										end
										names(strcmp('lyapunov', names)) = [];
									end
									if strcmp(sub(2).subs, 'normgain')
										if isempty(varargin{ii})
											instance(ii).objective_internal.normgain = struct(...
												'R',		proto.objective.normgain.R,...
												'R_shift',	proto.objective.normgain.R_shift,...
												'K',		proto.objective.normgain.K,...
												'K_shift',	proto.objective.normgain.K_shift,...
												'F',		proto.objective.normgain.F,...
												'F_shift',	proto.objective.normgain.F_shift...
											);
											instance(ii).objective_user.normgain = struct(...
												'R',		false,...
												'R_shift',	false,...
												'K',		false,...
												'K_shift',	false,...
												'F',		false,...
												'F_shift',	false...
											);
											return;
										end
										if ~isstruct(varargin{ii}) || ~isscalar(varargin{ii})
											error('control:design:gamma:GammasynOptions:input', 'Option ''objective.%s'' must be a scalar structure.', sub(2).subs);
										end
										if isstruct(varargin{ii}) && isscalar(varargin{ii})
											names_2 = fieldnames(varargin{ii});
											if isfield(varargin{ii}, 'R')
												instance(ii).objective_internal.normgain.R = instance(ii).checkProperty('R', varargin{ii}.R);
												instance(ii).objective_user.normgain.R = true;
												names_2(strcmp('R', names_2)) = [];
											end
											if isfield(varargin{ii}, 'R_shift')
												instance(ii).objective_internal.normgain.R_shift = instance(ii).checkProperty('R_shift', varargin{ii}.R_shift);
												instance(ii).objective_user.normgain.R_shift = true;
												names_2(strcmp('R_shift', names_2)) = [];
											end
											if isfield(varargin{ii}, 'K')
												instance(ii).objective_internal.normgain.K = instance(ii).checkProperty('K', varargin{ii}.K);
												instance(ii).objective_user.normgain.K = true;
												names_2(strcmp('K', names_2)) = [];
											end
											if isfield(varargin{ii}, 'K_shift')
												instance(ii).objective_internal.normgain.K_shift = instance(ii).checkProperty('K_shift', varargin{ii}.K_shift);
												instance(ii).objective_user.normgain.K_shift = true;
												names_2(strcmp('K_shift', names_2)) = [];
											end
											if isfield(varargin{ii}, 'F')
												instance(ii).objective_internal.normgain.F = instance(ii).checkProperty('F', varargin{ii}.F);
												instance(ii).objective_user.normgain.F = true;
												names_2(strcmp('F', names_2)) = [];
											end
											if isfield(varargin{ii}, 'F_shift')
												instance(ii).objective_internal.normgain.F_shift = instance(ii).checkProperty('F_shift', varargin{ii}.F_shift);
												instance(ii).objective_user.normgain.F_shift = true;
												names_2(strcmp('F_shift', names_2)) = [];
											end
											if ~isempty(names_2)
												error('control:design:gamma:GammasynOptions:input', 'Option ''objective.normgain'' does not have field%s ''%s''.', iftern(length(names_2) > 1, 's', ''), strjoin(names_2, ''', '''));
											end
										else
											error('control:design:gamma:GammasynOptions:input', 'Option ''objective.normgain'' must be a scalar structure.');
										end
										names(strcmp('normgain', names)) = [];
									end
									if ~isempty(names)
										error('control:design:gamma:GammasynOptions:input', 'Option ''objective'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
									end
								end
							else
								% indexing into this.objective with {} or ()
								try
									subsref(instance(ii).objective_internal, sub(2:end));
								catch e
									error(e.identifier, e.message);
								end
								error('control:design:gamma:GammasynOptions:input', 'Undefined indexing type for Option ''objective''.');
							end
						else
							% assigning to whole this.objective structure (partially)
							if isempty(varargin{ii})
								instance(ii).objective_internal = struct(...
									'preventNaN',		proto.objective.preventNaN,...
									'kreisselmeier',	struct(...
										'rho',	proto.objective.kreisselmeier.rho,...
										'max',	proto.objective.kreisselmeier.max...
									),...
									'lyapunov', struct(...
										'Q',	proto.objective.lyapunov.Q...
									),...
									'normgain', struct(...
										'R',		proto.objective.normgain.R,...
										'R_shift',	proto.objective.normgain.R_shift,...
										'K',		proto.objective.normgain.K,...
										'K_shift',	proto.objective.normgain.K_shift,...
										'F',		proto.objective.normgain.F,...
										'F_shift',	proto.objective.normgain.F_shift...
									)...
								);
								instance(ii).objective_user = struct(...
									'preventNaN',		false,...
									'kreisselmeier',	struct(...
										'rho',	false,...
										'max',	false...
									),...
									'lyapunov', struct(...
										'Q',	false...
									),...
									'normgain', struct(...
										'R',		false,...
										'R_shift',	false,...
										'K',		false,...
										'K_shift',	false,...
										'F',		false,...
										'F_shift',	false...
									)...
								);
								return;
							end
							if ~isstruct(varargin{ii}) || ~isscalar(varargin{ii})
								error('control:design:gamma:GammasynOptions:input', 'Option ''objective'' must be a scalar structure.');
							end
							names = fieldnames(varargin{ii});
							if isfield(varargin{ii}, 'preventNaN')
								instance(ii).objective_internal.preventNaN = instance(ii).checkProperty('preventNaN', varargin{ii}.preventNaN);
								instance(ii).objective_user.preventNaN = true;
								names(strcmp('preventNaN', names)) = [];
							end
							if isfield(varargin{ii}, 'kreisselmeier')
								if isstruct(varargin{ii}.kreisselmeier) && isscalar(varargin{ii}.kreisselmeier)
									names_2 = fieldnames(varargin{ii}.kreisselmeier);
									if isfield(varargin{ii}.kreisselmeier, 'rho')
										instance(ii).objective_internal.kreisselmeier.rho = instance(ii).checkProperty('rho', varargin{ii}.kreisselmeier.rho);
										instance(ii).objective_user.kreisselmeier.rho = true;
										names_2(strcmp('rho', names_2)) = [];
									end
									if isfield(varargin{ii}.kreisselmeier, 'max')
										instance(ii).objective_internal.kreisselmeier.max = instance(ii).checkProperty('max', varargin{ii}.kreisselmeier.max);
										instance(ii).objective_user.kreisselmeier.max = true;
										names_2(strcmp('max', names_2)) = [];
									end
									if ~isempty(names_2)
										error('control:design:gamma:GammasynOptions:input', 'Option ''objective.kreisselmeier'' does not have field%s ''%s''.', iftern(length(names_2) > 1, 's', ''), strjoin(names_2, ''', '''));
									end
								else
									error('control:design:gamma:GammasynOptions:input', 'Option ''objective.kreisselmeier'' must be a scalar structure.');
								end
								names(strcmp('kreisselmeier', names)) = [];
							end
							if isfield(varargin{ii}, 'lyapunov')
								if isstruct(varargin{ii}.lyapunov) && isscalar(varargin{ii}.lyapunov)
									names_2 = fieldnames(varargin{ii}.lyapunov);
									if isfield(varargin{ii}.lyapunov, 'Q')
										instance(ii).objective_internal.lyapunov.Q = instance(ii).checkProperty('Q', varargin{ii}.lyapunov.Q);
										instance(ii).objective_user.lyapunov.Q = true;
										names_2(strcmp('Q', names_2)) = [];
									end
									if ~isempty(names_2)
										error('control:design:gamma:GammasynOptions:input', 'Option ''objective.lyapunov'' does not have field%s ''%s''.', iftern(length(names_2) > 1, 's', ''), strjoin(names_2, ''', '''));
									end
								else
									error('control:design:gamma:GammasynOptions:input', 'Option ''objective.lyapunov'' must be a scalar structure.');
								end
								names(strcmp('lyapunov', names)) = [];
							end
							if isfield(varargin{ii}, 'normgain')
								if isstruct(varargin{ii}.normgain) && isscalar(varargin{ii}.normgain)
									names_2 = fieldnames(varargin{ii}.normgain);
									if isfield(varargin{ii}.normgain, 'R')
										instance(ii).objective_internal.normgain.R = instance(ii).checkProperty('R', varargin{ii}.normgain.R);
										instance(ii).objective_user.normgain.R = true;
										names_2(strcmp('R', names_2)) = [];
									end
									if isfield(varargin{ii}.normgain, 'R_shift')
										instance(ii).objective_internal.normgain.R_shift = instance(ii).checkProperty('R', varargin{ii}.normgain.R_shift);
										instance(ii).objective_user.normgain.R_shift = true;
										names_2(strcmp('R_shift', names_2)) = [];
									end
									if isfield(varargin{ii}.normgain, 'K')
										instance(ii).objective_internal.normgain.K = instance(ii).checkProperty('K', varargin{ii}.normgain.K);
										instance(ii).objective_user.normgain.K = true;
										names_2(strcmp('K', names_2)) = [];
									end
									if isfield(varargin{ii}.normgain, 'K_shift')
										instance(ii).objective_internal.normgain.K_shift = instance(ii).checkProperty('K_shift', varargin{ii}.normgain.K_shift);
										instance(ii).objective_user.normgain.K_shift = true;
										names_2(strcmp('K_shift', names_2)) = [];
									end
									if isfield(varargin{ii}.normgain, 'F')
										instance(ii).objective_internal.normgain.F = instance(ii).checkProperty('F', varargin{ii}.normgain.F);
										instance(ii).objective_user.normgain.F = true;
										names_2(strcmp('F', names_2)) = [];
									end
									if isfield(varargin{ii}.normgain, 'F_shift')
										instance(ii).objective_internal.normgain.F_shift = instance(ii).checkProperty('F_shift', varargin{ii}.normgain.F_shift);
										instance(ii).objective_user.normgain.F_shift = true;
										names_2(strcmp('F_shift', names_2)) = [];
									end
									if ~isempty(names_2)
										error('control:design:gamma:GammasynOptions:input', 'Option ''objective.normgain'' does not have field%s ''%s''.', iftern(length(names_2) > 1, 's', ''), strjoin(names_2, ''', '''));
									end
								else
									error('control:design:gamma:GammasynOptions:input', 'Option ''objective.normgain'' must be a scalar structure.');
								end
								names(strcmp('normgain', names)) = [];
							end
							if ~isempty(names)
								error('control:design:gamma:GammasynOptions:input', 'Option ''objective'' does not have field%s ''%s''.', iftern(length(names) > 1, 's', ''), strjoin(names, ''', '''));
							end
						end
					else
						available = instance(ii).available_options();
						match = strcmp(sub(1).subs, available(:, 1));
						if any(match(:))
							if length(sub) == 1
								instance(ii).(sub(1).subs) = varargin{ii};
							else
								temp = builtin('subsasgn', instance(ii).(sub(1).subs), sub(2:end), varargin{ii});
								instance(ii).(sub(1).subs) = temp;
							end
						else
							instance(ii) = builtin('subsasgn', instance(ii), sub, varargin{ii});
						end
					end
				end
				if isempty(oldsub)
					this = instance;
				else
					if ~assignall
						this = subsasgn(this, oldsub, instance);
					else
						this = instance;
					end
				end
			else
				if isempty(oldsub)
					this = builtin('subsasgn', instance, sub, varargin{:});
				else
					instance = builtin('subsasgn', instance, sub, varargin{:});
					this = subsasgn(this, oldsub, instance);
				end
			end
		end

		function [] = useoptions(this, options, varargin)
			%USEOPTIONS set supplied options to current option set
			%	Input:
			%		this:		instance
			%		options:	structure or GammasynOptions to set
			%		varargin:	name value pairs (options is the first name) to set
			if isempty(this)
				return;
			end
			opt = this(1).options;
			args = varargin;
			if isstruct(options)
				setoptions = options;
				sametype = true;
			elseif isa(options, 'control.design.gamma.GammasynOptions')
				setoptions = options.get_user();
				sametype = true;
			else
				setoptions = struct();
				args = [{options}, args];
				sametype = false;
			end
			if nargin >= 3 || ~sametype
				% extract cellpath arguments
				if mod(numel(args), 2) == 0
					% errors anyway if not given in pairs
					hasdot = false(size(args, 2), 1);
					isnormalarg = false(size(args, 2), 1);
					for ii = 1:size(args, 2)
						isnormalarg(ii, 1) = ischar(args{ii});
						hasdot(ii, 1) = isnormalarg(ii, 1) && any(args{ii}(:) == '.');
					end
					isspecial = ~isnormalarg;
					isarg = isspecial(1:2:end) | hasdot(1:2:end) | isnormalarg(1:2:end);
					if any(hasdot(:))
						for ii = 1:2:length(hasdot)
							if hasdot(ii)
								args{ii} = strsplit(args{ii}, '.');
							end
						end
					end
					specialargs = args(reshape([
						isarg;
						isarg
					], 1, []));
					args(reshape([
						isarg;
						isarg
					], 1, [])) = [];
				else
					error('control:design:gamma:GammasynOptions:input', 'Options must be specified in pairs.');
				end
				% parse unmatched elements with nested paths that are specified as end of the path (is unique among the parameter names)
				nested = opt(~[opt{:, 2}], :);
				nested_not = opt([opt{:, 2}], :);
				% parse nested paths specified as cell paths
				% and parse normal names specified as characters
				if ~isempty(specialargs)
					if mod(numel(specialargs), 2) == 0
						specialargs = reshape(specialargs, 2, [])';
						optionordinal = 1:(numel(args) + numel(specialargs))/2;
						optionordinal = optionordinal(isarg);
						for ii = 1:size(specialargs, 1)
							if iscell(specialargs{ii, 1}) && ndims(specialargs{ii, 1}) <= 2 && ~isempty(specialargs{ii, 1})
								if isrow(specialargs{ii, 1}) || iscolumn(specialargs{ii, 1})
									if iscellstr(specialargs{ii, 1})
										if numel(specialargs{ii, 1}) == 1
											if ~isempty(specialargs{ii, 1}{1})
												match = strcmp(specialargs{ii, 1}{1}, nested_not(:, 1));
												if any(match(:))
													if sum(match) == 1
														setoptions.(nested_not{match, 1}) = specialargs{ii, 2};
													else
														error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' matches for multiple options.', strjoin(specialargs{ii, 1}, '.'));
													end
												else
													error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' does not match any option.', strjoin(specialargs{ii, 1}, '.'));
												end
											else
												error('control:design:gamma:GammasynOptions:input', 'Option path for option %d must not be empty.', optionordinal(1, ii));
											end
										else
											match = strcmp(specialargs{ii, 1}{1}, nested(:, 6));
											if any(match(:))
												possiblematches = nested(match, :);
												pathlen = cellfun(@length, possiblematches(:, 4), 'UniformOutput', true);
												normalizedpath = cell(size(possiblematches, 1), max(pathlen)); %#ok<CPROPLC> call to max function
												for jj = 1:size(normalizedpath, 1)
													normalizedpath(jj, 1:length(nested{jj, 4})) = [nested{jj, 4}(2:end), nested{jj, 1}];
												end
												for jj = 2:numel(specialargs{ii, 1})
													hasdepth = ~all(jj - 1 > pathlen);
													if hasdepth
														match = strcmp(specialargs{ii, 1}{jj}, normalizedpath(:, jj - 1));
														if any(match(:))
															possiblematches = possiblematches(match, :);
															normalizedpath = normalizedpath(match, :);
														else
															error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' does not match any option.', strjoin(specialargs{ii, 1}, '.'));
														end
													else
														error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' does not match any option.', strjoin(specialargs{ii, 1}, '.'));
													end
												end
												if ~isempty(possiblematches)
													if size(possiblematches, 1) == 1
														setoptions = setstruct(setoptions, [possiblematches{1, 4}, possiblematches{1, 1}], specialargs{ii, 2});
													else
														error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' matches for multiple options.', strjoin(specialargs{ii, 1}, '.'));
													end
												else
													error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' does not match any option.', strjoin(specialargs{ii, 1}, '.'));
												end
											else
												error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' does not match any option.', strjoin(specialargs{ii, 1}, '.'));
											end
										end
									else
										error('control:design:gamma:GammasynOptions:input', 'Option path for option %d must be a cellstring.', optionordinal(1, ii));
									end
								else
									error('control:design:gamma:GammasynOptions:input', 'Option path for option %d must be a row or column path.', optionordinal(1, ii));
								end
							elseif ischar(specialargs{ii, 1})
								match = strcmp(specialargs{ii, 1}, nested_not(:, 1));
								if any(match(:))
									if sum(match) == 1
										setoptions.(nested_not{match, 1}) = specialargs{ii, 2};
									else
										error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' matches for multiple options.', specialargs{ii, 1});
									end
								else
									error('control:design:gamma:GammasynOptions:input', 'Option path ''%s'' does not match any option.', specialargs{ii, 1});
								end
							else
								error('control:design:gamma:GammasynOptions:input', 'Option path for option %d must be scalar cell.', optionordinal(1, ii));
							end
						end
					else
						error('control:design:gamma:GammasynOptions:input', 'Options must be specified in pairs.');
					end
				end
			end
			for tt = 1:numel(this) %#ok<FORPF> no parfor for object assignments
				this(tt).use_options(setoptions);
			end
		end
	end

end

function [y] = getfirst(x)
	if isempty(x)
		y = '';
	else
		y = x{1};
	end
end