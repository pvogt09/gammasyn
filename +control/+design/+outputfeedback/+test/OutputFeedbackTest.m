function [pass] = OutputFeedbackTest(~)
	%OUTPUTFEEDBACKTEST test cases for checking dimensions and return values of output feedback classes
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	package = meta.package.fromName(path_to_package([realpath(fullfile(mfilename('fullpath'), '..', '..')), filesep]));
	classes = package.ClassList;
	classes = classes(~[package.ClassList.Abstract]);
	feedbackclasses = cell(size(classes, 1), 1);
	for ii = 1:size(classes, 1)
		feedbackclasses{ii} = str2func(classes(ii).Name);
	end
% 	feedbackclasses = {
% 		@control.design.outputfeedback.ConstantOutputFeedback;
% 		@control.design.outputfeedback.DynamicOutputFeedback;
% 		@control.design.outputfeedback.ObserverDirectOutputFeedback;
% 		@control.design.outputfeedback.ObserverOutputFeedback;
% 		@control.design.outputfeedback.ObserverStateFeedback;
% 		@control.design.outputfeedback.PDDirectOutputFeedback;
% 		@control.design.outputfeedback.PDOutputFeedback;
% 		@control.design.outputfeedback.PDRealDirectOutputFeedback;
% 		@control.design.outputfeedback.PDRealOutputFeedback;
%		@control.design.outputfeedback.PDRealStateFeedback;
%		@control.design.outputfeedback.PDStateFeedback;
%  		@control.design.outputfeedback.PDDirect1DFOutputFeedback;
% 		@control.design.outputfeedback.PIDDirectOutputFeedback;
% 		@control.design.outputfeedback.PIDDirectOutputFeedback;
% 		@control.design.outputfeedback.PIDirectOutputFeedback;
% 		@control.design.outputfeedback.PIDOutputFeedback;
% 		@control.design.outputfeedback.PIDRealOutputFeedback;
% 		@control.design.outputfeedback.PIObserverDirectOutputFeedback;
% 		@control.design.outputfeedback.PIObserverOutputFeedback;
% 		@control.design.outputfeedback.PIOutputFeedback;
% 		@control.design.outputfeedback.PIStateFeedback;
% 		@control.design.outputfeedback.StateFeedback;
% 	};
	systems = {
		% continuous
		struct(...
			'E',		[],...
			'A',		-eye(3),...
			'B',		(1:3 == 3)',...
			'C',		1:3 == 1,...
			'D',		(0),...
			'C_ref',	1:3 == 1,...
			'D_ref',	(0)...
		);
		struct(...
			'E',		eye(3),...
			'A',		-eye(3),...
			'B',		(1:3 == 3)',...
			'C',		1:3 == 1,...
			'C_dot',	1:3 == 1,...
			'D',		(0),...
			'C_ref',	1:3 == 1,...
			'D_ref',	(0)...
		);
		ss(tf([1, 1], [1, 4, 6]));
		struct(...
			'E',		eye(3),...
			'A',		-eye(3),...
			'B',		[
				(1:3 == 3)', (1:3 == 2)'
			],...
			'C',		1:3 == 1,...
			'C_dot',	1:3 == 1,...
			'D',		[0,	0],...
			'C_ref',	1:3 == 1,...
			'D_ref',	[0,	0]...
		);
		struct(...
			'E',		eye(3),...
			'A',		-eye(3),...
			'B',		[
				(1:3 == 3)', (1:3 == 2)'
			],...
			'C',		[
				1:3 == 1;
				1:3 == 2
			],...
			'C_dot',	1:3 == 1,...
			'D',		[
				0,	0;
				0,	0
			],...
			'C_ref',		[
				1:3 == 1;
				1:3 == 2
			],...
			'D_ref',		[
				0,	0;
				0,	0
			]...
		);
		ss(tf(1, [1, 1]));
		ss(tf(1, poly([-1, -2])));
		ss(tf(1, poly([-1, -2, -3])));
		ss(tf(1, poly([-1, -2, -3, -4])));
		ss(tf(1, poly([-1, -2, -3, -4, -5])));
		ss(tf(1, poly([-1, -2, -3, -4, -5, -6])));
		ss(tf(1, poly([-1, -2, -3, -4, -5, -6, -7])));
		ss(tf(1, poly([-1, -2, -3, -4, -5, -6, -7, -8])));
		ss(-eye(5) + diag(ones(4, 1), 1), double(1:5 == 5)', double(1:5 == 1), []);
		% discrete
		struct(...
			'E',		[],...
			'A',		0.9*eye(3),...
			'B',		(1:3 == 3)',...
			'C',		1:3 == 1,...
			'D',		0,...
			'C_ref',	1:3 == 1,...
			'D_ref',	0,...
			'T',		0.01...
		);
		struct(...
			'E',		eye(3),...
			'A',		0.9*eye(3),...
			'B',		(1:3 == 3)',...
			'C',		1:3 == 1,...
			'C_dot',	1:3 == 1,...
			'D',		0,...
			'C_ref',	1:3 == 1,...
			'D_ref',	0,...
			'T',		0.01...
		);
		ss(c2d(tf([1, 1], [1, 4, 6]), 0.01));
		struct(...
			'E',		eye(3),...
			'A',		0.9*eye(3),...
			'B',		[
				(1:3 == 3)', (1:3 == 2)'
			],...
			'C',		1:3 == 1,...
			'C_dot',	1:3 == 1,...
			'D',		[0,	0],...
			'C_ref',	1:3 == 1,...
			'D_ref',	[0,	0],...
			'T',		0.01...
		);
		struct(...
			'E',		eye(3),...
			'A',		0.9*eye(3),...
			'B',		[
				(1:3 == 3)', (1:3 == 2)'
			],...
			'C',		[
				1:3 == 1;
				1:3 == 2
			],...
			'C_dot',	1:3 == 1,...
			'D',		[
				0,	0;
				0,	0
			],...
			'C_ref',	[
				1:3 == 1;
				1:3 == 2
			],...
			'D_ref',	[
				0,	0;
				0,	0
			],...
			'T',		0.01...
		);
		ss(tf(1, [1, -0.9], 0.01));
		ss(tf(1, poly([0.9, 0.8]), 0.01));
		ss(tf(1, poly([0.9, 0.8, 0.7]), 0.01));
		ss(tf(1, poly([0.9, 0.8, 0.7, 0.6]), 0.01));
		ss(tf(1, poly([0.9, 0.8, 0.7, 0.6, 0.5]), 0.01));
		ss(tf(1, poly([0.9, 0.8, 0.7, 0.6, 0.5, 0.4]), 0.01));
		ss(tf(1, poly([0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3]), 0.01));
		ss(tf(1, poly([0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2]), 0.01));
		ss(0.9*eye(5) + diag(ones(4, 1), 1), double(1:5 == 5)', double(1:5 == 1), [], 0.01);
	};
	T = 0.01;
	for ii = 1:size(feedbackclasses, 1)
		cls = feedbackclasses{ii};
		for jj = 1:size(systems, 1)
			if isstruct(systems{jj})
				T_x = 10*eye(size(systems{jj}.A, 1));
				T_y = 20*eye(size(systems{jj}.C, 1));
				if isfield(systems{jj}, 'C_dot')
					T_y_dot = 0.1*eye(size(systems{jj}.C_dot, 1));
				else
					T_y_dot = [];
				end
				T_u = 100*eye(size(systems{jj}.B, 2));
				T_w = T_y;
				basesystem = systems{jj};
				if ~isfield(basesystem, 'C_dot')
					basesystem.C_dot = zeros(0, size(basesystem.A, 1));
				end
				if isempty(basesystem.E)
					basesystem.E = eye(size(basesystem.A, 1));
				end
			else
				[Atemp, Btemp, Ctemp, Dtemp] = ssdata(systems{jj});
				T_x = 10*eye(size(Atemp, 1));
				T_y = 20*eye(size(Ctemp, 1));
				T_y_dot = 0.1*eye(size(Ctemp, 1));
				T_u = 100*eye(size(Btemp, 2));
				T_w = T_y;
				basesystem = struct(...
					'E',		E,...
					'A',		Atemp,...
					'B',		Btemp,...
					'C',		Ctemp,...
					'C_dot',	zeros(size(Ctemp, 1), size(Atemp, 1)),...
					'D',		Dtemp...
				);
			end
			test.TestSuite.assertNoException('cls(systems{jj});', 'control:outputfeedback:test', 'Construction of an outputfeedback class must not throw an exception.');
			controller = cls(systems{jj});
			test.TestSuite.assertEqual(controller.SimulinkVariant(), baseclassname(controller), 'control:outputfeedback:test', 'Variant name must be equal to class name.');
			test.TestSuite.assertNoException('controller.amend(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('controller.E(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning E must not throw an exception.');
			test.TestSuite.assertNoException('controller.A(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning A must not throw an exception.');
			test.TestSuite.assertNoException('controller.B(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning B must not throw an exception.');
			test.TestSuite.assertNoException('controller.C(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning C must not throw an exception.');
			test.TestSuite.assertNoException('controller.C_dot(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning C_dot must not throw an exception.');
			test.TestSuite.assertNoException('controller.D(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning D must not throw an exception.');
			test.TestSuite.assertNoException('controller.C_ref(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning C_ref must not throw an exception.');
			test.TestSuite.assertNoException('controller.D_ref(systems{jj});', 'control:outputfeedback:test', 'Amending system with outputfeedback class and returning D_ref must not throw an exception.');
			test.TestSuite.assertNoException('[r] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound, f_bound] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound, f_bound, rkf_bound] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound, f_bound, rkf_bound, r_nonlin] = controller.gainpattern(systems{jj});', 'control:outputfeedback:test', 'Gainpattern of outputfeedback class must not throw an exception.');
			system = controller.amend(systems{jj});
			if isa(system, 'ss')
				[A, B, C, D] = ssdata(system);
				if isempty(system.e)
					E = eye(size(A, 1));
				else
					E = system.e;
				end
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	controller.C_dot(systems{jj}),...
					'D',		D,...
					'C_ref',	controller.C_ref(systems{jj}),...
					'D_ref',	controller.D_ref(systems{jj})...
				);
			elseif isa(system, 'tf')
				[A, B, C, D] = ssdata(system);
				E = eye(size(A, 1));
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	controller.C_dot(systems{jj}),...
					'D',		D,...
					'C_ref',	controller.C_ref(systems{jj}),...
					'D_ref',	controller.D_ref(systems{jj})...
				);
			end
			E = controller.E(systems{jj});
			A = controller.A(systems{jj});
			B = controller.B(systems{jj});
			C = controller.C(systems{jj});
			C_dot = controller.C_dot(systems{jj});
			D = controller.D(systems{jj});
			C_ref = controller.C_ref(systems{jj});
			D_ref = controller.D_ref(systems{jj});
			[r, k, f, rkf, r_bound, k_bound, f_bound, rkf_bound, r_nonlin] = controller.gainpattern(systems{jj});
			R_test = double(r{1}(:, :, 1));
			K_test = double(k{1}(:, :, 1));
			F_test = double(f{1}(:, :, 1));
			test.TestSuite.assertEqual(size(system.A, 1), size(system.A, 2), 'control:outputfeedback:test', 'System matrix must be square.');
			test.TestSuite.assertEqual(size(system.E, 1), size(system.E, 2), 'control:outputfeedback:test', 'Descriptor matrix must be square.');
			test.TestSuite.assertEqual(size(system.A, 1), size(system.E, 1), 'control:outputfeedback:test', 'System matrix must have the same size as descriptor matrix.');
			test.TestSuite.assertEqual(size(system.C, 2), size(system.A, 1), 'control:outputfeedback:test', 'Output matrix must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_dot, 2), size(system.A, 1), 'control:outputfeedback:test', 'Derivative output matrix must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.B, 1), size(system.A, 1), 'control:outputfeedback:test', 'Control matrix must have same number of rows as system matrix.');
			test.TestSuite.assertEqual(size(system.D, 1), size(system.C, 1), 'control:outputfeedback:test', 'Throughput matrix must have same number of rows as output matrix.');
			test.TestSuite.assertEqual(size(system.D, 2), size(system.B, 2), 'control:outputfeedback:test', 'Throughput matrix must have same number of columns as control matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 1), size(system.D_ref, 1), 'control:outputfeedback:test', 'Throughput matrix of references must have same number of rows as output matrix of references.');
			test.TestSuite.assertEqual(size(system.C_ref, 2), size(system.A, 2), 'control:outputfeedback:test', 'Output matrix of reference must have same number of columns as system matrix.');

			test.TestSuite.assertEqual(E, system.E, 'control:outputfeedback:test', 'Descriptor matrices must be equal.');
			test.TestSuite.assertEqual(A, system.A, 'control:outputfeedback:test', 'System matrices must be equal.');
			test.TestSuite.assertEqual(B, system.B, 'control:outputfeedback:test', 'Control matrices must be equal.');
			test.TestSuite.assertEqual(C, system.C, 'control:outputfeedback:test', 'Output matrices must be equal.');
			test.TestSuite.assertEqual(C_dot, system.C_dot, 'control:outputfeedback:test', 'Derivative Output matrices must be equal.');
			test.TestSuite.assertEqual(D, system.D, 'control:outputfeedback:test', 'Throughput matrices must be equal.');
			test.TestSuite.assertEqual(C_ref, system.C_ref, 'control:outputfeedback:test', 'Output matrices of references must be equal.');
			test.TestSuite.assertEqual(D_ref, system.D_ref, 'control:outputfeedback:test', 'Throughput matrices of references must be equal.');

			RKF_test = {R_test, K_test, F_test};
			test.TestSuite.assertNoException('R_scaled = controller.scalegain(RKF_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj});', 'control:outputfeedback:test', 'Gain scaling function must not throw an exception.');
			R_scaled = controller.scalegain(RKF_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj});
			test.TestSuite.assertEqual(numel(R_scaled), 3, 'control:outputfeedback:test', 'Gain matrix must return three gains on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{1}, 1), size(R_test, 1), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{1}, 2), size(R_test, 2), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{2}, 1), size(K_test, 1), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{2}, 2), size(K_test, 2), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{3}, 1), size(F_test, 1), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{3}, 2), size(F_test, 2), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			R_scaled = controller.scalegain(R_scaled, inv(T_x), inv(T_u), inv(T_y), inv(T_y_dot), inv(T_w), systems{jj});
			for kk = 1:numel(RKF_test)
				test.TestSuite.assertEqual(R_scaled{kk}, RKF_test{kk}, 'control:outputfeedback:test', 'Scaling and unscaling must not change result.');
			end

			test.TestSuite.assert(iscell(r), 'control:outputfeedback:test', 'Proportional gain pattern must be a cell array.');
			test.TestSuite.assertEqual(size(r, 2), 2, 'control:outputfeedback:test', 'Proportional gain pattern must have two elements.');
			test.TestSuite.assertEqual(size(r{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(r{1}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain must have same number of columns as output matrix.');
			if size(r{1}, 3) > 1 || isnumeric(r{1})
				test.TestSuite.assert(isnumeric(r{1}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
				test.TestSuite.assert(isnumeric(r{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
				test.TestSuite.assertEqual(size(r{1}, 3), size(r{2}, 1), 'control:outputfeedback:test', 'Proportional gain equation system must have same number as contraint border.');
				test.TestSuite.assertEqual(size(r{2}, 2), 1, 'control:outputfeedback:test', 'Proportional gain border must have same one column.');
			else
				test.TestSuite.assert(islogical(r{1}), 'control:outputfeedback:test', 'Proportional gain must be of type ''logical''.');
				test.TestSuite.assert(isnumeric(r{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
				test.TestSuite.assertEqual(size(r{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(r{2}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain must have same number of columns as output matrix.');
			end
			test.TestSuite.assert(iscell(k), 'control:outputfeedback:test', 'Derivative gain pattern must be a cell array.');
			test.TestSuite.assertEqual(size(k, 2), 2, 'control:outputfeedback:test', 'Derivative gain pattern must have two elements.');
			test.TestSuite.assertEqual(size(k{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(k{1}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain must have same number of columns as output matrix.');
			if size(k{1}, 3) > 1 || isnumeric(k{1})
				test.TestSuite.assert(isnumeric(k{1}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
				test.TestSuite.assert(isnumeric(k{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
				test.TestSuite.assertEqual(size(k{1}, 3), size(k{2}, 1), 'control:outputfeedback:test', 'Derivative gain equation system must have same number as contraint border.');
				test.TestSuite.assertEqual(size(k{2}, 2), 1, 'control:outputfeedback:test', 'Derivative gain border must have same one column.');
			else
				test.TestSuite.assert(islogical(k{1}), 'control:outputfeedback:test', 'Derivative gain must be of type ''logical''.');
				test.TestSuite.assert(isnumeric(k{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
				test.TestSuite.assertEqual(size(k{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(k{2}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain must have same number of columns as output matrix.');
			end
			test.TestSuite.assert(iscell(f), 'control:outputfeedback:test', 'Prefilter gain pattern must be a cell array.');
			test.TestSuite.assertEqual(size(f, 2), 2, 'control:outputfeedback:test', 'Prefilter gain pattern must have two elements.');
			test.TestSuite.assertEqual(size(f{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(f{1}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain must have same number of columns as reference output matrix.');
			if size(f{1}, 3) > 1 || isnumeric(f{1})
				test.TestSuite.assert(isnumeric(f{1}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
				test.TestSuite.assert(isnumeric(f{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
				test.TestSuite.assertEqual(size(f{1}, 3), size(f{2}, 1), 'control:outputfeedback:test', 'Prefilter gain equation system must have same number as contraint border.');
				test.TestSuite.assertEqual(size(f{2}, 2), 1, 'control:outputfeedback:test', 'Prefilter gain border must have same one column.');
			else
				test.TestSuite.assert(islogical(f{1}), 'control:outputfeedback:test', 'Prefilter gain must be of type ''logical''.');
				test.TestSuite.assert(isnumeric(f{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
				test.TestSuite.assertEqual(size(f{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(f{2}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain must have same number of columns as reference output matrix.');
			end
			if ~isempty(rkf)
				test.TestSuite.assert(iscell(rkf), 'control:outputfeedback:test', 'Combined gain pattern must be a cell array.');
				test.TestSuite.assertEqual(size(rkf, 2), 2, 'control:outputfeedback:test', 'Combined gain pattern must have two elements.');
				test.TestSuite.assertEqual(size(rkf{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(rkf{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain must have same number of columns as output, derivative output and reference output matrix.');
				if size(rkf{1}, 3) > 1 || isnumeric(rkf{1})
					test.TestSuite.assert(isnumeric(rkf{1}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assert(isnumeric(rkf{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf{1}, 3), size(rkf{2}, 1), 'control:outputfeedback:test', 'Combined gain equation system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(rkf{2}, 2), 1, 'control:outputfeedback:test', 'Combined gain border must have same one column.');
				else
					test.TestSuite.assert(islogical(f{1}), 'control:outputfeedback:test', 'Combined gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(f{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(rkf{2}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain must have same number of columns as output, derivative output and reference output matrix.');
				end
			end

			if ~isempty(r_bound)
				test.TestSuite.assert(iscell(r_bound), 'control:outputfeedback:test', 'Proportional gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(r_bound, 2), 2, 'control:outputfeedback:test', 'Proportional gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(r_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(r_bound{1}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of columns as output matrix.');
				if size(r_bound{1}, 3) > 1 || isnumeric(r_bound{1})
					test.TestSuite.assert(isnumeric(r_bound{1}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
					test.TestSuite.assert(isnumeric(r_bound{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
					test.TestSuite.assertEqual(size(r_bound{1}, 3), size(r_bound{2}, 1), 'control:outputfeedback:test', 'Proportional gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(r_bound{2}, 2), 1, 'control:outputfeedback:test', 'Proportional gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(r_bound{1}), 'control:outputfeedback:test', 'Proportional gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(r_bound{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
					test.TestSuite.assertEqual(size(r_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(r_bound{2}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of columns as output matrix.');
				end
			end
			if ~isempty(k_bound)
				test.TestSuite.assert(iscell(k_bound), 'control:outputfeedback:test', 'Derivative gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(k_bound, 2), 2, 'control:outputfeedback:test', 'Derivative gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(k_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(k_bound{1}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of columns as output matrix.');
				if size(k_bound{1}, 3) > 1 || isnumeric(r_bound{1})
					test.TestSuite.assert(isnumeric(k_bound{1}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
					test.TestSuite.assert(isnumeric(k_bound{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
					test.TestSuite.assertEqual(size(k_bound{1}, 3), size(k_bound{2}, 1), 'control:outputfeedback:test', 'Derivative gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(k_bound{2}, 2), 1, 'control:outputfeedback:test', 'Derivative gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(r_bound{1}), 'control:outputfeedback:test', 'Derivative gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(r_bound{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
					test.TestSuite.assertEqual(size(k_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(k_bound{2}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of columns as output matrix.');
				end
			end
			if ~isempty(f_bound)
				test.TestSuite.assert(iscell(f_bound), 'control:outputfeedback:test', 'Prefilter gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(f_bound, 2), 2, 'control:outputfeedback:test', 'Prefilter gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(f_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(f_bound{1}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of columns as output matrix.');
				if size(f_bound{1}, 3) > 1 || isnumeric(f_bound{1})
					test.TestSuite.assert(isnumeric(f_bound{1}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
					test.TestSuite.assert(isnumeric(f_bound{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
					test.TestSuite.assertEqual(size(f_bound{1}, 3), size(f_bound{2}, 1), 'control:outputfeedback:test', 'Prefilter gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(f_bound{2}, 2), 1, 'control:outputfeedback:test', 'Prefilter gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(f_bound{1}), 'control:outputfeedback:test', 'Prefilter gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(f_bound{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
					test.TestSuite.assertEqual(size(f_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(f_bound{2}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of columns as output matrix.');
				end
			end
			if ~isempty(rkf_bound)
				test.TestSuite.assert(iscell(rkf_bound), 'control:outputfeedback:test', 'Combined gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(rkf_bound, 2), 2, 'control:outputfeedback:test', 'Combined gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(rkf_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(rkf_bound{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain bounds must have same number of columns as output, derivative output and reference output matrix.');
				if size(rkf_bound{1}, 3) > 1 || isnumeric(rkf_bound{1})
					test.TestSuite.assert(isnumeric(rkf_bound{1}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assert(isnumeric(rkf_bound{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf_bound{1}, 3), size(rkf_bound{2}, 1), 'control:outputfeedback:test', 'Combined gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(rkf_bound{2}, 2), 1, 'control:outputfeedback:test', 'Combined gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(rkf_bound{1}), 'control:outputfeedback:test', 'Combined gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(rkf_bound{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(rkf_bound{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain bounds must have same number of columns as output, derivative output and reference output matrix.');
				end
			end
			test.TestSuite.assertNoException('[f, f_fixed] = controller.prefilterpattern({r{1}(:, :, 1), k{1}(:, :, 1)}, systems{jj});', 'control:outputfeedback:test', 'Prefilterpattern of outputfeedback class must not throw an exception.');
			[f, f_fixed] = controller.prefilterpattern({r{1}(:, :, 1), k{1}(:, :, 1)}, systems{jj});
			test.TestSuite.assert(islogical(f_fixed), 'control:outputfeedback:test', 'Prefilter gain pattern must be a logical matrix.');
			test.TestSuite.assert(isnumeric(f), 'control:outputfeedback:test', 'Prefilter must be a numeric matrix.');
			test.TestSuite.assertEqual(size(f, 1), size(B, 2), 'control:outputfeedback:test', 'Prefilter gain must have same number of rows as columns in B.');
			test.TestSuite.assertEqual(size(f, 1), size(f_fixed, 1), 'control:outputfeedback:test', 'Prefilter gain pattern and prefilter gain must have same number of columns.');
			test.TestSuite.assertEqual(size(f, 2), size(f_fixed, 2), 'control:outputfeedback:test', 'Prefilter gain pattern and prefilter gain must have same number of rows.');
			F_test = double(f);
			if ~isempty(r_nonlin)
				test.TestSuite.assert(isfunctionhandle(r_nonlin), 'control:outputfeedback:test', 'Nonlinear gain function must be a function handle.');
				test.TestSuite.assertEqual(nargin(r_nonlin), 3, 'control:outputfeedback:test', 'Nonlinear gain function must have 3 input arguments.');
				test.TestSuite.assertNoException('[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = r_nonlin(R_test, K_test, F_test);', 'control:outputfeedback:test', 'Nonlinear gain constraint function must not throw an exception for 6 output arguments.');
				if nargout(r_nonlin) >= 12
					test.TestSuite.assertNoException('[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);', 'control:outputfeedback:test', 'Nonlinear gain constraint function must not throw an exception for 12 output arguments.');
					[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);
				else
					test.TestSuite.assertNoExceptionExcept('[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);', {
						'MATLAB:maxlhs';
						'MATLAB:TooManyOutputs';
						'MATLAB:unassignedOutputs'
					}, 'control:outputfeedback:test', 'Nonlinear gain constraint function must not throw an exception for 12 output arguments.');
					try
						[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);
					catch e
						[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = r_nonlin(R_test, K_test, F_test);
						gradc_R = NaN(size(system.B, 2), size(system.C, 1), size(c_R, 1));
						gradceq_R = NaN(size(system.B, 2), size(system.C, 1), size(ceq_R, 1));
						gradc_K = NaN(size(system.B, 2), size(system.C_dot, 1), size(c_K, 1));
						gradceq_K = NaN(size(system.B, 2), size(system.C_dot, 1), size(ceq_K, 1));
						gradc_F = NaN(size(system.B, 2), size(F_test, 2), size(c_F, 1));
						gradceq_F = NaN(size(system.B, 2), size(F_test, 2), size(ceq_F, 1));
					end
				end
				if isempty(c_R)
					test.TestSuite.assert(isempty(gradc_R), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradc_R, 3), size(c_R, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradc_R, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradc_R, 2), size(system.C, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must same number of rows as control matrix.');
				end
				if isempty(ceq_R)
					test.TestSuite.assert(isempty(gradceq_R), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradceq_R, 3), size(ceq_R, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradceq_R, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradceq_R, 2), size(system.C, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must same number of rows as control matrix.');
				end
				if isempty(c_K)
					test.TestSuite.assert(isempty(gradc_K), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradc_K, 3), size(c_K, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradc_K, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradc_K, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must same number of rows as control matrix.');
				end
				if isempty(ceq_K)
					test.TestSuite.assert(isempty(gradceq_K), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradceq_K, 3), size(ceq_K, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradceq_K, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradceq_K, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must same number of rows as control matrix.');
				end
				if isempty(c_F)
					test.TestSuite.assert(isempty(gradc_F), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradc_F, 3), size(c_F, 1), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradc_F, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradc_F, 2), size(F_test, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must same number of rows as prefilter matrix.');
				end
				if isempty(ceq_F)
					test.TestSuite.assert(isempty(gradceq_F), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradceq_F, 3), size(ceq_F, 1), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradceq_F, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradceq_F, 2), size(F_test, 1), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must same number of rows as prefilter matrix.');
				end
			end

			test.TestSuite.assertNoException('[r_parametric] = controller.gainpattern_parametric(systems{jj});', 'control:outputfeedback:test', 'Parametric gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r_parametric, k_parametric] = controller.gainpattern_parametric(systems{jj});', 'control:outputfeedback:test', 'Parametric gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r_parametric, k_parametric, f_parametric] = controller.gainpattern_parametric(systems{jj});', 'control:outputfeedback:test', 'Parametric gainpattern of outputfeedback class must not throw an exception.');
			[r_parametric, k_parametric, f_parametric] = controller.gainpattern_parametric(systems{jj});
			test.TestSuite.assertEqual(size(r_parametric, 1), size(B, 2), 'control:outputfeedback:test', 'Parametric proportional gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(r_parametric, 2), size(C, 1), 'control:outputfeedback:test', 'Parametric proportional gain must have same number of columns as output matrix.');
			test.TestSuite.assertEqual(size(k_parametric, 1), size(B, 2), 'control:outputfeedback:test', 'Parametric derivative gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(k_parametric, 2), size(C_dot, 1), 'control:outputfeedback:test', 'Parametric derivative gain must have same number of columns as output matrix.');
			test.TestSuite.assertEqual(size(f_parametric, 1), size(B, 2), 'control:outputfeedback:test', 'Parametric prefilter gain must have same number of rows as columns in B.');
			test.TestSuite.assertEqual(size(f_parametric, 2), size(f_fixed, 2), 'control:outputfeedback:test', 'Parametric prefilter gain pattern and prefilter gain must have same number of rows.');
			if islogical(r{1}) && ~isempty(r_parametric)
				if isa(r_parametric, 'realp')% TODO: also check genmat (how?)
					isfree = r_parametric.Free;
					test.TestSuite.assertEqual(~isfree, r{1}, 'control:outputfeedback:test', 'Parametric gain pattern must match numeric gain pattern.');
					if any(~isfree)
						test.TestSuite.assertEqual(double(r_parametric(~isfree)), r{2}(~isfree), 'control:outputfeedback:test', 'Parametric gain pattern values must match numeric gain pattern values.');
					end
				end
			end
			if islogical(k{1}) && ~isempty(k_parametric)
				if isa(k_parametric, 'realp')
					isfree = k_parametric.Free;
					test.TestSuite.assertEqual(~isfree, k{1}, 'control:outputfeedback:test', 'Parametric derivative gain pattern must match numeric derivative gain pattern.');
					if any(~isfree)
						test.TestSuite.assertEqual(double(k_parametric(~isfree)), k{2}(~isfree), 'control:outputfeedback:test', 'Parametric derivative gain pattern values must match numeric derivative gain pattern values.');
					end
				end
			end
			if islogical(f_fixed) && ~isempty(f_parametric)
				if isa(f_parametric, 'realp')
					isfree = f_parametric.Free;
					test.TestSuite.assertEqual(~isfree, f_fixed, 'control:outputfeedback:test', 'Parametric prefilter pattern must match numeric prefilter pattern.');
					if any(~isfree)
						test.TestSuite.assertEqual(double(f_parametric(~isfree)), f(~isfree), 'control:outputfeedback:test', 'Parametric prefilter pattern values must match numeric prefilter pattern values.');
					end
				end
			end


			test.TestSuite.assertNoException('F_scaled = controller.scaleprefilter(F_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj});', 'control:outputfeedback:test', 'Prefilter scaling function must not throw an exception.');
			F_scaled = controller.scaleprefilter(F_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj});
			test.TestSuite.assertEqual(size(F_scaled, 1), size(F_test, 1), 'control:outputfeedback:test', 'Prefilter matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(F_scaled, 2), size(F_test, 2), 'control:outputfeedback:test', 'Prefilter matrix must not change size on scaling.');
			F_scaled = controller.scaleprefilter(F_scaled, inv(T_x), inv(T_u), inv(T_y), inv(T_y_dot), inv(T_w), systems{jj});
			test.TestSuite.assertEqual(F_scaled, F_test, 'control:outputfeedback:test', 'Scaling and unscaling must not change result.');

			test.TestSuite.assertNoException('[partitionR, partitionF] = controller.gainpartitioning({r{1}(:, :, 1), k{1}(:, :, 1)}, f, systems{jj});', 'control:outputfeedback:test', 'Gainpartitioning of outputfeedback class must not throw an exception.');
			[partitionR, partitionF] = controller.gainpartitioning({r{1}(:, :, 1), k{1}(:, :, 1)}, f, systems{jj});
			test.TestSuite.assert(isstruct(partitionR), 'control:outputfeedback:test', 'Partitioning of R must be a structure.');
			test.TestSuite.assert(isstruct(partitionF), 'control:outputfeedback:test', 'Partitioning of F must be a structure.');

			test.TestSuite.assertNoException('[system_controller, ~, ~, ~, ~, ~, ~, ~, needsstate, useCasCdot] = controller.realization(R_test, F_test, systems{jj});', 'control:outputfeedback:test', 'Returning controllerfor outputfeedback class must not throw an exception.');
			[system_controller, ~, ~, ~, ~, ~, ~, ~, needsstate, useCasCdot] = controller.realization(R_test, F_test, systems{jj});
			test.TestSuite.assert(islogical(needsstate), 'control:outputfeedback:test', 'State indicator must be of type ''logical''.');
			test.TestSuite.assert(isscalar(needsstate), 'control:outputfeedback:test', 'State indicator must be scalar.');
			if isa(system_controller, 'ss')
				[A_controller, B_controller, C_controller, D_controller] = ssdata(system_controller);
				if isempty(system_controller.e)
					E_controller = eye(size(A_controller, 1));
				else
					E_controller = system_controller.e;
				end
				system_controller = struct(...
					'E',		E_controller,...
					'A',		A_controller,...
					'B',		B_controller,...
					'C',		C_controller,...
					'C_dot',	zeros(size(C_controller, 1), size(A_controller, 1)),...
					'D',		D_controller,...
					'C_ref',	C_controller,...
					'D_ref',	D_controller...
				);
			elseif isa(system_controller, 'tf')
				[A_controller, B_controller, C_controller, D_controller] = ssdata(system_controller);
				E_controller = eye(size(A_controller, 1));
				system_controller = struct(...
					'E',		E_controller,...
					'A',		A_controller,...
					'B',		B_controller,...
					'C',		C_controller,...
					'C_dot',	zeros(size(C_controller, 1), size(A_controller, 1)),...
					'D',		D_controller,...
					'C_ref',	C_controller,...
					'D_ref',	D_controller...
				);
			end
			test.TestSuite.assertEqual(size(system_controller.A, 1), size(system_controller.A, 2), 'control:outputfeedback:test', 'System matrix of controller must be square.');
			test.TestSuite.assertEqual(size(system_controller.E, 1), size(system_controller.E, 2), 'control:outputfeedback:test', 'Descriptor matrix of controller must be square.');
			test.TestSuite.assertEqual(size(system_controller.A, 1), size(system_controller.E, 1), 'control:outputfeedback:test', 'System matrix of controller must have the same size as descriptor matrix.');
			test.TestSuite.assertEqual(size(system_controller.C, 2), size(system_controller.A, 1), 'control:outputfeedback:test', 'Output matrix of controller must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system_controller.C_dot, 2), size(system_controller.A, 1), 'control:outputfeedback:test', 'Derivative output matrix of controller must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system_controller.D, 1), size(system_controller.C, 1), 'control:outputfeedback:test', 'Throughput matrix of controller must have same number of rows as output matrix.');
			test.TestSuite.assertEqual(size(system_controller.D, 2), size(system_controller.B, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have same number of columns as control matrix.');
			test.TestSuite.assertEqual(size(system_controller.C_ref, 2), size(system_controller.A, 2), 'control:outputfeedback:test', 'Output matrix of references of controller must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system_controller.C_ref, 1), size(system_controller.D_ref, 1), 'control:outputfeedback:test', 'Throughput matrix of references of controller must have same number of columns as output matrix of references.');

			testsystem = systems{jj};
			checkdim = (~isstruct(testsystem) && isct(testsystem)) || (isstruct(testsystem) && (~isfield(testsystem, 'T') || testsystem.T == -1));
			checkdim = checkdim && ~isa(controller, 'control.design.outputfeedback.DynamicOutputFeedback');% DynamicOutputFeedback does not have size(C, 1) desired values
			if checkdim% only valid for continuous time systems
				test.TestSuite.assertEqual(size(basesystem.B, 2), size(system_controller.C, 1), 'control:outputfeedback:test', 'Output matrix of controller must have as much rows as B.');
				test.TestSuite.assertEqual(size(basesystem.B, 2), size(system_controller.D, 1), 'control:outputfeedback:test', 'Throughput matrix of controller must have as much rows as B.');
				if needsstate
					if useCasCdot
						if isa(controller, 'control.design.outputfeedback.PDStateFeedback')
							test.TestSuite.assertEqual(3*size(basesystem.A, 1) + size(basesystem.C, 1), size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
							test.TestSuite.assertEqual(3*size(basesystem.A, 1) + size(basesystem.C, 1), size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
						else
							test.TestSuite.assertEqual(size(basesystem.A, 1) + 3*size(basesystem.C, 1), size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
							test.TestSuite.assertEqual(size(basesystem.A, 1) + 3*size(basesystem.C, 1), size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
						end
					else
						test.TestSuite.assertEqual(size(basesystem.A, 1) + size(basesystem.C, 1) + size(basesystem.C_dot, 1)*2, size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
						test.TestSuite.assertEqual(size(basesystem.A, 1) + size(basesystem.C, 1) + size(basesystem.C_dot, 1)*2, size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
					end
				else
					if useCasCdot
						test.TestSuite.assertEqual(size(basesystem.C, 1)*4, size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
						test.TestSuite.assertEqual(size(basesystem.C, 1)*4, size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
					else
						test.TestSuite.assertEqual(size(basesystem.C, 1)*2 + size(basesystem.C_dot, 1)*2, size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
						test.TestSuite.assertEqual(size(basesystem.C, 1)*2 + size(basesystem.C_dot, 1)*2, size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as two times the number of outputs and derivative outputs of the system.');
					end
				end
			end
			test.TestSuite.assertNoException('controller.amend_parametric(systems{jj});', 'control:outputfeedback:test', 'Amending system with parametric outputfeedback class must not throw an exception.');
			system = controller.amend_parametric(systems{jj});
			if isa(system, 'ss') || isa(system, 'genss')
				[A, B, C, D] = ssdata(system);
				if isa(system, 'genss') || isempty(system.e)
					E = eye(size(A, 1));
				else
					E = system.e;
				end
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	zeros(size(C, 1), size(A, 1)),...
					'D',		D,...
					'C_ref',	C,...
					'D_ref',	D...
				);
			elseif isa(system, 'tf')
				[A, B, C, D] = ssdata(system);
				E = eye(size(A, 1));
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	zeros(size(C, 1), size(A, 1)),...
					'D',		D,...
					'C_ref',	C,...
					'D_ref',	D...
				);
			end
			test.TestSuite.assertEqual(size(system.A, 1), size(system.A, 2), 'control:outputfeedback:test', 'System matrix of parametric augmented system must be square.');
			test.TestSuite.assertEqual(size(system.E, 1), size(system.E, 2), 'control:outputfeedback:test', 'Descriptor matrix of parametric augmented system must be square.');
			test.TestSuite.assertEqual(size(system.A, 1), size(system.E, 1), 'control:outputfeedback:test', 'System matrix of parametric augmented system must have the same size as descriptor matrix.');
			test.TestSuite.assertEqual(size(system.C, 2), size(system.A, 1), 'control:outputfeedback:test', 'Output matrix of parametric augmented system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_dot, 2), size(system.A, 1), 'control:outputfeedback:test', 'Derivative output matrix of parametric augmented system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.D, 1), size(system.C, 1), 'control:outputfeedback:test', 'Throughput matrix of parametric augmented system must have same number of rows as output matrix.');
			test.TestSuite.assertEqual(size(system.D, 2), size(system.B, 2), 'control:outputfeedback:test', 'Throughput matrix of parametric augmented system must have same number of columns as control matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 2), size(system.A, 2), 'control:outputfeedback:test', 'Output matrix of references of parametric augmented system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 1), size(system.D_ref, 1), 'control:outputfeedback:test', 'Throughput matrix of references of parametric augmented system must have same number of columns as output matrix of references.');

			% discrete time
			if isa(systems{jj}, 'ss')
				if systems{jj}.Ts > 0
					s = systems{jj};
				else
					s = c2d(systems{jj}, T);
				end
			elseif isa(system, 'tf')
				if systems{jj}.Ts > 0
					s = systems{jj};
				else
					s = c2d(systems{jj}, T);
				end
			elseif isa(systems{jj}, 'struct')
				s = systems{jj};
				if ~isfield(s, 'T')
					s.A = expm(s.A*T);
				end
			else
				s = systems{jj};
			end
			test.TestSuite.assertNoException('cls(s, T);', 'control:outputfeedback:test', 'Construction of a discrete outputfeedback class must not throw an exception.');
			controllerT = cls(s, T);
			test.TestSuite.assertNoException('controllerT.amend(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.E(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning E must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.A(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning A must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.B(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning B must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.C(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning C must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.C_dot(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning C_dot must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.D(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning D must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.C_ref(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning C_ref must not throw an exception.');
			test.TestSuite.assertNoException('controllerT.D_ref(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with outputfeedback class and returning D_ref must not throw an exception.');

			test.TestSuite.assertNoException('[r] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound, f_bound] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound, f_bound, rkf_bound] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r, k, f, rkf, r_bound, k_bound, f_bound, rkf_bound, r_nonlin] = controllerT.gainpattern(systems{jj}, T);', 'control:outputfeedback:test', 'Gainpattern of discrete outputfeedback class must not throw an exception.');
			system = controllerT.amend(systems{jj}, T);
			if isa(system, 'ss')
				[A, B, C, D] = ssdata(system);
				if isempty(system.e)
					E = eye(size(A, 1));
				else
					E = system.e;
				end
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	controllerT.C_dot(systems{jj}, T),...
					'D',		D,...
					'C_ref',	controllerT.C_ref(systems{jj}, T),...
					'D_ref',	controllerT.D_ref(systems{jj}, T)...
				);
			elseif isa(system, 'tf')
				[A, B, C, D] = ssdata(system);
				E = eye(size(A, 1));
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	controllerT.C_dot(systems{jj}, T),...
					'D',		D,...
					'C_ref',	controllerT.C_ref(systems{jj}, T),...
					'D_ref',	controllerT.D_ref(systems{jj}, T)...
				);
			end
			E = controllerT.E(systems{jj}, T);
			A = controllerT.A(systems{jj}, T);
			B = controllerT.B(systems{jj}, T);
			C = controllerT.C(systems{jj}, T);
			C_dot = controllerT.C_dot(systems{jj}, T);
			D = controllerT.D(systems{jj}, T);
			C_ref = controllerT.C_ref(systems{jj}, T);
			D_ref = controllerT.D_ref(systems{jj}, T);
			[r, k, f, rkf, r_bound, k_bound, f_bound, rkf_bound, r_nonlin] = controllerT.gainpattern(systems{jj}, T);
			R_test = double(r{1}(:, :, 1));
			K_test = double(k{1}(:, :, 1));
			F_test = double(f{1}(:, :, 1));
			test.TestSuite.assertEqual(size(system.A, 1), size(system.A, 2), 'control:outputfeedback:test', 'System matrix of discrete system must be square.');
			test.TestSuite.assertEqual(size(system.E, 1), size(system.E, 2), 'control:outputfeedback:test', 'Descriptor matrix of discrete system must be square.');
			test.TestSuite.assertEqual(size(system.A, 1), size(system.E, 1), 'control:outputfeedback:test', 'System matrix of discrete system must have the same size as descriptor matrix.');
			test.TestSuite.assertEqual(size(system.C, 2), size(system.A, 1), 'control:outputfeedback:test', 'Output matrix of discrete system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_dot, 2), size(system.A, 1), 'control:outputfeedback:test', 'Derivative output matrix of discrete system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.B, 1), size(system.A, 1), 'control:outputfeedback:test', 'Control matrix of discrete system must have same number of rows as system matrix.');
			test.TestSuite.assertEqual(size(system.D, 1), size(system.C, 1), 'control:outputfeedback:test', 'Throughput matrix of discrete system must have same number of rows as output matrix.');
			test.TestSuite.assertEqual(size(system.D, 2), size(system.B, 2), 'control:outputfeedback:test', 'Throughput matrix of discrete system must have same number of columns as control matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 2), size(system.A, 1), 'control:outputfeedback:test', 'Output matrix of references of discrete system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 1), size(system.D_ref, 1), 'control:outputfeedback:test', 'Throughput matrix of references of discrete system must have same number of columns as output matrix of references.');

			test.TestSuite.assertEqual(E, system.E, 'control:outputfeedback:test', 'Descriptor matrices of discrete system must be equal.');
			test.TestSuite.assertEqual(A, system.A, 'control:outputfeedback:test', 'System matrices of discrete system must be equal.');
			test.TestSuite.assertEqual(B, system.B, 'control:outputfeedback:test', 'Control matrices of discrete system must be equal.');
			test.TestSuite.assertEqual(C, system.C, 'control:outputfeedback:test', 'Output matrices of discrete system must be equal.');
			test.TestSuite.assertEqual(C_dot, system.C_dot, 'control:outputfeedback:test', 'Derivative Output matrices of discrete system must be equal.');
			test.TestSuite.assertEqual(D, system.D, 'control:outputfeedback:test', 'Throughput matrices of discrete system must be equal.');
			test.TestSuite.assertEqual(C_ref, system.C_ref, 'control:outputfeedback:test', 'Output matrices of references of discrete system must be equal.');
			test.TestSuite.assertEqual(D_ref, system.D_ref, 'control:outputfeedback:test', 'Throughput matrices of references of discrete system must be equal.');

			RKF_test = {R_test, K_test, F_test};
			test.TestSuite.assertNoException('R_scaled = controller.scalegain(RKF_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj}, T);', 'control:outputfeedback:test', 'Gain scaling function must not throw an exception.');
			R_scaled = controller.scalegain(RKF_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj}, T);
			test.TestSuite.assertEqual(numel(R_scaled), 3, 'control:outputfeedback:test', 'Gain matrix must return three gains on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{1}, 1), size(R_test, 1), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{1}, 2), size(R_test, 2), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{2}, 1), size(K_test, 1), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{2}, 2), size(K_test, 2), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{3}, 1), size(F_test, 1), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(R_scaled{3}, 2), size(F_test, 2), 'control:outputfeedback:test', 'Gain matrix must not change size on scaling.');
			R_scaled = controller.scalegain(R_scaled, inv(T_x), inv(T_u), inv(T_y), inv(T_y_dot), inv(T_w), systems{jj}, T);
			for kk = 1:numel(RKF_test)
				test.TestSuite.assertEqual(R_scaled{kk}, RKF_test{kk}, 'control:outputfeedback:test', 'Scaling and unscaling must not change result.');
			end

			test.TestSuite.assert(iscell(r), 'control:outputfeedback:test', 'Proportional discrete gain pattern must be a cell array.');
			test.TestSuite.assertEqual(size(r, 2), 2, 'control:outputfeedback:test', 'Proportional discrete gain pattern must have two elements.');
			test.TestSuite.assertEqual(size(r{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional discrete gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(r{1}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional discrete gain must have same number of columns as output matrix.');
			if size(r{1}, 3) > 1 || isnumeric(r{1})
				test.TestSuite.assert(isnumeric(r{1}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
				test.TestSuite.assert(isnumeric(r{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
				test.TestSuite.assertEqual(size(r{1}, 3), size(r{2}, 1), 'control:outputfeedback:test', 'Proportional gain equation system must have same number as contraint border.');
				test.TestSuite.assertEqual(size(r{2}, 2), 1, 'control:outputfeedback:test', 'Proportional gain border must have same one column.');
			else
				test.TestSuite.assert(islogical(r{1}), 'control:outputfeedback:test', 'Proportional gain must be of type ''logical''.');
				test.TestSuite.assert(isnumeric(r{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
				test.TestSuite.assertEqual(size(r{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(r{2}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain must have same number of columns as output matrix.');
			end
			test.TestSuite.assert(iscell(k), 'control:outputfeedback:test', 'Derivative discrete gain pattern must be a cell array.');
			test.TestSuite.assertEqual(size(k, 2), 2, 'control:outputfeedback:test', 'Derivative discrete gain pattern must have two elements.');
			test.TestSuite.assertEqual(size(k{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative discrete gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(k{1}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative discrete gain must have same number of columns as output matrix.');
			if size(k{1}, 3) > 1 || isnumeric(k{1})
				test.TestSuite.assert(isnumeric(k{1}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
				test.TestSuite.assert(isnumeric(k{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
				test.TestSuite.assertEqual(size(k{1}, 3), size(k{2}, 1), 'control:outputfeedback:test', 'Derivative gain equation system must have same number as contraint border.');
				test.TestSuite.assertEqual(size(k{2}, 2), 1, 'control:outputfeedback:test', 'Derivative gain border must have same one column.');
			else
				test.TestSuite.assert(islogical(k{1}), 'control:outputfeedback:test', 'Derivative gain must be of type ''logical''.');
				test.TestSuite.assert(isnumeric(k{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
				test.TestSuite.assertEqual(size(k{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(k{2}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain must have same number of columns as output matrix.');
			end
			test.TestSuite.assert(iscell(f), 'control:outputfeedback:test', 'Prefilter discrete gain pattern must be a cell array.');
			test.TestSuite.assertEqual(size(f, 2), 2, 'control:outputfeedback:test', 'Prefilter discrete gain pattern must have two elements.');
			test.TestSuite.assertEqual(size(f{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter discrete gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(f{1}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter discrete gain must have same number of columns as output matrix of references.');
			if size(f{1}, 3) > 1 || isnumeric(f{1})
				test.TestSuite.assert(isnumeric(f{1}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
				test.TestSuite.assert(isnumeric(f{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
				test.TestSuite.assertEqual(size(f{1}, 3), size(f{2}, 1), 'control:outputfeedback:test', 'Prefilter gain equation system must have same number as contraint border.');
				test.TestSuite.assertEqual(size(f{2}, 2), 1, 'control:outputfeedback:test', 'Prefilter gain border must have same one column.');
			else
				test.TestSuite.assert(islogical(f{1}), 'control:outputfeedback:test', 'Prefilter gain must be of type ''logical''.');
				test.TestSuite.assert(isnumeric(f{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
				test.TestSuite.assertEqual(size(f{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(f{2}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain must have same number of columns as output matrix.');
			end
			if ~isempty(rkf)
				test.TestSuite.assert(iscell(rkf), 'control:outputfeedback:test', 'Combined gain pattern must be a cell array.');
				test.TestSuite.assertEqual(size(rkf, 2), 2, 'control:outputfeedback:test', 'Combined gain pattern must have two elements.');
				test.TestSuite.assertEqual(size(rkf{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(rkf{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain must have same number of columns as output, derivative output and reference output matrix.');
				if size(rkf{1}, 3) > 1 || isnumeric(rkf{1})
					test.TestSuite.assert(isnumeric(rkf{1}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assert(isnumeric(rkf{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf{1}, 3), size(rkf{2}, 1), 'control:outputfeedback:test', 'Combined gain equation system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(rkf{2}, 2), 1, 'control:outputfeedback:test', 'Combined gain border must have same one column.');
				else
					test.TestSuite.assert(islogical(rkf{1}), 'control:outputfeedback:test', 'Combined gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(rkf{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(rkf{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain must have same number of columns as output, derivative output and reference output matrix.');
				end
			end

			if ~isempty(r_bound)
				test.TestSuite.assert(iscell(r_bound), 'control:outputfeedback:test', 'Proportional gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(r_bound, 2), 2, 'control:outputfeedback:test', 'Proportional gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(r_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(r_bound{1}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of columns as output matrix.');
				if size(r_bound{1}, 3) > 1 || isnumeric(r_bound{1})
					test.TestSuite.assert(isnumeric(r_bound{1}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
					test.TestSuite.assert(isnumeric(r_bound{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
					test.TestSuite.assertEqual(size(r_bound{1}, 3), size(r_bound{2}, 1), 'control:outputfeedback:test', 'Proportional gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(r_bound{2}, 2), 1, 'control:outputfeedback:test', 'Proportional gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(r_bound{1}), 'control:outputfeedback:test', 'Proportional gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(r_bound{2}), 'control:outputfeedback:test', 'Proportional gain must be numeric.');
					test.TestSuite.assertEqual(size(r_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(r_bound{2}, 2), size(system.C, 1), 'control:outputfeedback:test', 'Proportional gain bounds must have same number of columns as output matrix.');
				end
			end
			if ~isempty(k_bound)
				test.TestSuite.assert(iscell(k_bound), 'control:outputfeedback:test', 'Derivative gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(k_bound, 2), 2, 'control:outputfeedback:test', 'Derivative gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(k_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(k_bound{1}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of columns as output matrix.');
				if size(k_bound{1}, 3) > 1 || isnumeric(k_bound{1})
					test.TestSuite.assert(isnumeric(k_bound{1}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
					test.TestSuite.assert(isnumeric(k_bound{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
					test.TestSuite.assertEqual(size(k_bound{1}, 3), size(k_bound{2}, 1), 'control:outputfeedback:test', 'Derivative gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(k_bound{2}, 2), 1, 'control:outputfeedback:test', 'Derivative gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(k_bound{1}), 'control:outputfeedback:test', 'Derivative gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(k_bound{2}), 'control:outputfeedback:test', 'Derivative gain must be numeric.');
					test.TestSuite.assertEqual(size(k_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(k_bound{2}, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Derivative gain bounds must have same number of columns as output matrix.');
				end
			end
			if ~isempty(f_bound)
				test.TestSuite.assert(iscell(f_bound), 'control:outputfeedback:test', 'Prefilter gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(f_bound, 2), 2, 'control:outputfeedback:test', 'Prefilter gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(f_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(f_bound{1}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of columns as output matrix of references.');
				if size(f_bound{1}, 3) > 1 || isnumeric(f_bound{1})
					test.TestSuite.assert(isnumeric(f_bound{1}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
					test.TestSuite.assert(isnumeric(f_bound{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
					test.TestSuite.assertEqual(size(f_bound{1}, 3), size(f_bound{2}, 1), 'control:outputfeedback:test', 'Prefilter gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(f_bound{2}, 2), 1, 'control:outputfeedback:test', 'Prefilter gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(f_bound{1}), 'control:outputfeedback:test', 'Prefilter gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(f_bound{2}), 'control:outputfeedback:test', 'Prefilter gain must be numeric.');
					test.TestSuite.assertEqual(size(f_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(f_bound{2}, 2), size(system.C_ref, 1), 'control:outputfeedback:test', 'Prefilter gain bounds must have same number of columns as output matrix.');
				end
			end
			if ~isempty(rkf_bound)
				test.TestSuite.assert(iscell(rkf_bound), 'control:outputfeedback:test', 'Combined gain bound pattern must be a cell array.');
				test.TestSuite.assertEqual(size(rkf_bound, 2), 2, 'control:outputfeedback:test', 'Combined gain bound pattern must have two elements.');
				test.TestSuite.assertEqual(size(rkf_bound{1}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain bounds must have same number of rows as control matrix.');
				test.TestSuite.assertEqual(size(rkf_bound{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain bounds must have same number of columns as output, derivative output and reference output matrix.');
				if size(rkf_bound{1}, 3) > 1 || isnumeric(rkf_bound{1})
					test.TestSuite.assert(isnumeric(rkf_bound{1}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assert(isnumeric(rkf_bound{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf_bound{1}, 3), size(rkf_bound{2}, 1), 'control:outputfeedback:test', 'Combined gain bound system must have same number as contraint border.');
					test.TestSuite.assertEqual(size(rkf_bound{2}, 2), 1, 'control:outputfeedback:test', 'Combined gain bound border must have same one column.');
				else
					test.TestSuite.assert(islogical(rkf_bound{1}), 'control:outputfeedback:test', 'Combined gain must be of type ''logical''.');
					test.TestSuite.assert(isnumeric(rkf_bound{2}), 'control:outputfeedback:test', 'Combined gain must be numeric.');
					test.TestSuite.assertEqual(size(rkf_bound{2}, 1), size(system.B, 2), 'control:outputfeedback:test', 'Combined gain bounds must have same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(rkf_bound{1}, 2), size(system.C, 1) + size(system.C_dot, 1) + size(system.C_ref, 1), 'control:outputfeedback:test', 'Combined gain bounds must have same number of columns as output, derivative output and reference output matrix.');
				end
			end
			test.TestSuite.assertNoException('[f, f_fixed] = controllerT.prefilterpattern({r{1}(:, :, 1), k{1}(:, :, 1)}, systems{jj}, T);', 'control:outputfeedback:test', 'Prefilterpattern of discrete outputfeedback class must not throw an exception.');
			[f, f_fixed] = controllerT.prefilterpattern({r{1}(:, :, 1), k{1}(:, :, 1)}, systems{jj}, T);
			test.TestSuite.assert(islogical(f_fixed), 'control:outputfeedback:test', 'Prefilter gain pattern of discrete system must be a logical matrix.');
			test.TestSuite.assert(isnumeric(f), 'control:outputfeedback:test', 'Prefilter of discrete system must be a numeric matrix.');
			test.TestSuite.assertEqual(size(f, 1), size(B, 2), 'control:outputfeedback:test', 'Prefilter gain of discrete system must have same number of rows as columns in B.');
			test.TestSuite.assertEqual(size(f, 1), size(f_fixed, 1), 'control:outputfeedback:test', 'Prefilter gain of discrete system pattern and prefilter gain must have same number of columns.');
			test.TestSuite.assertEqual(size(f, 2), size(f_fixed, 2), 'control:outputfeedback:test', 'Prefilter gain of discrete system pattern and prefilter gain must have same number of rows.');
			F_test = double(f);
			if ~isempty(r_nonlin)
				test.TestSuite.assert(isfunctionhandle(r_nonlin), 'control:outputfeedback:test', 'Nonlinear gain function must be a function handle.');
				test.TestSuite.assertEqual(nargin(r_nonlin), 3, 'control:outputfeedback:test', 'Nonlinear gain function must have 3 input arguments.');
				test.TestSuite.assertNoException('[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = r_nonlin(R_test, K_test, F_test);', 'control:outputfeedback:test', 'Nonlinear gain constraint function must not throw an exception for 6 output arguments.');
				if nargout(r_nonlin) >= 12
					test.TestSuite.assertNoException('[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);', 'control:outputfeedback:test', 'Nonlinear gain constraint function must not throw an exception for 12 output arguments.');
					[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);
				else
					test.TestSuite.assertNoExceptionExcept('[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);', {
						'MATLAB:maxlhs';
						'MATLAB:TooManyOutputs';
						'MATLAB:unassignedOutputs'
					}, 'control:outputfeedback:test', 'Nonlinear gain constraint function must not throw an exception for 12 output arguments.');
					try
						[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = r_nonlin(R_test, K_test, F_test);
					catch e
						[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = r_nonlin(R_test, K_test, F_test);
						gradc_R = NaN(size(system.B, 2), size(system.C, 1), size(c_R, 1));
						gradceq_R = NaN(size(system.B, 2), size(system.C, 1), size(ceq_R, 1));
						gradc_K = NaN(size(system.B, 2), size(system.C_dot, 1), size(c_K, 1));
						gradceq_K = NaN(size(system.B, 2), size(system.C_dot, 1), size(ceq_K, 1));
						gradc_F = NaN(size(system.B, 2), size(F_test, 2), size(c_F, 1));
						gradceq_F = NaN(size(system.B, 2), size(F_test, 2), size(ceq_F, 1));
					end
				end
				if isempty(c_R)
					test.TestSuite.assert(isempty(gradc_R), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradc_R, 3), size(c_R, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradc_R, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradc_R, 2), size(system.C, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain inequality constraint must same number of rows as control matrix.');
				end
				if isempty(ceq_R)
					test.TestSuite.assert(isempty(gradceq_R), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradceq_R, 3), size(ceq_R, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradceq_R, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradceq_R, 2), size(system.C, 1), 'control:outputfeedback:test', 'Gradient of nonlinear proportional gain equality constraint must same number of rows as control matrix.');
				end
				if isempty(c_K)
					test.TestSuite.assert(isempty(gradc_K), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradc_K, 3), size(c_K, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradc_K, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradc_K, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain inequality constraint must same number of rows as control matrix.');
				end
				if isempty(ceq_K)
					test.TestSuite.assert(isempty(gradceq_K), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradceq_K, 3), size(ceq_K, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradceq_K, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradceq_K, 2), size(system.C_dot, 1), 'control:outputfeedback:test', 'Gradient of nonlinear derivative gain equality constraint must same number of rows as control matrix.');
				end
				if isempty(c_F)
					test.TestSuite.assert(isempty(gradc_F), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradc_F, 3), size(c_F, 1), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradc_F, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradc_F, 2), size(F_test, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain inequality constraint must same number of rows as prefilter matrix.');
				end
				if isempty(ceq_F)
					test.TestSuite.assert(isempty(gradceq_F), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must be empty.');
				else
					test.TestSuite.assertEqual(size(gradceq_F, 3), size(ceq_F, 1), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must same number of planes as inequality constraints.');
					test.TestSuite.assertEqual(size(gradceq_F, 1), size(system.B, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must same number of rows as control matrix.');
					test.TestSuite.assertEqual(size(gradceq_F, 2), size(F_test, 2), 'control:outputfeedback:test', 'Gradient of nonlinear prefilter gain equality constraint must same number of rows as control matrix.');
				end
			end

			test.TestSuite.assertNoException('[r_parametric] = controller.gainpattern_parametric(systems{jj}, T);', 'control:outputfeedback:test', 'Parametric gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r_parametric, k_parametric] = controller.gainpattern_parametric(systems{jj}, T);', 'control:outputfeedback:test', 'Parametric gainpattern of outputfeedback class must not throw an exception.');
			test.TestSuite.assertNoException('[r_parametric, k_parametric, f_parametric] = controller.gainpattern_parametric(systems{jj}, T);', 'control:outputfeedback:test', 'Parametric gainpattern of outputfeedback class must not throw an exception.');
			[r_parametric, k_parametric, f_parametric] = controller.gainpattern_parametric(systems{jj}, T);
			test.TestSuite.assertEqual(size(r_parametric, 1), size(B, 2), 'control:outputfeedback:test', 'Parametric proportional gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(r_parametric, 2), size(C, 1), 'control:outputfeedback:test', 'Parametric proportional gain must have same number of columns as output matrix.');
			test.TestSuite.assertEqual(size(k_parametric, 1), size(B, 2), 'control:outputfeedback:test', 'Parametric derivative gain must have same number of rows as control matrix.');
			test.TestSuite.assertEqual(size(k_parametric, 2), size(C_dot, 1), 'control:outputfeedback:test', 'Parametric derivative gain must have same number of columns as output matrix.');
			test.TestSuite.assertEqual(size(f_parametric, 1), size(B, 2), 'control:outputfeedback:test', 'Parametric prefilter gain must have same number of rows as columns in B.');
			test.TestSuite.assertEqual(size(f_parametric, 2), size(f_fixed, 2), 'control:outputfeedback:test', 'Parametric prefilter gain pattern and prefilter gain must have same number of rows.');
			if islogical(r{1}) && ~isempty(r_parametric)
				if isa(r_parametric, 'realp')% TODO: also check genmat (how?)
					isfree = r_parametric.Free;
					test.TestSuite.assertEqual(~isfree, r{1}, 'control:outputfeedback:test', 'Parametric gain pattern must match numeric gain pattern.');
					if any(~isfree)
						test.TestSuite.assertEqual(double(r_parametric(~isfree)), r{2}(~isfree), 'control:outputfeedback:test', 'Parametric gain pattern values must match numeric gain pattern values.');
					end
				end
			end
			if islogical(k{1}) && ~isempty(k_parametric)
				if isa(k_parametric, 'realp')
					isfree = k_parametric.Free;
					test.TestSuite.assertEqual(~isfree, k{1}, 'control:outputfeedback:test', 'Parametric derivative gain pattern must match numeric derivative gain pattern.');
					if any(~isfree)
						test.TestSuite.assertEqual(double(k_parametric(~isfree)), k{2}(~isfree), 'control:outputfeedback:test', 'Parametric derivative gain pattern values must match numeric derivative gain pattern values.');
					end
				end
			end
			if islogical(f_fixed) && ~isempty(f_parametric)
				if isa(f_parametric, 'realp')
					isfree = f_parametric.Free;
					test.TestSuite.assertEqual(~isfree, f_fixed, 'control:outputfeedback:test', 'Parametric prefilter pattern must match numeric prefilter pattern.');
					if any(~isfree)
						test.TestSuite.assertEqual(double(f_parametric(~isfree)), f(~isfree), 'control:outputfeedback:test', 'Parametric prefilter pattern values must match numeric prefilter pattern values.');
					end
				end
			end

			test.TestSuite.assertNoException('F_scaled = controller.scaleprefilter(F_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj}, T);', 'control:outputfeedback:test', 'Prefilter scaling function must not throw an exception.');
			F_scaled = controller.scaleprefilter(F_test, T_x, T_u, T_y, T_y_dot, T_w, systems{jj}, T);
			test.TestSuite.assertEqual(size(F_scaled, 1), size(F_test, 1), 'control:outputfeedback:test', 'Prefilter matrix must not change size on scaling.');
			test.TestSuite.assertEqual(size(F_scaled, 2), size(F_test, 2), 'control:outputfeedback:test', 'Prefilter matrix must not change size on scaling.');
			F_scaled = controller.scaleprefilter(F_scaled, inv(T_x), inv(T_u), inv(T_y), inv(T_y_dot), inv(T_w), systems{jj}, T);
			test.TestSuite.assertEqual(F_scaled, F_test, 'control:outputfeedback:test', 'Scaling and unscaling must not change result.');

			test.TestSuite.assertNoException('[partitionR, partitionF] = controller.gainpartitioning({r{1}(:, :, 1), k{1}(:, :, 1)}, f, systems{jj}, T);', 'control:outputfeedback:test', 'Gainpartitioning of outputfeedback class must not throw an exception.');
			[partitionR, partitionF] = controller.gainpartitioning({r{1}(:, :, 1), k{1}(:, :, 1)}, f, systems{jj}, T);
			test.TestSuite.assert(isstruct(partitionR), 'control:outputfeedback:test', 'Partitioning of R must be a structure.');
			test.TestSuite.assert(isstruct(partitionF), 'control:outputfeedback:test', 'Partitioning of F must be a structure.');

			test.TestSuite.assertNoException('[system_controller, ~, ~, ~, ~, ~, ~, ~, needsstate] = controller.realization(R_test, F_test, systems{jj}, T);', 'control:outputfeedback:test', 'Returning controllerfor outputfeedback class must not throw an exception.');
			[system_controller, ~, ~, ~, ~, ~, ~, ~, needsstate] = controller.realization(R_test, F_test, systems{jj}, T);
			test.TestSuite.assert(islogical(needsstate), 'control:outputfeedback:test', 'State indicator must be of type ''logical''.');
			test.TestSuite.assert(isscalar(needsstate), 'control:outputfeedback:test', 'State indicator must be scalar.');
			if isa(system_controller, 'ss')
				[A_controller, B_controller, C_controller, D_controller] = ssdata(system_controller);
				if isempty(system_controller.e)
					E_controller = eye(size(A_controller, 1));
				else
					E_controller = system_controller.e;
				end
				system_controller = struct(...
					'E',		E_controller,...
					'A',		A_controller,...
					'B',		B_controller,...
					'C',		C_controller,...
					'C_dot',	zeros(size(C_controller, 1), size(A_controller, 1)),...
					'D',		D_controller,...
					'C_ref',	C_controller,...
					'D_ref',	D_controller...
				);
			elseif isa(system_controller, 'tf')
				[A_controller, B_controller, C_controller, D_controller] = ssdata(system_controller);
				E_controller = eye(size(A_controller, 1));
				system_controller = struct(...
					'E',		E_controller,...
					'A',		A_controller,...
					'B',		B_controller,...
					'C',		C_controller,...
					'C_dot',	zeros(size(C_controller, 1), size(A_controller, 1)),...
					'D',		D_controller,...
					'C_ref',	C_controller,...
					'D_ref',	D_controller...
				);
			end
			test.TestSuite.assertEqual(size(system_controller.A, 1), size(system_controller.A, 2), 'control:outputfeedback:test', 'System matrix of controller must be square.');
			test.TestSuite.assertEqual(size(system_controller.E, 1), size(system_controller.E, 2), 'control:outputfeedback:test', 'Descriptor matrix of controller must be square.');
			test.TestSuite.assertEqual(size(system_controller.A, 1), size(system_controller.E, 1), 'control:outputfeedback:test', 'System matrix of controller must have the same size as descriptor matrix.');
			test.TestSuite.assertEqual(size(system_controller.C, 2), size(system_controller.A, 1), 'control:outputfeedback:test', 'Output matrix of controller must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system_controller.C_dot, 2), size(system_controller.A, 1), 'control:outputfeedback:test', 'Derivative output matrix of controller must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system_controller.D, 1), size(system_controller.C, 1), 'control:outputfeedback:test', 'Throughput matrix of controller must have same number of rows as output matrix.');
			test.TestSuite.assertEqual(size(system_controller.D, 2), size(system_controller.B, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have same number of columns as control matrix.');
			test.TestSuite.assertEqual(size(system_controller.C_ref, 2), size(system_controller.A, 1), 'control:outputfeedback:test', 'Output matrix of references of controller must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system_controller.C_ref, 1), size(system_controller.D_ref, 1), 'control:outputfeedback:test', 'Throughput matrix of references of controller must have same number of rows as output matrix of references.');

			checkdim = ~isa(controller, 'control.design.outputfeedback.DynamicOutputFeedback');
			if checkdim
				test.TestSuite.assertEqual(size(basesystem.B, 2), size(system_controller.C, 1), 'control:outputfeedback:test', 'Output matrix of controller must have as much rows as B.');
				test.TestSuite.assertEqual(size(basesystem.B, 2), size(system_controller.D, 1), 'control:outputfeedback:test', 'Throughput matrix of controller must have as much rows as B.');
				if needsstate
					test.TestSuite.assertEqual(size(basesystem.A, 1) + size(basesystem.C, 1)*2, size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as three times the number of outputs of the system.');
					test.TestSuite.assertEqual(size(basesystem.A, 1) + size(basesystem.C, 1)*2, size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as three times the number of outputs of the system.');
				else
					test.TestSuite.assertEqual(size(basesystem.C, 1)*3, size(system_controller.B, 2), 'control:outputfeedback:test', 'Control matrix of controller must have as three times the number of outputs of the system.');
					test.TestSuite.assertEqual(size(basesystem.C, 1)*3, size(system_controller.D, 2), 'control:outputfeedback:test', 'Throughput matrix of controller must have as three times the number of outputs of the system.');
				end
			end
			test.TestSuite.assertNoException('controllerT.amend_parametric(systems{jj}, T);', 'control:outputfeedback:test', 'Amending discrete system with parametric outputfeedback class must not throw an exception.');
			system = controllerT.amend_parametric(systems{jj}, T);
			if isa(system, 'ss') || isa(system, 'genss')
				[A, B, C, D] = ssdata(system);
				if isa(system, 'genss') || isempty(system.e)
					E = eye(size(A, 1));
				else
					E = system.e;
				end
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	zeros(size(C, 1), size(A, 1)),...
					'D',		D,...
					'C_ref',	C,...
					'D_ref',	D...
				);
			elseif isa(system, 'tf')
				[A, B, C, D] = ssdata(system);
				E = eye(size(A, 1));
				system = struct(...
					'E',		E,...
					'A',		A,...
					'B',		B,...
					'C',		C,...
					'C_dot',	zeros(size(C, 1), size(A, 1)),...
					'D',		D,...
					'C_ref',	C,...
					'D_ref',	D...
				);
			end
			test.TestSuite.assertEqual(size(system.A, 1), size(system.A, 2), 'control:outputfeedback:test', 'System matrix of parametric augmented system must be square.');
			test.TestSuite.assertEqual(size(system.E, 1), size(system.E, 2), 'control:outputfeedback:test', 'Descriptor matrix of parametric augmented system must be square.');
			test.TestSuite.assertEqual(size(system.A, 1), size(system.E, 1), 'control:outputfeedback:test', 'System matrix of parametric augmented system must have the same size as descriptor matrix.');
			test.TestSuite.assertEqual(size(system.C, 2), size(system.A, 1), 'control:outputfeedback:test', 'Output matrix of parametric augmented system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_dot, 2), size(system.A, 1), 'control:outputfeedback:test', 'Derivative output matrix of parametric augmented system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.D, 1), size(system.C, 1), 'control:outputfeedback:test', 'Throughput matrix of parametric augmented system must have same number of rows as output matrix.');
			test.TestSuite.assertEqual(size(system.D, 2), size(system.B, 2), 'control:outputfeedback:test', 'Throughput matrix of parametric augmented system must have same number of columns as control matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 2), size(system.A, 1), 'control:outputfeedback:test', 'Output matrix of references of parametric augmented system must have same number of columns as system matrix.');
			test.TestSuite.assertEqual(size(system.C_ref, 1), size(system.D_ref, 1), 'control:outputfeedback:test', 'Throughput matrix of references of parametric augmented system must have same number of rows as output matrix of references.');
		end
	end
end