%% configure system
plotoptions = struct(...
	'reducemodel',			true,...
	'plot',					true,...
	'step',					false,...
	'step_obj',				false,...
	'step_koni',			false,...
	'markers',				false,...
	'plotkoni',				true,...
	'plotsolution',			true,...
	'plotsolution_obj',		false,...
	'colorkoni',			[0.75, 0.75, 0.75],...
	'colorsolution',		[0, 1, 0],...
	'colorsolution_obj',	[1, 1, 0]...
);
controller = control.design.outputfeedback.DynamicOutputFeedback(1);
m_k = 1000;
m_g = 50;
l = 10;
bridge1 = example.bridge(l, m_k, m_g);
m_g = 4000;
bridge2 = example.bridge(l, m_k, m_g);
m_g = 50;
l = 8;
bridge3 = example.bridge(l, m_k, m_g);
%m_g = 2000;
%bridge3 = example.bridge(l, m_k, m_g);
% sys1 = ss([0, 1;0,1], kron([1, 1], [0;1]), [1, 0], 0);
% sys2 = ss([-2, -1;4,-10], kron([1, 1], [0;1]), [1, 0], 0);
% sys3 = ss([-2, -1;4,-10], kron([1, 1], [0;1]), [1, 0], 0);
% sys1 = ss([
% 	bridge1.a,						zeros(size(bridge1.a, 1), 1);
% 	zeros(1, size(bridge1.a, 2)),	0
% ], [
% 	bridge1.b,						zeros(size(bridge1.b, 1), 1);
% 	zeros(1, size(bridge1.b, 2)),	1
% ], [
% 	bridge1.c,						zeros(size(bridge1.c, 1), 1);
% 	zeros(1, size(bridge1.c, 2)),	1
% ], zeros(size(bridge1.d, 1) + 1, size(bridge1.d, 2) + 1));
% sys2 = ss([
% 	bridge2.a,						zeros(size(bridge2.a, 1), 1);
% 	zeros(1, size(bridge2.a, 2)),	0
% ], [
% 	bridge2.b,						zeros(size(bridge2.b, 1), 1);
% 	zeros(1, size(bridge2.b, 2)),	1
% ], [
% 	bridge2.c,						zeros(size(bridge2.c, 1), 1);
% 	zeros(1, size(bridge2.c, 2)),	1
% ], zeros(size(bridge2.d, 1) + 1, size(bridge2.d, 2) + 1));
% sys3 = ss([
% 	bridge3.a,						zeros(size(bridge3.a, 1), 1);
% 	zeros(1, size(bridge3.a, 2)),	0
% ], [
% 	bridge3.b,						zeros(size(bridge3.b, 1), 1);
% 	zeros(1, size(bridge3.b, 2)),	1
% ], [
% 	bridge3.c,						zeros(size(bridge3.c, 1), 1);
% 	zeros(1, size(bridge3.c, 2)),	1
% ], zeros(size(bridge3.d, 1) + 1, size(bridge3.d, 2) + 1));
sys4 = example.bridge(8, m_k, 50);
sys5 = example.bridge(12, m_k, 50);
sys6 = example.bridge(8, m_k, 4000);
sys7 = example.bridge(12, m_k, 4000);
% systems = {
% 	sys1;
% 	sys2;
% 	%sys3
% 	%ss(sys4.A, sys4.B, sys4.C, sys4.D);
% 	%ss(sys5.A, sys5.B, sys5.C, sys5.D);
% 	%ss(sys6.A, sys6.B, sys6.C, sys6.D);
% 	%ss(sys7.A, sys7.B, sys7.C, sys7.D)
% };
system = [
	bridge1;
	bridge2;
	bridge3
];
% K_opt für sys1 und sys2 mit beschränkter Optimierung
% K_opt = [
% 	15636.739362,	-144.594574,	24065.322365,	-940.953105;
% 	-23371.789658,	15431.005867,	49.309405,		0.807167
% ];
% K_opt für sys1 bis sys7 mit beschränkter Optimierung
% K_opt = [
% 	1158324.76058061,	52649.8476715839,	19035120.5389798,	1004.71430743182;
% 	910.491000702407,	-1788.58136236280,	53028.5103450414,	1.52056181323372
% ];
mass = 50:10:4000;
len = 4:0.5:12;
if plotoptions.reducemodel
	mass = mass(1:floor(length(mass)/10):end);
	%len = len(1:floor(length(len)/2):end);
end
testsystems = struct('A', {}, 'B', {}, 'C', {},...
	...'C_dot', {},
	'D', {});
for ii = 1:size(mass, 2)
	for jj = 1:size(len, 2)
		temp = example.bridge(len(jj), m_k, mass(ii));
		%temp.C_dot = temp.C(1:3, :);
		testsystems(ii, jj) = temp;
	end
end
testsystems = reshape(testsystems, [], 1);

%% transform parameters
a = 0.6;
b = 0.5;
R = 50;
%% 
R_0 = [
	100,	10000,	-10000,	1;
	100,	100,	100,	1
];
use3systems = false;
if ~use3systems
	system(3:end) = [];
end
weight = {repmat([
	1,	1,	100
], size(system, 1), 1), [
	0;
	0;
	1
]};
weight{1}(size(system, 1) + 1:end, :) = [];
	weight{2}(size(system, 1) + 1:end, :) = [];
if use3systems
	weight{1}(3:end, :) = 0;
	weight{2}(1:2, :) = 0;
	weight{2}(3:end, :) = 1;
else
	weight{1}(3:end, :) = 0;
	weight{2}(1:2, :) = 1;
	weight{2}(3:end, :) = 0;
end
polearea = {repmat([
	control.design.gamma.area.Circlesquare(R),	control.design.gamma.area.Hyperbolasquare(a, b), control.design.gamma.area.Imag(1, a)
], size(system, 1), 1),	repmat([
	control.design.gamma.area.Ellipsesquare(30*a, 20*b, -20 + 0i)
], size(system, 1), 1)};
%polearea = polearea{1};
%%
R_fixed = [];
solver = optimization.solver.Optimizer.IPOPT;
options = optimization.options.OptionFactory.instance.options(solver,...
	'ProblemType',					optimization.options.ProblemType.CONSTRAINED,...
	'Retries',						1,...
	'Algorithm',					solver.defaultalgorithm,...
	'FunctionTolerance',			1E-8,...
	'StepTolerance',				1E-10,...
	'ConstraintTolerance',			1E-7,...
	'MaxFunctionEvaluations',		25E3,...
	'MaxIterations',				25E3,...
	'MaxSQPIter',					25E3,...
	'SpecifyObjectiveGradient',		true,...
	'SpecifyObjectiveHessian',		false,...
	'SpecifyConstraintGradient',	true,...
	'SpecifyConstraintHessian',		false,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed'...
);
objectiveoptions = struct(...
	'usecompiled',			true,...% indicator, if compiled functions should be used
	'eigenvaluederivative',	GammaEigenvalueDerivativeType.VANDERAA,...
	'type',					GammaJType.ZERO,...% type of pole area weighting in objective function
	'weight',				1,...
	'allowvarorder',		false,...%allow variable state number for different multi models
	'allownegativeweight',	true...%allow variable state number for different multi models
);
[R_opt, J_opt, info] = control.design.gamma.gammasyn(system, polearea{1}, weight, R_fixed, R_0, options, objectiveoptions)

objectiveoptions_obj = struct(...
	'usecompiled',			true,...% indicator, if compiled functions should be used
	'eigenvaluederivative',	GammaEigenvalueDerivativeType.VANDERAA,...
	'type',					GammaJType.NORMGAIN,...% type of pole area weighting in objective function
	'weight',				1/1000,...
	'allowvarorder',		false,...%allow variable state number for different multi models
	'allownegativeweight',	true,...%allow variable state number for different multi models
	'strategy',				GammaSolutionStrategy.FEASIBILITYITERATION,...
	'objective',			struct(...
		'normgain',				struct(...
			'R',					1./abs(R_0).*[
				10,	10,	10,	1;
				1,	1,	1,	1
			]/(100)...
		)...
	)...
);
[R_opt_obj, J_opt_obj, info_obj] = control.design.gamma.gammasyn(system, polearea, weight, R_fixed, R_0, options, objectiveoptions_obj)
%return;
R_koni = [
	144.675,	10294.8,	-10032.5,	-1000.68;
	7.244,		25.353,		-306.174,	0.625
];
system(3:end) = [];
%% plot poles of systems
leftmost = Inf;
rightmost = -Inf;
maximag = -Inf;
legendentries = cell(8, 1);
if plotoptions.plot
	exportfigure = figure;
	hold('all');
	if exist('a', 'var') && exist('b', 'var')
		phi = atan(b/a);
		s = linspace(-phi, phi, 200);
		plot(-R*cos(s), R*sin(s), 'k');
		wp = unique([linspace(-sin(phi)*R, sin(phi)*R, 400), linspace(-sin(phi/50)*R, sin(phi/50)*R, 50), 0]);
		legendentries{1, 1} = plot(-a/b*sqrt(wp.^2 + b^2), wp, 'k');
		plot([0, 0], [-R, R], 'r');
		
		if plotoptions.plotsolution_obj
			theta = linspace(0, 2*pi, 200);
			legendentries{2, 1} = plot(30*a*cos(theta) - 20, 20*b*sin(theta), 'b');
		end
	else
		wp = linspace(0, 2*pi, 500);
		plot(R*cos(wp), R*sin(wp), 'k');
	end

	% plot systems Konigorski
	if plotoptions.plotkoni
		eigenvalues_test_koni = NaN(size(testsystems(1).A, 1), length(testsystems));
		for ii = 1:length(testsystems)
			A = testsystems(ii).A;
			B = testsystems(ii).B;
			C = testsystems(ii).C;
			if isfield(testsystems, 'C_dot')
				if iscell(R_opt)
					E = eye(size(A, 1)) - B*R_koni{2}*testsystems(ii).C_dot;
					Dcl = eig(A - B*R_koni{1}*C, E);
				else
					Dcl = eig(A - B*R_koni{1}*C);
				end
			else
				if iscell(R_opt)
					Dcl = eig(A - B*R_koni{1}*C);
				else
					Dcl = eig(A - B*R_koni*C);
				end
			end

			%Dol = eig(A);
			eigenvalues_test_koni(:, ii) = Dcl;
		end
		legendentries{3, 1} = scatter(real(eigenvalues_test_koni(:))', imag(eigenvalues_test_koni(:))', [], plotoptions.colorkoni, '+');
		leftmost = min([leftmost; real(eigenvalues_test_koni(:))]);
		rightmost = max([rightmost; real(eigenvalues_test_koni(:))]);
		maximag = max([maximag; imag(eigenvalues_test_koni(:))]);
		if ~exist('systems', 'var')
			systems = system;
			delsystems = true;
		else
			delsystems = false;
		end
		eigenvalues_koni = NaN(size(systems(1).A, 1), length(systems));
		for ii = 1:length(systems)
			if iscell(systems)
				[A, B, C, ~] = ssdata(systems{ii});
			elseif isstruct(systems)
				A = systems(ii).A;
				B = systems(ii).B;
				C = systems(ii).C;
			else
				error();
			end
			if iscell(R_opt)
				if isstruct(systems) && isfield(systems(ii), 'C_dot')
					E = eye(size(A, 1)) - B*R_koni{2}*systems(ii).C_dot;
					Dcl = eig(A - B*R_koni{1}*C, E);
				elseif iscell(systems)
					E = eye(size(A, 1)) - B*R_koni{2}*C(1:3, :);
					Dcl = eig(A - B*R_koni{1}*C, E);
				else
					Dcl = eig(A - B*R_koni{1}*C);
				end
			else
				Dcl = eig(A - B*R_koni*C);
			end
			eigenvalues_koni(:, ii) = Dcl;
		end
		legendentries{4, 1} = scatter(real(eigenvalues_koni(:)), imag(eigenvalues_koni(:)), [], 'v', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', plotoptions.colorkoni);
		if plotoptions.markers
			scatter(real(eigenvalues_koni(:)), imag(eigenvalues_koni(:)), 7, '.', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0, 0, 0]);
		end
		leftmost = min([leftmost; real(eigenvalues_koni(:))]);
		rightmost = max([rightmost; real(eigenvalues_koni(:))]);
		maximag = max([maximag; imag(eigenvalues_koni(:))]);
	end
	if plotoptions.plotsolution
		% plot systems
		eigenvalues_test = NaN(size(testsystems(1).A, 1), length(testsystems));
		for ii = 1:length(testsystems)
			A = testsystems(ii).A;
			B = testsystems(ii).B;
			C = testsystems(ii).C;
			if isfield(testsystems, 'C_dot')
				if iscell(R_opt)
					E = eye(size(A, 1)) - B*R_opt{2}*testsystems(ii).C_dot;
					Dcl = eig(A - B*R_opt{1}*C, E);
				else
					Dcl = eig(A - B*R_opt{1}*C);
				end
			else
				if iscell(R_opt)
					Dcl = eig(A - B*R_opt{1}*C);
				else
					Dcl = eig(A - B*R_opt*C);
				end
			end

			%Dol = eig(A);

			eigenvalues_test(:, ii) = Dcl;
		end
		legendentries{5, 1} = scatter(real(eigenvalues_test(:))', imag(eigenvalues_test(:))', [], plotoptions.colorsolution, '*');
		leftmost = min([leftmost; real(eigenvalues_test(:))]);
		rightmost = max([rightmost; real(eigenvalues_test(:))]);
		maximag = max([maximag; imag(eigenvalues_test(:))]);
		if ~exist('systems', 'var')
			systems = system;
			delsystems = true;
		else
			%delsystems = false;
		end
		eigenvalues = NaN(size(systems(1).A, 1), length(systems));
		for ii = 1:length(systems)
			if iscell(systems)
				[A, B, C, ~] = ssdata(systems{ii});
			elseif isstruct(systems)
				A = systems(ii).A;
				B = systems(ii).B;
				C = systems(ii).C;
			else
				error();
			end
			if iscell(R_opt)
				if isstruct(systems) && isfield(systems(ii), 'C_dot')
					E = eye(size(A, 1)) - B*R_opt{2}*systems(ii).C_dot;
					Dcl = eig(A - B*R_opt{1}*C, E);
				elseif iscell(systems)
					E = eye(size(A, 1)) - B*R_opt{2}*C(1:3, :);
					Dcl = eig(A - B*R_opt{1}*C, E);
				else
					Dcl = eig(A - B*R_opt{1}*C);
				end
			else
				Dcl = eig(A - B*R_opt*C);
			end
			eigenvalues(:, ii) = Dcl;
		end
		legendentries{6, 1} = scatter(real(eigenvalues(:)), imag(eigenvalues(:)), [], 'd', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', plotoptions.colorsolution);
		if plotoptions.markers
			scatter(real(eigenvalues(:)), imag(eigenvalues(:)), 7, '.', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0, 0, 0]);
		end
		leftmost = min([leftmost; real(eigenvalues(:))]);
		rightmost = max([rightmost; real(eigenvalues(:))]);
		maximag = max([maximag; imag(eigenvalues(:))]);
		if delsystems
			clear systems;
		end
		clear delsystems
	end
	if plotoptions.plotsolution_obj
		% plot systems
		eigenvalues_test_obj = NaN(size(testsystems(1).A, 1), length(testsystems));
		for ii = 1:length(testsystems)
			A = testsystems(ii).A;
			B = testsystems(ii).B;
			C = testsystems(ii).C;
			if isfield(testsystems, 'C_dot')
				if iscell(R_opt_obj)
					E = eye(size(A, 1)) - B*R_opt_obj{2}*testsystems(ii).C_dot;
					Dcl = eig(A - B*R_opt_obj{1}*C, E);
				else
					Dcl = eig(A - B*R_opt_obj{1}*C);
				end
			else
				if iscell(R_opt_obj)
					Dcl = eig(A - B*R_opt_obj{1}*C);
				else
					Dcl = eig(A - B*R_opt_obj*C);
				end
			end

			%Dol = eig(A);
			eigenvalues_test_obj(:, ii) = Dcl;
		end
		legendentries{7, 1} = scatter(real(eigenvalues_test_obj(:))', imag(eigenvalues_test_obj(:))', [], plotoptions.colorsolution_obj, '*');
		leftmost = min([leftmost; real(eigenvalues_test_obj(:))]);
		rightmost = max([rightmost; real(eigenvalues_test_obj(:))]);
		maximag = max([maximag; imag(eigenvalues_test_obj(:))]);
		if ~exist('systems', 'var')
			systems = system;
			delsystems = true;
		else
			%delsystems = false;
		end
		eigenvalues_obj = NaN(size(systems(1).A, 1), length(systems));
		for ii = 1:length(systems)
			if iscell(systems)
				[A, B, C, ~] = ssdata(systems{ii});
			elseif isstruct(systems)
				A = systems(ii).A;
				B = systems(ii).B;
				C = systems(ii).C;
			else
				error();
			end
			if iscell(R_opt_obj)
				if isstruct(systems) && isfield(systems(ii), 'C_dot')
					E = eye(size(A, 1)) - B*R_opt_obj{2}*systems(ii).C_dot;
					Dcl = eig(A - B*R_opt_obj{1}*C, E);
				elseif iscell(systems)
					E = eye(size(A, 1)) - B*R_opt_obj{2}*C(1:3, :);
					Dcl = eig(A - B*R_opt_obj{1}*C, E);
				else
					Dcl = eig(A - B*R_opt_obj{1}*C);
				end
			else
				Dcl = eig(A - B*R_opt_obj*C);
			end
			eigenvalues_obj(:, ii) = Dcl;
		end
		legendentries{8, 1} = scatter(real(eigenvalues_obj(:)), imag(eigenvalues_obj(:)), [], 'd', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', plotoptions.colorsolution_obj);
		if plotoptions.markers
			scatter(real(eigenvalues_obj(:)), imag(eigenvalues_obj(:)), 7, '.', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0, 0, 0]);
		end
		leftmost = min([leftmost; real(eigenvalues_obj(:))]);
		rightmost = max([rightmost; real(eigenvalues_obj(:))]);
		maximag = max([maximag; imag(eigenvalues_obj(:))]);
		if delsystems
			clear systems;
		end
		clear delsystems
	end
	hold('off');
	grid('on');
	xlim([leftmost, rightmost]);
	ylim([-maximag, maximag]);
	%legendentries(3:2:end, :) = [];
	isemptylegend = cellfun(@isempty, legendentries, 'UniformOutput', true);
	entries = cellfun(@(x) x(iftern(isempty(x), [], 1)), legendentries, 'UniformOutput', false);
	legends = {
		'Polgebiet';
		'Polgebiet optional';
		'Lösung \cite{FOELLINGER1994}';
		'Entwurf \cite{FOELLINGER1994}';
		'Lösung $\mat{R}_1$';
		'Entwurf $\mat{R}_1$';
		'Lösung $\mat{R}_2$';
		'Entwurf $\mat{R}_2$'
	};
	legends(isemptylegend, :) = [];
	entries(isemptylegend, :) = [];
	legend([entries{:}], legends);
	filename = 'VerladeEigenwerte';
	polearea_strictstring = polearea{1}.getstring();
	polearea_loosestring = polearea{2}.getstring();
	for ii = 1:size(polearea_strictstring, 1)
		polearea_strict_string{ii, 1} = strjoin(polearea_strictstring(ii, :), ', ');
	end
	for ii = 1:size(polearea_loosestring, 1)
		polearea_loose_string{ii, 1} = strjoin(polearea_loosestring(ii, :), ', ');
	end
	comment = [
		'Polgebiet:', newline(),...
		sprintf('\t'), strjoin(polearea_strict_string, [newline(), sprintf('\t')]), newline,...
		'Polgebiet optional:', newline(),...
		sprintf('\t'), strjoin(polearea_loose_string, [newline(), sprintf('\t')])
	];
	if exist('struct2text', 'file')
		comment = [
			comment, newline(),...
			'Optionen:', newline(),...
			sprintf('\t'), strjoin(strsplit(struct2text(objectiveoptions), newline()), [newline(), sprintf('\t')]), newline(),...
			'Optionen Zielfunktion', newline(),...
			sprintf('\t'), strjoin(strsplit(struct2text(objectiveoptions_obj), newline()), [newline(), sprintf('\t')])
		];
	end
	TikzSettings(...
		'tikzFileComment',	comment,...
		'grid',				'on',...
		'clean',			false,...
		'parseStrings',		false,...
		'externalData',		true,...
		'dataPath',			fullfile('.', 'Bilder', 'Regelung', [filename, '_Daten']),...
		'relativeDataPath',	[filename, '_Daten'],...
		'currfile',			true,...
		'addplotplus',		false,...
		'postprocess',		{
			%@latex.matlab2tikz.filter.emptyticks2nomajorticks;
			@(x) regexprep(x, 'at\s*=\s*\{\(0\\figurewidth,\s*\d+\.\d+\\figureheight\)\}', 'name=plot1');
			@(x) regexprep(x, '(?<before>width=\\figurewidth,\s*height=\d+\.\d+\\figureheight,\s*)(?<after>scale only axis)', '$<before>at=(plot1.below south west),\n\t\tanchor=above north west,\n\t\t$<after>')
		},...
		'extraAxisOptions',	{
			'scaled y ticks = false';
			'y tick label style={/pgf/number format/fixed}'
		}...
	).export(fullfile('.', 'Bilder', 'Regelung', [filename, '.tikz']), true, exportfigure);
end

%% plot step responses
if plotoptions.step
	t = (0:0.01:16).';
	w = zeros(length(t), 4);
	w(:, 1) = 10;

	plot_opt = figure;
	subplot(3, 1, 1);
	if plotoptions.step_obj
		plot_obj = figure;
		subplot(3, 1, 1);
	end
	if plotoptions.step_koni
		plot_koni = figure;
		subplot(3, 1, 1);
	end
	for ii = 1:length(testsystems)
		A = testsystems(ii).A;
		B = testsystems(ii).B;
		C = testsystems(ii).C;

		Acl = A - B*R_opt*C;
		Bcl = B*R_opt;
		Ccl = [
			[1, 0, 0, 0, 0; 0, 0, 1, 0, 0];
			-R_opt*C
		];
		Dcl = [
			zeros(2, 4);
			R_opt
		];

		Gcl = ss(Acl, Bcl, Ccl, Dcl);
		y = lsim(Gcl, w, t);

		figure(plot_opt);
		subplot(3, 1, 1);
		plot(t, y(:, 1));
		hold('all');
		subplot(3, 1, 2);
		plot(t, y(:, 2));
		hold('all');
		subplot(3, 1, 3);
		plot(t, y(:, 3));
		hold('all');
		
		if plotoptions.step_obj
			Acl = A - B*R_opt_obj*C;
			Bcl = B*R_opt_obj;
			Ccl = [
				[1, 0, 0, 0, 0; 0, 0, 1, 0, 0];
				-R_opt_obj*C
			];
			Dcl = [
				zeros(2, 4);
				R_opt_obj
			];

			Gcl = ss(Acl, Bcl, Ccl, Dcl);
			y = lsim(Gcl, w, t);
			
			figure(plot_obj);
			subplot(3, 1, 1);
			plot(t, y(:, 1));
			hold('all');
			subplot(3, 1, 2);
			plot(t, y(:, 2));
			hold('all');
			subplot(3, 1, 3);
			plot(t, y(:, 3));
			hold('all');
		end
		if plotoptions.step_koni
			Acl = A - B*R_koni*C;
			Bcl = B*R_koni;
			Ccl = [
				[1, 0, 0, 0, 0; 0, 0, 1, 0, 0];
				-R_koni*C
			];
			Dcl = [
				zeros(2, 4);
				R_koni
			];

			Gcl = ss(Acl, Bcl, Ccl, Dcl);
			y = lsim(Gcl, w, t);
			
			figure(plot_koni);
			subplot(3, 1, 1);
			plot(t, y(:, 1));
			hold('all');
			subplot(3, 1, 2);
			plot(t, y(:, 2));
			hold('all');
			subplot(3, 1, 3);
			plot(t, y(:, 3));
			hold('all');
		end
	end
	figure(plot_opt);
	subplot(3, 1, 3);
	xlabel('t');
	ylabel('u');
	grid('on');
	hold('off');
	title('Lösung');
	subplot(3, 1, 1);
	xlabel('t');
	ylabel('s');
	grid('on');
	hold('off');
	subplot(3, 1, 2);
	xlabel('t');
	ylabel('\phi');
	grid('on');
	hold('off');
	if plotoptions.step_obj
		figure(plot_obj);
		subplot(3, 1, 3);
		xlabel('t');
		ylabel('u');
		grid('on');
		hold('off');
		title('Lösung Zielfunktion');
		subplot(3, 1, 1);
		xlabel('t');
		ylabel('s');
		grid('on');
		hold('off');
		subplot(3, 1, 2);
		xlabel('t');
		ylabel('\phi');
		grid('on');
		hold('off');
	end
	if plotoptions.step_koni
		figure(plot_koni);
		subplot(3, 1, 3);
		xlabel('t');
		ylabel('u');
		grid('on');
		hold('off');
		title('Lösung \cite{FOELLINGER1994}');
		subplot(3, 1, 1);
		xlabel('t');
		ylabel('s');
		grid('on');
		hold('off');
		subplot(3, 1, 2);
		xlabel('t');
		ylabel('\phi');
		grid('on');
		hold('off');
	end
end