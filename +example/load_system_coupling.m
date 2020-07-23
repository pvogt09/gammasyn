function [system, system_properties] = load_system(sys_string)
	%% init
	E_cell =		{};
	A_cell =		{}; %#ok<NASGU>
	B_cell =		{}; %#ok<NASGU>
	C_cell =		{};
	C_dot_cell =	{};
	D_cell =		{};
	C_ref_cell =	{}; %#ok<NASGU>
	D_ref_cell =	{};

	number_couplingconditions = [];
	number_models = []; %#ok<NASGU>
	R_0 = []; %#ok<NASGU>
	K_0 = [];
	F_0 = []; %#ok<NASGU>

	%% system database
	switch sys_string
		case {'threetank', 'threetank_exact', 'threetank_feedthrough'}
			A_handle = @(q1,q2,q3,x10,x20,x30,A1,A2,A3) sqrt(981/2)*[
				-q1/A1/(x10-x20),	q1/A1/(x10-x20),						0;
				q1/A2/(x10-x20),	-1/A2*(q1/(x10-x20) + q2/(x20-x30)),	q2/A2/(x20-x30);
				0,					q2/A3/(x20-x30),						-1/A3*(q2/(x20-x30)+q3/x30)
			];
			B_handle = @(A1,A3)[1/A1 0; 0 0; 0 1/A3];
			C_ref = [1 0 0; 1 -1 0];
			D_ref = zeros(2);

			if strcmp(sys_string, 'threetank_feedthrough')
				D_ref(2) = 1;
			end

			% params = [q1  q2  q3  x10  x20 x30 A1  A2  A3   ]
			if strcmp(sys_string, 'threetank_exact')
				params =   [% exact design
					4	4	4	13   10  9   100 100 100;
					5	4	4	13   10  9   100 100 100;
					6	4	4	13   10  9   100 100 100;
					7	4	4	13   11  10  100 100 100;
					8	4	5	13   12  11  100 100 100;
					9	4	6	14   13  12  100 100 100;
					10	4	7	15   14  13  100 100 100;
					4	4	8	16   15  14  150 150 110;
					4	4	8	17   16  15  150 150 120;
					4	4	8	18   17  16  150 150 130;
					4	4	8	19   18  17  150 150 140;
					% exact design
				];
% 				params =   [% exact design more realistic parameters
% 						0.8	0.8	0.8	13   10  9   140 140 140;
% 						5	4	4	13   10  9   100 100 100;
% 						6	4	4	13   10  9   100 100 100;
% 						7	4	4	13   11  10  100 100 100;
% 						8	4	5	13   12  11  100 100 100;
% 						9	4	6	14   13  12  100 100 100;
% 						10	4	7	15   14  13  100 100 100;
% 						4	4	8	16   15  14  150 150 110;
% 						4	4	8	17   16  15  150 150 120;
% 						4	4	8	18   17  16  150 150 130;
% 						4	4	8	19   18  17  150 150 140;
% 						% exact design
% 				];
% 				params(:, 1:3) = params(:, 1:3)./5;
% 				params(:, 7:9) = params(:, 7:9)./2;
			else
				params = [
					% approximate design
					4	4	4	13   10  9   100 100 100;
					5	5	4	13   10  9   100 90  100;
					6	6	4	13   10  9   100 90  100;
					7	7	4	13   11  9   100 90  100;
					8	8	5	13   12  9   100 90  100;
					9	9	6	14   13  10  100 90  100;
					10	9	7	15   14  11  100 80  100;
					4	9	8	16   15  12  150 70  110;
					4	9	8	17   16  13  150 60  120;
					4	9	8	18   17  14  150 70  130;
					4	9	8	19   18  15  150 80  140;
					% approximate design
				];
			end
			R_0 = [
				0 5 -1000;
				10 -30 10
			]+randn(2,3)*10*0;
			F_0 = [0, 25; 1, 1];
			number_couplingconditions = 1;

			number_models = size(params,1);
			A_cell =		cell(number_models, 1);
			B_cell =		cell(number_models, 1);
			C_ref_cell =	cell(number_models, 1);
			D_ref_cell =	cell(number_models, 1);

			for ii = 1:number_models
				A_cell{ii, 1} =		A_handle(params(ii,1),params(ii,2),params(ii,3),params(ii,4),params(ii,5),params(ii,6),params(ii,7),params(ii,8),params(ii,9));
				B_cell{ii, 1} =		B_handle(params(ii,7),params(ii,9));
				C_ref_cell{ii, 1} =	C_ref;
				D_ref_cell{ii, 1} =	D_ref;
			end
		case 'double_PT2'
			A_handle = @(x)[
				0     1     0     0;
				-x(1) -x(2)  0     0;
				0     0     0     1;
				0     0    -x(3) -x(4)
			];
			B_handle = @(x)[0 0; x(1) 0; 0 0; 0 x(2)];
			C_ref  = [1 0 0 0; 1 0 -1 0];
			number_couplingconditions = 1;
			params = [
				1.00  2   1   1   1       1;
				1.01  2   1   1   1       1;
				1.05  2   1   1   1       1;
				1.15  2   1   1   1       1;
				1.30  2   1   1   1       1;
				1.50  2   1   1   1       1;
				1.30  2   1   1   1.01    1;
				1     2   1   1   1.10    1;
				1     2   1   1   1.15    1;
				1     2   1   1   1.20    1;
				1     2   1   1   1.30    1;
				1.30  2   1.2 1   1.40    1;
				1     2   1   1   1.50    1;
				1     2   1   1   1.60    1
			];

			R_0 = [
				35.5980 16.9527 17.3512 23.1759
				4.8300 16.4747 48.1356 23.6950
			];% + randn(2,4)*10;

			F_0 = [
				1, 25;
				1, 1
			];

			number_models = size(params, 1);
			A_cell =		cell(number_models, 1);
			B_cell =		cell(number_models, 1);
			C_ref_cell =	cell(number_models, 1);
			for ii = 1:number_models
				A_cell{ii, 1} = A_handle(params(ii,1:4));
				B_cell{ii,1} =	B_handle(params(ii,5:6));
				C_ref_cell{ii} =	C_ref;
			end
		case 'diagonal_system'
			A_handle = @(m,c)diag([-m -c]);
			B_handle = @()eye(2);
			C_ref = [1 0; 1 -1];
			number_couplingconditions = 1;
			params = [1 2];
			R_0 = ones(2);
			F_0 = ones(2);

			number_models = size(params, 1);
			A_cell =		cell(number_models, 1);
			B_cell =		cell(number_models, 1);
			C_ref_cell =	cell(number_models, 1);
			for ii = 1:number_models
				A_cell{ii, 1} =		A_handle(params(:,1),params(:,2));
				B_cell{ii, 1} =		B_handle();
				C_ref_cell{ii, 1} =	C_ref;
			end
		case 'diagonal_system2'
			A_handle = @(m,c)diag([-m -c -m*c -m-c]);
			B_handle = @()rand(4);
			C_ref = [1 0 0 0; 1 -1 0 0; 1 0 -1 0; 1 0 0 -1];
			number_couplingconditions = 3;
			params = [1.01 2; 1.1 2];
			R_0 = ones(4);
			F_0 = ones(4);

			number_models = size(params, 1);
			A_cell =		cell(number_models, 1);
			B_cell =		cell(number_models, 1);
			C_ref_cell =	cell(number_models, 1);
			for ii = 1:number_models
				A_cell{ii, 1} =		A_handle(params(ii, 1), params(ii, 2));
				B_cell{ii, 1} =		B_handle();
				C_ref_cell{ii, 1} =	C_ref;
			end
		case 'descriptor_system' % not possible with numeric design because of too many multiple eigenvalues
			A_handle = @(x)[zeros(3), diag([x(1) x(2) 1/x(3)]); [0 0 -1; 1 -1 0; -1 0 0], [1 1 0; 0 0 0; 0 0 -1]];
			B_handle = [0 0 0 0 0 1].';
			E = blkdiag(eye(3),zeros(3));
			C_ref = [1 0 0 0 0 0];
			params = [
				1	4	6
				2	8	5
				3	12	4
				4	16	3
				5	20	2
				6	24	1
			];
			R_0 = [-5 50 100 0 -30 -50]+randn(1, 6)*10;
			F_0 = 500*randn(1, 1);
			number_models = size(params, 1);
			E_cell =		cell(number_models, 1);
			A_cell =		cell(number_models, 1);
			B_cell =		cell(number_models, 1);
			C_ref_cell =	cell(number_models, 1);
			for ii = 1:number_models
				A_cell{ii, 1} = A_handle(params(ii, :));
				B_cell{ii, 1} = B_handle;
				C_ref_cell{ii, 1} = C_ref;
				E_cell{ii, 1} = E;
			end
		case 'descriptor_system2' % not possible with numeric design because of too many multiple eigenvalues
			A_handle = @(x)[zeros(3), diag([x(1) x(2) 1/x(3)]); [0 0 -1; 1 -x(4) 0; -1 0 0], [x(5) 1 0; 0 0 0; 0 0 -1]];
			B_handle = [0 0 0 0 0 1].';
			E = blkdiag(eye(3),zeros(3));
			C_ref = [1 0 0 0 0 0];
			params = [
				1	4	6	10	4
				2	8	5	9	3
				3	12	4	8	2
				4	16	3	7	1
				5	20	2	6	0
				6	24	1	5	-1
			];
			R_0 = [-5 50 100 0 -30 -50]+randn(1, 6)*10;
			F_0 = 500*randn(1, 1);
			number_models = size(params, 1);
			E_cell =		cell(number_models, 1);
			A_cell =		cell(number_models, 1);
			B_cell =		cell(number_models, 1);
			C_ref_cell =	cell(number_models, 1);
			for ii = 1:number_models
				A_cell{ii, 1} = A_handle(params(ii, :));
				B_cell{ii, 1} = B_handle;
				C_ref_cell{ii, 1} = C_ref;
				E_cell{ii, 1} = E;
			end
		otherwise
			error('Requested system does not exist.')
	end

	%% output_structure
	number_models = size(A_cell, 1);
	number_states = size(A_cell{1},1);
	number_controls = size(B_cell{1},2);


	if isempty(E_cell)
		E_cell = cell(number_models, 1);
		E_cell(:) = {eye(number_states)};
	end
	if isempty(A_cell)
		error('Systems must have A matrices.');
	end
	if isempty(B_cell)
		error('Systems must have B matrices.');
	end
	if isempty(C_cell) || isempty(D_cell)
		if ~isempty(C_cell) && isempty(D_cell)
			number_measurements = size(C_cell{1}, 1);
			D_cell = cell(number_models, 1);
			D_cell(:) = {zeros(number_measurements, number_controls)};
		elseif isempty(C_cell) && ~isempty(D_cell)
			number_measurements = size(D_cell{1}, 1);
			C_cell = cell(number_models, 1);
			C_cell(:) = {zeros(number_measurements, number_states)};
		elseif isempty(C_cell) && isempty(D_cell)
			number_measurements = number_states;
			C_cell = cell(number_models, 1);
			D_cell = cell(number_models, 1);
			C_cell(:) = {eye(number_measurements, number_states)};
			D_cell(:) = {zeros(number_measurements, number_controls)};
		end
	end
	if isempty(C_dot_cell)
		C_dot_cell = cell(number_models, 1);
		C_dot_cell(:) = {zeros(0, number_states)};
		number_measurements_xdot = 0;
	else
		number_measurements_xdot = size(C_dot_cell{1}, 1);
	end
	if isempty(C_ref_cell) || isempty(D_ref_cell)
		if ~isempty(C_ref_cell) && isempty(D_ref_cell)
			number_references = size(C_ref_cell{1}, 1);
			D_ref_cell = cell(number_models, 1);
			D_ref_cell(:) = {zeros(number_references, number_controls)};
		elseif isempty(C_ref_cell) && ~isempty(D_ref_cell)
			number_references = size(D_ref_cell{1}, 1);
			C_ref_cell = cell(number_models, 1);
			C_ref_cell(:) = {zeros(number_references, number_states)};
		elseif isempty(C_ref_cell) && isempty(D_ref_cell)
			number_references = 0;
			C_ref_cell = cell(number_models, 1);
			D_ref_cell = cell(number_models, 1);
			C_ref_cell(:) = {zeros(number_references, number_states)};
			D_ref_cell(:) = {zeros(number_references, number_controls)};
		end
	else
		if size(C_ref_cell{1}, 1) ~= size(D_ref_cell{1}, 1)
			error('Sizes of C_ref and D_ref do not match.')
		else
			number_references = size(C_ref_cell{1}, 1);
		end
	end

	if isempty(number_couplingconditions)
		number_couplingconditions = 0;
	end
	if isempty(R_0)
		R_0 = zeros(number_controls, number_measurements);
	end
	if isempty(K_0)
		K_0 = zeros(number_controls, number_measurements_xdot);
	end
	if isempty(F_0)
		F_0 = zeros(number_controls, number_references);
	end
	RKF_0 = {R_0, K_0, F_0};

	system = struct(...
		'E',							E_cell,...
		'A',							A_cell,...
		'B',							B_cell,...
		'C',							C_cell,...
		'C_dot',						C_dot_cell,...
		'D',							D_cell,...
		'C_ref',						C_ref_cell,...
		'D_ref',						D_ref_cell...
	);

	system_properties = struct(...
		'number_states',				number_states,...
		'number_controls',				number_controls,...
		'number_references',			number_references,...
		'number_couplingconditions',	number_couplingconditions,...
		'number_models',				number_models,...
		'RKF_0',						{RKF_0},...
		'name',							sys_string...
	);
end