function [txt] = cursordata(datatip, event)
	%CURSORDATA set tooltip data
	%	Input:
	%		datatip:	handle to current datatip
	%		even:		click event on datatip
	%	Output:
	%		txt:		text to display in the datatip
	set(datatip, 'FontName', 'FixedWidth');
	set(datatip, 'FontSize', 5);
	l = get(event, 'Target');
	istestsystem = true;
	if isa(l, 'matlab.graphics.chart.primitive.Scatter')
		idx = get(event, 'DataIndex');
	else
		istestsystem = false;
		idx = get(l, 'UserData');
	end
	ax = get(l, 'Parent');
	systems = get(ax, 'UserData');
	if isstruct(systems) || iscell(systems)
		if iscell(systems)
			if istestsystem && isstruct(systems{1}) && isfield(systems{1}, 'systems')
				testsystems = systems{1}.systems;
			elseif ~istestsystem && isstruct(systems{2}) && isfield(systems{2}, 'systems')
				testsystems = systems{2}.systems;
			else
				testsystems = systems.systems;
			end
	else
		testsystems = systems;
	end
	pos = get(event, 'Position');
	if length(testsystems) >= idx
		system = testsystems(idx);
		E = system.E;
		A = system.A;
		B = system.B;
		C = system.C;
		D = system.D;
		Etxt = num2str(E);
		Atxt = num2str(A);
		Btxt = num2str(B);
		Ctxt = num2str(C);
		Dtxt = num2str(D);
		linesdyn = cell(size(A, 1), 1);
		parfor ii = 1:size(A, 1)
			if ii == ceil(size(A, 1)/2)
				linesdyn{ii, 1} = ['[', Etxt(ii, :), '] x'' = [', Atxt(ii, :), '] x + [', Btxt(ii, :), '] u'];
			else
				linesdyn{ii, 1} = ['[', Etxt(ii, :), ']      [', Atxt(ii, :), ']     [', Btxt(ii, :), ']  '];
			end
		end
		linesout = cell(size(C, 1), 1);
		parfor ii = 1:size(C, 1)
			if ii == ceil(size(C, 1)/2)
				linesout{ii, 1} = ['y = [', Ctxt(ii, :), '] x + [', Dtxt(ii, :), '] u'];
			else
				% char(8294) is "Zero-width non-joiner", a whitespace character that is not removed in old versions of Matlab, where a non-break space is removed, to keep indentation
				linesout{ii, 1} = [char(8204), char(160), '   [', Ctxt(ii, :), ']     [', Dtxt(ii, :), ']  '];
			end
		end
		txt = [
			['System: ', num2str(idx)], linesdyn', {''}, linesout', {['Eigenvalue: ', num2str(pos(1) + 1i*pos(2))]}
		];
	else
		txt = [{'Error in Datacursor function.'}, {['Eigenvalue: ', num2str(pos(1) + 1i*pos(2))]}];
	end
end