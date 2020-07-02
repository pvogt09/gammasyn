function [sys] = psys2cornermodel(plti)
	%PSYS2CORNERMODEL convert psys vertex systems to ss or dss
	%	Input:
	%		plti:	psys to convert
	%	Output:
	%		sys:	system as ss or dss
	if ndims(plti) == 2 && ltisys.ispsys(plti) %#ok<ISMAT> compatibility with Octave
		sys = convert2corner(plti);
	elseif ndims(plti) == 3
		is = ltisys.ispsysarray(plti);
		if any(~is(:))
			error('ltisys:input', 'System is not a psys.');
		end
		dim = size(plti);
		systest = reshape(plti, [dim(1), dim(2), prod(dim(3:end))]);
		sysarray = cell(size(systest, 3), 1);
		%n_states = NaN(size(systest, 3), 1);
		n_controls = NaN(size(systest, 3), 1);
		n_measurements = NaN(size(systest, 3), 1);
		for ii = 1:size(systest, 3)
			sysarray{ii, 1} = convert2corner(squeeze(systest(:, :, ii)));
			%n_states(ii, 1) = size(sysarray{ii, 1}.a, 1);
			n_controls(ii, 1) = size(sysarray{ii, 1}.b, 2);
			n_measurements(ii, 1) = size(sysarray{ii, 1}.c, 1);
		end
		if any(isnan(n_controls(:))) || any(n_controls(:) ~= n_controls(1))
			error('ltisys:input', 'Number of controls must be equal for all systems.');
		end
		if any(isnan(n_measurements(:))) || any(n_measurements(:) ~= n_measurements(1))
			error('ltisys:input', 'Number of measurements must be equal for all systems.');
		end
		sys = stack(1, sysarray{:});
	else
		error('ltisys:input', 'System is not a psys.');
	end
end

function [sys] = convert2corner(plti)
	%CONVERT2CORNER convert psys to poltopic model and convert vertex systems to ss or dss
	%	Input:
	%		plti:	psys to convert
	%	Output:
	%		sys:	system as ss or dss
	pstype = real(plti(2, 1));
	if pstype == 2
		plti = aff2pol(plti);
	end
	n_vertices = real(plti(3, 1));
	n_states = real(plti(4, 1));
	n_controls = real(plti(5, 1));
	n_measurements = real(plti(6, 1));
	systemmatrices = plti(:, 1:1 + n_vertices*(n_states + n_controls + 2));
	systems = cell(n_vertices, 1);
	for ii = 1:n_vertices
		b = 2 + (ii - 1)*(n_states + n_controls + 2);
		systems{ii, 1} = systemmatrices(1:n_states + n_measurements + 1, b + 1:b + n_states + n_controls + 1);
		if n_states ~= systems{ii, 1}(1, end)
			error('ltisys:input', 'Number of states must be equal for all systems.');
		end
		systems{ii, 1} = ltisys.ltisys2ss(systems{ii, 1});
	end
	%vertices = plti(1:max(2, plti(2, 2 + n_vertices*(n_states + n_controls + 2))), 2 + n_vertices*(n_states + n_controls + 2):size(plti, 2));
	%n_parameters = vertices(2, 1);
	sys = stack(1, systems{:});
end