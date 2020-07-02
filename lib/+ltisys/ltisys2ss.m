function [sys] = ltisys2ss(lti)
	%LTISYS2SS convert ltisys to ss or dss object
	%	Input:
	%		lti:	ltisys to convert
	%	Output:
	%		sys:	system as ss if E = I, else system as dss
	if ndims(lti) == 2 && ltisys.isltisys(lti) %#ok<ISMAT> compatibility with Octave
		sys = convert(lti);
	elseif ndims(lti) > 2 %#ok<ISMAT> compatibility with Octave
		is = ltisys.isltisysarray(lti);
		if ~all(is(:))
			error('ltisys:input', 'System is not a ltisys.');
		end
		dim = size(lti);
		systest = reshape(lti, [dim(1), dim(2), prod(dim(3:end))]);
		sysarray = cell(size(systest, 3), 1);
		%n_states = NaN(size(systest, 3), 1);
		n_controls = NaN(size(systest, 3), 1);
		n_measurements = NaN(size(systest, 3), 1);
		for ii = 1:size(systest, 3)
			sysarray{ii, 1} = convert(squeeze(systest(:, :, ii)));
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
		error('ltisys:input', 'System is not a ltisys.');
	end
end

function [sys] = convert(lti)
	%CONVERT convert a ltisys to ss or dss
	%	Input:
	%		lti:	ltisys to convert
	%	Output:
	%		sys:	system as ss if E = I, else system as dss
	[rp, cp] = size(lti);

	na = lti(1, cp);

	a = lti(1:na, 1:na);
	b = real(lti(1:na, na + 1:cp - 1));
	c = real(lti(na + 1:rp - 1, 1:na));
	d = real(lti(na + 1:rp - 1, na + 1:cp - 1));
	e = imag(a);
	a = real(a);

	hase = max(abs(e(:))) > 0;
	e = e + eye(na);
	if hase
		sys = dss(a, b, c, d, e);
	else
		sys = ss(a, b, c, d);
	end
end