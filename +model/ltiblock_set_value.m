function [system] = ltiblock_set_value(system, name, block)
	%LTIBLOCK_SET_VALUE set value of system model to value of supplied ltiblock 
	%	Input:
	%		system:	system to replace block value in
	%		name:	name of block to replace value in
	%		block:	value to replace
	%	Output:
	%		system:	system with replaced value
	if ~isa(system, 'genss')
		if isa(system, 'ss') || isa(system, 'tf') || isa(system, 'uss') || isa(system, 'dss')
			return;
		else
			error('model:ltiblock:input', 'System must be a control system, not a ''%s''.', class(system));
		end
	end
	if ~ischar(name)
		error('model:ltiblock:input', 'Block name must be a string.');
	end
	isrealp = isa(block, 'realp');
	if ~isrealp && ~model.isltiblock(block)
		error('model:ltiblock:input', 'Block must be a valid ltiss, not a ''%s''.', class(block));
	end
	blocknames = fieldnames(system.Block);
	if ~any(strcmp(blocknames, name))
		error('model:ltiblock:input', 'System does not contain block ''%s''.', name);
	end
	systemblock = system.Block.(name);
	systemclass = class(systemblock);
	blockclass = class(block);
	% realp
	if isa(systemblock, 'realp')
		if isrealp
			system.Block.(name).Value = block.Value;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	% state space
	if isa(systemblock, 'ltiblock.ss')
		if isa(block, 'ltiblock.ss')
			system.Block.(name).a = block.a;
			system.Block.(name).b = block.b;
			system.Block.(name).c = block.c;
			system.Block.(name).d = block.d;
		elseif isa(block, 'tunableSS')
			system.Block.(name).a = block.A;
			system.Block.(name).b = block.B;
			system.Block.(name).c = block.C;
			system.Block.(name).d = block.D;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	if isa(systemblock, 'tunableSS')
		if isa(block, 'ltiblock.ss')
			system.Block.(name).A = block.a;
			system.Block.(name).B = block.b;
			system.Block.(name).C = block.c;
			system.Block.(name).D = block.d;
		elseif isa(block, 'tunableSS')
			system.Block.(name).A = block.A;
			system.Block.(name).B = block.B;
			system.Block.(name).C = block.C;
			system.Block.(name).D = block.D;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	% static gain
	if isa(systemblock, 'ltiblock.gain') || isa(systemblock, 'tunableGain')
		if isa(block, 'ltiblock.gain') || isa(block, 'tunableGain')
			system.Block.(name).Value = block.Value;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	% transfer function
	isltitf = isa(systemblock, 'ltiblock.tf');
	istf = isa(systemblock, 'tunableTF');
	if isltitf
		if isa(block, 'ltiblock.tf')
			system.Block.(name).num.Value = block.num.Value;
			system.Block.(name).den.Value = block.den.Value;
		elseif isa(block, 'tunableTF')
			system.Block.(name).num.Value = block.Numerator.Value;
			system.Block.(name).den.Value = block.Denominator.Value;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	if istf
		if isa(block, 'ltiblock.tf')
			system.Block.(name).Numerator.Value = block.num.Value;
			system.Block.(name).Denominator.Value = block.den.Value;
		elseif isa(block, 'tunableTF')
			system.Block.(name).Numerator.Value = block.Numerator.Value;
			system.Block.(name).Denominator.Value = block.Denominator.Value;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	% pid block
	if isa(systemblock, 'ltiblock.pid') || isa(systemblock, 'tunablePID')
		if isa(block, 'ltiblock.pid') || isa(block, 'tunablePID')
			system.Block.(name).Ts.Value = block.Ts.Value;
			system.Block.(name).Kp.Value = block.Kp.Value;
			system.Block.(name).Ki.Value = block.Ki.Value;
			system.Block.(name).Kd.Value = block.Kd.Value;
		else
			error('model:ltiblock:input', 'Block to replace (''%s'') must be of the same type as replaced block (''%s'').', systemclass, blockclass);
		end
		return;
	end
	% pid block
	if isa(systemblock, 'ltiblock.pid2') || isa(systemblock, 'tunablePID2')
		% TODO: ltiblock.pid2
		error('model:ltiblock:input:todo', 'PID with 2 DOF not yet implemented.');
	end
	% no valid ltiblock
	error('model:ltiblock:input', 'System must be supplied as ltiblock, not as ''%s''.', class(block));
end