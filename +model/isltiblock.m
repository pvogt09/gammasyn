function [is] = isltiblock(block)
	%ISLTIBLOCK return if a block in a ltiblock
	%	Input:
	%		block:	variable to check
	%	Output:
	%		is:		true, if the variable is a ltiblock
	if iscell(block)
		is = cellfun(@model.isltiblock, block, 'UniformOutput', true);
	else
		is = isa(block, 'ltiblock.gain') || isa(block, 'ltiblock.ss') || isa(block, 'ltiblock.tf') || isa(block, 'ltiblock.pid') || isa(block, 'ltiblock.pid2');
		if ~is
			is = isa(block, 'tunableGain') || isa(block, 'tunableSS') || isa(block, 'tunableTF') || isa(block, 'tunablePID') || isa(block, 'tunablePID2');
		end
	end
end