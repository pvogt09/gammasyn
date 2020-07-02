function [numthreads] = numthreads()
	%NUMTHREADS return number of threads to use when parallelizing loops
	%	Output:
	%		numthreads:		number of threads to use
	persistent numcores;
	if isempty(numcores)
		numcores = uint32(feature('numcores'));
	end
	numthreads = numcores;
end