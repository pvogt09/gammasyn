function [message] = ipopt_status2message(status)
	%IPOPT_STATUS2MESSAGE convert ipopt return status into message
	%	Input:
	%		status:		status number returned by ipopt
	%	Output:
	%		message:	message describing status
	switch status
		case 0
			message = 'Optimal Solution Found.';
		case 1
			message = 'Solved To Acceptable Level.';
		case 2
			message = 'Converged to a point of local infeasibility. Problem may be infeasible.';
		case 3
			message = 'Search Direction is becoming Too Small.';
		case 4
			message = 'Iterates divering; problem might be unbounded.';
		case 5
			message = 'Stopping optimization at current point as requested by user.';
		case 6
			message = 'Feasible point for square problem found.';
		case -1
			message = 'Maximum Number of Iterations Exceeded.';
		case -2
			message = 'Restoration Failed!';
		case -3
			message = 'Error in step computation (regularization becomes too large?)!';
		case -4
			message = 'Maximum CPU time exceeded.';
		case -10
			message = 'Problem has too few degrees of freedom.';
		case -11
			message = 'invalid problem definition';
		case -12
			message = 'invalid option';
		case -13
			message = 'Objective or constraint function returned NaN or Inf.';
		case -100
			message = 'unrecoverable exception';
		case -101
			message = 'Unknown Exception caught in Ipopt';
		case -102
			message = 'Not enough memory.';
		case -199
			message = 'INTERNAL ERROR: Unknown SolverReturn value - Notify IPOPT Authors.';
		otherwise
			message = '';
	end
end