function [message] = solvopt_status2message(status)
	%IPOPT_STATUS2MESSAGE convert solvopt return status into message
	%	Input:
	%		status:		status number returned by solvopt
	%	Output:
	%		message:	message describing status
	switch status
		otherwise
			message = '';
	end
end