function [xout, DV, IV] = minsearch(FUN, x, options, varargin)
	%minsearch  Finds the minimum of a function of several variables.
	%
	%   X=minsearch('FUN',X0) starts at the matrix X0 and finds a minimum to the
	%   function which is described in FUN (usually an M-file: FUN.M).
	%   The function 'FUN' should return a scalar function value: f=FUN(X) and the
	%   partial derivatives of the function, df/dX, at the point X.
	%
	%   X=FMINU('FUN',X0,OPTIONS) allows a vector of optional parameters to
	%   be defined. OPTIONS(1) controls how much display output is given; set
	%   to 1 for a tabular display of results, (default is no display: 0).
	%   OPTIONS(2) is a measure of the precision required for the values of
	%   X at the solution. OPTIONS(3) is a measure of the precision
	%   required of the objective function at the solution.
	%   For more information type HELP FOPTIONS.
	%
	%   X=FMINU('FUN',X0,OPTIONS,'GRADFUN',P1,P2,...) passes the problem-
	%   dependent parameters P1,P2,... directly to the functions FUN
	%   and GRADFUN: FUN(X,P1,P2,...) and GRADFUN(X,P1,P2,...).  Pass
	%   empty matrices for OPTIONS, and 'GRADFUN' to use the default
	%   values.
	%
	%   [X,OPTIONS]=FMINU('FUN',X0,...) returns the parameters used in the
	%   optimization method.  For example, options(10) contains the number
	%   of function evaluations used.
	%
	%   The default algorithm is the BFGS Quasi-Newton method with a
	%   mixed quadratic and cubic line search procedure.

	%   Copyright (c) 1990-98 by The MathWorks, Inc.
	%   $Revision: 1.28 $  $Date: 1998/08/31 22:29:16 $
	%   Andy Grace 7-9-90.


	% ------------Initialization----------------
	IVMAX = 20;
	xout = x(:);
	N = length(xout);

	if nargin < 2
		error('oplace:optimize:function', 'minsearch requires at least two input arguments');
	end
	if nargin == 2
		options = [];
	end

	% Convert to inline function as needed.
	if ~isempty(FUN)
		if ischar(FUN)
			[optfkt, msg] = fcnchk(FUN,length(varargin));
			if ~isempty(msg)
				error('oplace:optimize:function', msg);
			end
		elseif isa(FUN, 'function_handle')
			optfkt = FUN;
		else
			error('oplace:optimize:function', 'FUN must be a function name or valid expression.');
		end
	else
		error('oplace:optimize:function', 'FUN must be a function name or valid expression.')
	end

	IV = zeros(IVMAX, 1);
	dimmax = 10+N*(N+9)/2;
	DV = zeros(dimmax, 1);

	[IV, DV] = set_opt_para(IV, DV, options);

	true=1;
	false=0;
	ISTART=1;
	IOUTNR=2;
	IPRINT=3;
	MAXITN=4;
	MAXIFN=5;
	ITN=6;
	IFN=7;
	IGN=8;
	IEXIT=11;
	IOLDN=12;
	IOLDX=13;
	IDVA=14;
	IF=1;
	IEPSG=2;
	IEPSX=4;
	IDF=6;
	IEPSA=7;
	ITOT=8;

	% ANFANGSWERTE UND FELDEINTEILUNG

	if (IV(ISTART)==0)
	   [IV,DV]=finished(N,x,IV(IDVA)+2*N,IV,DV,0);
	   xout=x;
	   return;
	end
	IOUT=IV(IOUTNR);
	IG=IV(IDVA);
	IS=IG+N;
	IU=IS+N;
	IW=IU+N;
	IL=IW+N;
	N1=N-1;
	IGN1=IG+N-1;
	ISN1=IS+N-1;
	IUN1=IU+N-1;
	ZEROCN=control.design.oplace.machconst(3);

	Iswitch=IV(ISTART);
	if (Iswitch==2 | Iswitch==10)
		%
		% NEUSTART DER ITERATION
		%
		IV(ITN)=0;
		IV(IFN)=1;
		IV(IGN)=1;
		IJ=IL+N*(N+1)/2;
		for I=1:N
			for J=1:I
				IJ=IJ-1;
				DV(IJ)=0.0;
			end
			DV(IJ)=1.0;
		end
		[f,grad,IUP] = feval(optfkt,x,varargin{:});
		DV(IF)=f;
		DV(IG:IGN1)=grad(:);
		if (IUP==0)
			[IV,DV]=finished(N,x,IU,IV,DV,1);
			xout=x;
			return;
		end
		DV(IDF)=0.1*abs(DV(IF));
	end

	% ANFANG DER ITERATIONSSCHLEIFE
	while (1)
		if ((Iswitch==2)|(Iswitch==4)|(Iswitch==10))
			IV(IEXIT)=4;
			if (IV(ITN)>=IV(MAXITN))
				[IV,DV]=finished(N,x,IU,IV,DV,2);
				xout=x;
				return;
			end
			monit(N,0,x,IV,DV,3);
			IV(ITN)=IV(ITN)+1;
			DV(IS:dimmax)=cholsol(N,DV(IL:dimmax),DV(IG:dimmax),DV(IS:dimmax));
			for I=0:N1
				DV(IS+I)=-DV(IS+I);
			end
			GS0=dotprd(N,DV(IG:IGN1),DV(IS:ISN1));
			IV(IEXIT)=8;
			if (GS0>=-ZEROCN)
				[IV,DV]=finished(N,x,IU,IV,DV,2);
				xout=x;
				return;
			end
			DV(IU:IUN1)=DV(IG:IGN1);
			% START DER EINDIMENSIONALEN SUCHE
			DSMAX=0.0;
			for I=0:N1
				DSMAX=max(DSMAX,abs(DV(IS+I)));
			end
			DV(IEPSA)=DV(IEPSX)/DSMAX;
			D=0.5*abs(GS0);
			ALFA=min(1.0,-2.0*DV(IDF)/GS0);
			F0=DV(IF);
			F1=DV(IF);
			GS1=GS0;
			DV(ITOT)=0.0;
			INCRF=false;
			DO30=true;
			B=0.0;
			%Label 30
			while (DO30)
				DO30=false;
				for I=1:N
					x(I)=x(I)+ALFA*DV(IS-1+I);
				end
				DV(ITOT)=DV(ITOT)+ALFA;
				if (INCRF)
					ALFA=B;
				end
				IV(IFN)=IV(IFN)+1;
				IV(IGN)=IV(IFN);
				[f,grad,IUP] = feval(optfkt,x,varargin{:});
				DV(IF)=f;
				DV(IG:IGN1)=grad(:);
				if (IUP==0)
					[IV,DV]=finished(N,x,IU,IV,DV,1);
					xout=x;
					return;
				end
				GS2=dotprd(N,DV(IG:IGN1),DV(IS:ISN1));
				INCRF=DV(IF)>=F0;
				DO35=true;
				DO50=false;
				if (IV(IFN)>=IV(MAXIFN))
					DO35=false;
					DO50=true;
				elseif (abs(GS2)<=D)
					DO35=false;
					DO50=true;
				end
				if (DO35)
					if (GS1*GS2 > 0.0)
						% NICHT-UNIMODALE FUNKTION
						A=-0.5;
						if (DV(IF)<=F1)
							% KONKAVE FUNKTION
							A=10.0;
							if (abs(GS2)<=abs(GS1))
								% QUADRATISCHE EXTRAPOLATION MIT SCHRITTWEITENBESCHRAENKUNG AUF 10*ALFA
								B=(DV(IF)-F1)/(GS2*ALFA);
								if (B>1.05)
									A=0.5/(B-1.0);
								end
							end
						end
					else
						% KUBISCHE INTERPOLATION
						A=GS1+GS2-3.0*(DV(IF)-F1)/ALFA;
						B=sign_fortran(sqrt(A*A-GS1*GS2),ALFA);
						A=-(B-A+GS2)/(B+B-GS1+GS2);
					end
					B=(1.0+A)*ALFA;
					ALFA=A*ALFA;
					if (INCRF)
						DO30=true;
						DO50=false;
						% GOTO Label 30
					else
						DO50=true;
						if (abs(ALFA)>DV(IEPSA))
							F1=DV(IF);
							GS1=GS2;
							DO30=true;
							DO50=false;
							% GOTO Label 30
						end
					end
				end %DO35
				if (DO50 & ~DO30)
					D=F0-DV(IF);
					D=max(D,-0.1*DV(ITOT)*GS0);
					if (D<=ZEROCN)
						D=0.1*abs(DV(IF));
					end
					DV(IDF)=D;
				end
			end %While(DO30)
		end %if (Iswitch==2 ...)

		% ENDE DER EINDIMENSIONALEN SUCHE

		if (Iswitch==1 | DO50)
			G2N=v2norm(N,DV(IG:IGN1));
			IV(IEXIT)=1;
			if (G2N<=DV(IEPSG))
				[IV,DV]=finished(N,x,IU,IV,DV,2);
				xout=x;
				return;
			end
		end
		if (Iswitch==3 | DO50)
			IV(IEXIT)=3;
			if (DV(ITOT)<=DV(IEPSA))
				[IV,DV]=finished(N,x,IU,IV,DV,2);
				xout=x;
				return;
			end
		end
		if (Iswitch==5 | DO50)
			IV(IEXIT)=5;
			if (IV(IFN)>=IV(MAXIFN))
				[IV,DV]=finished(N,x,IU,IV,DV,2);
				xout=x;
				return;
			end
		end

		% NEUE SUCHRICHTUNG

		for I=0:N1,
			DV(IS+I)=DV(IG+I)-DV(IU+I);
		end
		GS1=GS2-GS0;
		if (GS1>ZEROCN)
			GS1=1.0/(GS1*DV(ITOT));
			DV(IL:dimmax)=cholmod(N,DV(IL:dimmax),GS1,DV(IS:dimmax),DV(IW:dimmax),0);
			GS0=1.0/GS0;
			DV(IL:dimmax)=cholmod(N,DV(IL:dimmax),GS0,DV(IU:dimmax),DV(IW:dimmax),1);
		end
	end %while(1)
end