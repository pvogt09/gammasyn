function DOTPRD=dotprd(P, X, Y)
%  ***  RETURN THE INNER PRODUCT OF THE P-VECTORS X AND Y.  ***
%     INTEGER P
%     DOUBLE PRECISION X(P), Y(P)
%     INTEGER I
%     DOUBLE PRECISION ONE, SQTETA, T, ZERO
%     EXTERNAL MACHCONST
%  ***  MACHCONST(2) RETURNS A MACHINE-DEPENDENT CONSTANT, SQTETA, WHICH
%  ***  IS SLIGHTLY LARGER THAN THE SMALLEST POSITIVE NUMBER THAT
%  ***  CAN BE SQUARED WITHOUT UNDERFLOWING.

ONE=1.0;
SQTETA=0.0;
ZERO=0.0;

DOTPRD = ZERO;
if (P > 0)
	if (SQTETA == ZERO)
		SQTETA = control.design.oplace.machconst(2);
	end
	for I = 1:P,
		T = max(abs(X(I)),abs(Y(I)));
		if (T <= ONE)
			if (T >= SQTETA)
				T = (X(I)/SQTETA)*Y(I);
				if (abs(T) >= SQTETA)
					DOTPRD = DOTPRD + T*SQTETA;
				end
			end
		else
			DOTPRD = DOTPRD + X(I)*Y(I);
		end
	end
end