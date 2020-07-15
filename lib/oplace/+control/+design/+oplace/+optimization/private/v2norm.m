function norm2 = v2norm(P,X)
%
%  ***  RETURN THE 2-NORM OF THE P-VECTOR X, TAKING  ***
%  ***  CARE TO AVOID THE MOST LIKELY UNDERFLOWS.    ***
%
%      INTEGER P
%      DOUBLE PRECISION X(P)
%
%     INTEGER I
%     DOUBLE PRECISION ONE, R, SCALE, SQTETA, T, XI, ZERO
%     DOUBLE PRECISION DABS, DSQRT
%

ONE=1.0;
SQTETA=0.0;
ZERO= SQTETA;

norm2 = ZERO;
if (P <= 0)
	return;
end

I=1;
for j = 1:P,
	if (X(I) ~= ZERO)
		break;
	end
	I=I+1;
end

if (I == P+1)
	return;
end

SCALE = abs(X(I));
if (I == P)
	norm2 = SCALE;
	return;
end
T = ONE;
if (SQTETA == ZERO)
	SQTETA = control.design.oplace.machconst(2);
end

%     ***  SQTETA IS (SLIGHTLY LARGER THAN) THE SQUARE ROOT OF THE
%     ***  SMALLEST POSITIVE FLOATING POINT NUMBER ON THE MACHINE.
%     ***  THE TESTS INVOLVING SQTETA ARE DONE TO PREVENT UNDERFLOWS.

for I = I+1:P,
	XI = abs(X(I));
	if (XI <= SCALE)
		R = XI / SCALE;
		if (R > SQTETA)
			T = T + R*R;
		end
	else
		R = SCALE / XI;
		if (R <= SQTETA)
			R = ZERO;
		end
		T = ONE  +  T * R*R;
		SCALE = XI;
	end
end
norm2 = SCALE * sqrt(T);