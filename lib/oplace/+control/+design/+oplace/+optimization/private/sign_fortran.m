function X = sign_fortran(A,B)
% Emuliert die FORTRAN-Funktion sign(A,B) in MATLAB 

if(B>=0)
   X=abs(A);
else
   X=-abs(A);
end

 

