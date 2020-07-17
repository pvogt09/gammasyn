function [X,L,K]=extcare(A,B,C,D)
%EXTCARE  Extended care (for non left-invertible systems).
% [X,L,K]=extcare(A,B,C,D)

[Am,Bm,Cm,Dm,Fs,Us]=extendf(A,B,C,D);
[X,L,Km]=care(Am,Bm,Cm'*Cm,Dm'*Dm,Cm'*Dm);
K=Us*Km-Fs;
% --- end of extcare ---