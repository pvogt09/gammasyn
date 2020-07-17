function [X,L,K]=extdare(A,B,C,D)
%EXDARE   Extended dare (for non left-invertible systems).
% [X,L,K]=extdare(A,B,C,D)

[Am,Bm,Cm,Dm,Fs,Us]=extendf(A,B,C,D);
[X,L,Km]=dare(Am,Bm,Cm'*Cm,Dm'*Dm,Cm'*Dm);
K=Us*Km-Fs;
% --- end of extdare ---