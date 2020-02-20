clc
clear


syms r hp p
assume([r hp p], 'real')
M = [r hp; 0 p]
tria(M, 2)
