function lowA = tria(A, N)
[Q,R] = qr(A');
lowA = R';
end