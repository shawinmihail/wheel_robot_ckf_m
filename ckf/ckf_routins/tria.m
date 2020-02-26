function lowA = tria(A, N)
[Q,R] = qr(A');
lowA = R';
lowA = lowA(1:N, 1:N);
end