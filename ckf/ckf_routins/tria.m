function res = tria(A, N)
% S = tria(A)
% Returns lower triangular matrix S with NxN dimensions
% Let R be upper triangular matrix from QR decomposition of A': A' = Q*R;
% Then S = R'
res = (triu(qr(A',0)));
res = -(res(1:N, 1:N))';
end