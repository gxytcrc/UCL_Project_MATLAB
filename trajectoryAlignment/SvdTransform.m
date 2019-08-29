function [R_out] = SvdTransform(R_add)
[U,S,V] = svd(R_add);
R_out = U*V';
if det(R_add)<0
    R_out = U*diag([1 1 -1])*V';
end
end

