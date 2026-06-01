function flag = linesIntersect(A, B, C, D)
    % Returns true if line segments AB and CD intersect
    flag = false;

    % Vector cross products
    d1 = direction(C, D, A);
    d2 = direction(C, D, B);
    d3 = direction(A, B, C);
    d4 = direction(A, B, D);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ...
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        flag = true;
    end
end

function d = direction(P1, P2, P3)
    d = (P3(1) - P1(1))*(P2(2) - P1(2)) - (P3(2) - P1(2))*(P2(1) - P1(1));
end
