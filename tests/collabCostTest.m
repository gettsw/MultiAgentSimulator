function tests = collabCostTest
% Unit tests of CollabTargetCost (Lemma G recursion and the terminal correction hhat^T) against a
% fine time integration of the target dynamics. Run: runtests('collabCostTest')
tests = functiontests(localfunctions);
end

function testLindleyAgainstIntegration(tc)
rng(3);
for trial = 1:200
    A = 0.5 + 9.5 * rand;  B = 2 + 10 * rand;
    R0 = 20 * rand * (rand > 0.2);
    t0 = 5 * rand;  tf = t0 + 1 + 20 * rand;
    n = randi([0 4]);
    arr = t0 - 2 + (tf - t0 + 4) * rand(1, n);           % some before t0, some after tf
    dep = arr + 15 * rand(1, n);
    ev = [arr, dep];  db = [B * ones(1, n), -B * ones(1, n)];
    [J, Rf] = CollabTargetCost.lindley(R0, A, t0, tf, 0, ev, db);
    [Je, Re] = integrate(R0, A, B, t0, tf, arr, dep);
    verifyEqual(tc, J, Je, 'AbsTol', 2e-3 * max(1, Je));
    verifyEqual(tc, Rf, Re, 'AbsTol', 2e-3 * max(1, Re));
end
end

function testLindleyVectorized(tc)
% one call with several candidate plans gives the same as separate calls
ev = [2 6; 3 Inf; 1 4];  db = [10 -10; 10 0; 10 -10];  tf = [8; 9; 5];
J = CollabTargetCost.lindley(4, 1, 0, tf, 0, ev, db);
for k = 1:3
    verifyEqual(tc, J(k), CollabTargetCost.lindley(4, 1, 0, tf(k), 0, ev(k, :), db(k, :)), 'AbsTol', 1e-12);
end
end

function testTerminal(tc)
% untruncated closed form and its truncation at the mission end
R = 10; g = 1; c = 9; d = 5;
verifyEqual(tc, CollabTargetCost.terminal(R, g, c, d, Inf), 75, 'AbsTol', 1e-12);
verifyEqual(tc, CollabTargetCost.terminal(R, g, c, 0, Inf), 100 / 18, 'AbsTol', 1e-12);
verifyEqual(tc, CollabTargetCost.terminal(R, g, c, d, 0), 0);
s = 3;   % ends while waiting
verifyEqual(tc, CollabTargetCost.terminal(R, g, c, d, s), R * s + g * s^2 / 2, 'AbsTol', 1e-12);
s = 6;   % ends while clearing
Rw = R + g * d;
verifyEqual(tc, CollabTargetCost.terminal(R, g, c, d, s), R * d + g * d^2 / 2 + Rw * 1 - c / 2, 'AbsTol', 1e-12);
end

function [J, R] = integrate(R0, A, B, t0, tf, arr, dep)
dt = 1e-4;  t = t0;  R = R0;  J = 0;
while t < tf - dt / 2
    h = min(dt, tf - t);
    n = sum(arr <= t + h / 2 & dep > t + h / 2);
    r = A - B * n;
    if R <= 0 && r < 0, r = 0; end
    Rn = max(R + r * h, 0);
    J = J + h * (R + Rn) / 2;
    R = Rn;  t = t + h;
end
end
