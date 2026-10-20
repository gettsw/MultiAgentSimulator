function L = pathLength(pos)
% Distance travelled along a trajectory: L = sum_k ||p_{k+1} - p_k||. pos is n x 2.
L = sum(vecnorm(diff(pos, 1, 1), 2, 2));
end
