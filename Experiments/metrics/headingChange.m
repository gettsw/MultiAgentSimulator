function Theta = headingChange(ori)
% Total change of direction: Theta = sum_k |dtheta_k| with each step wrapped to [-pi, pi].
% ori is the n x 1 heading history (KinematicState.ori, which only updates while moving).
d = diff(ori(:));
Theta = sum(abs(atan2(sin(d), cos(d))));
end
