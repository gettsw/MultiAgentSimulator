function Ea = accelCost(acc, dt)
% Integrated squared acceleration: E_a = int_0^T ||a(t)||^2 dt, summed over the fixed steps dt.
% acc is the n x 2 commanded acceleration history.
Ea = sum(sum(acc.^2, 2)) * dt;
end
