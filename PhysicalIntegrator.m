classdef PhysicalIntegrator < handle
    methods (Static)
        function update(state, dt)
            state.vel = state.vel + state.acc * dt;
            vNorm = norm(state.vel);
            if vNorm > state.maxSpeed
                state.vel = (state.vel / vNorm) * state.maxSpeed;
            end
            state.pos = state.pos + state.vel * dt;
            if norm(state.vel) > 0.1
                state.ori = atan2(state.vel(2), state.vel(1));
            end
        end
    end
end