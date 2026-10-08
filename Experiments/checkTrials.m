% Sanity check for runTrial + runSweep + analyzeResults on real (short) simulations.
p = struct('Policy', "Classical RHC [1]", 'UncertaintyMode', "Static Uniform", 'EndTime', 100);
a = runTrial(p); b = runTrial(p);
assert(all(isfinite([a.finalJ a.steadyJ a.peakR])), 'Non-finite metric');
assert(isequal([a.finalJ a.steadyJ a.peakR], [b.finalJ b.steadyJ b.peakR]), ...
    'Repeated trial differs: state leaks between runs');

% Joint-only switch: Classical must run once, Joint once per value -> 3 trials per mode
T = runSweep(struct('Policy', ["Joint RHC", "Classical RHC [1]"], ...
                    'Policy_divergeScoring', ["pair", "classical"], ...
                    'UncertaintyMode', ["Static Uniform", "Dynamic Uncertainty"], ...
                    'EndTime', 60));
assert(height(T) == 6, 'Expected 6 trials after skipping duplicate Classical runs, got %d', height(T));
assert(all(T.Policy_divergeScoring(T.Policy == "Classical RHC [1]") == "n/a"), 'Classical switch not marked n/a');

R = analyzeResults(T, 'Plot', false);
assert(height(R.Paired) == 4 && all(R.Paired.NPairs == 1), 'Sweep output did not pair with the baseline');

% AgentStarts: each start pair is its own setting (and its own pairing key)
S = runSweep(struct('Policy', ["Joint RHC", "Classical RHC [1]"], 'UncertaintyMode', "Static Uniform", ...
                    'AgentStarts', {{[1 2], [2 3]}}, 'EndTime', 30));
assert(height(S) == 4 && all(ismember(S.AgentStarts, ["[1 2]", "[2 3]"])), 'AgentStarts not swept');
RS = analyzeResults(S, 'Plot', false);
assert(RS.Paired.NPairs == 2, 'AgentStarts settings did not pair separately');
fprintf('checkTrials OK  (Classical J=%.2f on Static Uniform, 100 s)\n', a.finalJ);
