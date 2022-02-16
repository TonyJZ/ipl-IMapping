% This demo script runs the ECO tracker with deep features on the
% included "Crossing" video.

% Add paths
% setup_paths();
if ~isdeployed
    setup_paths();
end

% Run ECO
trax_ECO_sim(true);