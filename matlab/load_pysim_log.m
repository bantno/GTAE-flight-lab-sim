function [sigs, t, raw] = load_pysim_log(file)
%LOAD_PYSIM_LOG  Load a flight log written by the Python sim.
%
%   [sigs, t] = load_pysim_log('flightlogs/111526_140307.mat')
%
%   The Python sim cannot write a Simulink.SimulationData.Dataset (that is a
%   MATLAB object only Simulink can construct), so it writes the same content
%   three ways:
%
%     raw.sigsOut  1xN struct array, elements have .Name and .Values.Time/.Data
%                  - the same access pattern as the Dataset's getElement(i)
%     raw.sigs     struct keyed by a valid MATLAB name, e.g. sigs.phi__deg_
%     raw.t        the time vector
%
%   plus every parameter from sim_init.mat, as the model's StopFcn used to save.
%
%   This helper returns `sigs` as a struct keyed by the ORIGINAL signal names
%   made valid, and also prints what it found.

raw = load(file);
t = raw.t;

sigs = struct();
if isfield(raw, 'sigsOut')
    ds = raw.sigsOut;
    for i = 1:numel(ds)
        nm = ds(i).Name;
        if iscell(nm), nm = nm{1}; end
        field = matlab.lang.makeValidName(nm);
        sigs.(field) = ds(i).Values.Data;
    end
elseif isfield(raw, 'sigs')
    sigs = raw.sigs;
end

fn = fieldnames(sigs);
fprintf('%s\n', file);
fprintf('  %d samples, t = %.3f .. %.3f s\n', numel(t), t(1), t(end));
fprintf('  %d signals:\n', numel(fn));
for i = 1:numel(fn)
    d = sigs.(fn{i});
    fprintf('    %-24s [%d x %d]\n', fn{i}, size(d,1), size(d,2));
end
end
