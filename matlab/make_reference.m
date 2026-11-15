function make_reference(varargin)
%MAKE_REFERENCE  Run AE4803Sim headless with recorded inputs; save reference data.
%
%   make_reference()            uses the model's own solver settings
%   make_reference('LabDir', D) D = folder holding the original AE4803Sim.slx
%
%   Optional name/value pairs let you override the solver so you can separate
%   "model difference" from "integration error in the reference itself":
%
%   make_reference('Solver','ode45','RelTol','1e-10','AbsTol','1e-12', ...
%                  'MaxStep','0.01','OutFile','ref_tight.mat')
%
%   SAFETY: this function NEVER touches the original lab folder. It copies the
%   model into a scratch working directory, loads THAT copy, modifies it only in
%   memory, simulates, and closes it with close_system(model, 0) so nothing is
%   saved. The original AE4803Sim.slx is opened read-only (copyfile source).
%
%   Outputs  matlab/reference/ref_run.mat  containing a struct `ref` with one
%   field per logged signal, each an [N x (1+k)] array of [time, data...], plus
%   the input profile that was injected.

here = fileparts(mfilename('fullpath'));
repo = fileparts(here);

opt = struct('LabDir', fullfile(fileparts(repo), 'AE4803 Flight Simulator'), ...
             'Solver', '', 'RelTol', '', 'AbsTol', '', 'MaxStep', '', ...
             'Inertia', '', 'OutFile', 'ref_run.mat');
for k = 1:2:numel(varargin)
    name = varargin{k};
    assert(isfield(opt, name), 'unknown option "%s"', name);
    opt.(name) = varargin{k+1};
end
labDir = opt.LabDir;

model    = 'AE4803Sim';
workDir  = fullfile(tempdir, 'ae4803_ref_work');
refDir   = fullfile(here, 'reference');
inputMat = fullfile(refDir, 'replay_input.mat');

assert(isfile(fullfile(labDir, [model '.slx'])), ...
    'Cannot find %s.slx in %s', model, labDir);
assert(isfile(inputMat), ...
    'Missing %s - run tools/make_input_profile.py first', inputMat);

% ---- scratch copy -------------------------------------------------------
if isfolder(workDir), rmdir(workDir, 's'); end
mkdir(workDir);
mkdir(fullfile(workDir, 'flightlogs'));   % StopFcn writes here; keep it off the lab
copyfile(fullfile(labDir, [model '.slx']),  fullfile(workDir, [model '.slx']));
copyfile(fullfile(labDir, 'sim_init.mat'),  fullfile(workDir, 'sim_init.mat'));

oldDir = pwd;
cleanupDir = onCleanup(@() cd(oldDir));
cd(workDir);

load('sim_init.mat');                                     %#ok<LOAD>
inp = load(inputMat);
t_in = inp.t(:);
roll_ts     = [t_in, inp.roll(:)];
pitch_ts    = [t_in, inp.pitch(:)];
yaw_ts      = [t_in, inp.yaw(:)];
throttle_ts = [t_in, inp.throttle(:)];
assignin('base', 'roll_ts', roll_ts);
assignin('base', 'pitch_ts', pitch_ts);
assignin('base', 'yaw_ts', yaw_ts);
assignin('base', 'throttle_ts', throttle_ts);

tEnd = t_in(end);

% ---- load the COPY, modify in memory only -------------------------------
load_system(fullfile(workDir, [model '.slx']));
closer = onCleanup(@() closeNoSave(model));

% The model's own StopFcn saves the whole workspace into ./flightlogs; we do
% not want that side effect during a batch reference run.
set_param(model, 'StopFcn', '');

% Replace the live joystick with the recorded command time series. The Joystick
% subsystem only communicates through global Goto tags, so deleting it and
% re-publishing the same four tags leaves the rest of Controls untouched.
delete_block([model '/Controls/Joystick']);

tags = {'command_roll_joystick',     'roll_ts',     [-430 -150];
        'command_pitch_joystick',    'pitch_ts',    [-430 -100];
        'command_yaw_joystick',      'yaw_ts',      [-430  -50];
        'command_throttle_joystick', 'throttle_ts', [-430    0]};
for i = 1:size(tags, 1)
    fw = sprintf('%s/Controls/InjFW%d', model, i);
    gt = sprintf('%s/Controls/InjGoto%d', model, i);
    p  = tags{i,3};
    add_block('simulink/Sources/From Workspace', fw, ...
        'VariableName', tags{i,2}, ...
        'SampleTime',   '0', ...
        'Interpolate',  'on', ...         % linear interp, matches ae4803.replay.InterpolatedInput
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [p(1) p(2) p(1)+90 p(2)+24]);
    add_block('simulink/Signal Routing/Goto', gt, ...
        'GotoTag', tags{i,1}, 'TagVisibility', 'global', ...
        'Position', [p(1)+130 p(2) p(1)+220 p(2)+24]);
    add_line([model '/Controls'], sprintf('InjFW%d/1', i), ...
                                  sprintf('InjGoto%d/1', i));
end

% Keep the batch run quiet and fast: no UDP traffic, no pacing.
set_param([model '/X-Plane11' char(10) 'Visualization'], 'Commented', 'on');
set_param(model, 'EnablePacing', 'off');
set_param(model, 'StopTime', num2str(tEnd));
set_param(model, 'SignalLogging', 'on');
set_param(model, 'SignalLoggingName', 'sigsOut');

% Optional solver overrides (used to build a tight-tolerance "truth" run).
if ~isempty(opt.Solver),  set_param(model, 'Solver',  opt.Solver);  end
if ~isempty(opt.RelTol),  set_param(model, 'RelTol',  opt.RelTol);  end
if ~isempty(opt.AbsTol),  set_param(model, 'AbsTol',  opt.AbsTol);  end
if ~isempty(opt.MaxStep), set_param(model, 'MaxStep', opt.MaxStep); end

% Optional inertia override, used to probe how the 6DOF block interprets the
% (non-symmetric) inertia matrix stored in the block mask.
if ~isempty(opt.Inertia)
    set_param([model '/Generic Light Aircraft Physics/Aircraft Dynamics/6DOF (Quaternion)'], ...
              'inertia', opt.Inertia);
    fprintf('inertia overridden with: %s\n', opt.Inertia);
end

fprintf('simulating %s for %.2f s (solver %s, MaxStep %s)...\n', ...
    model, tEnd, get_param(model,'Solver'), get_param(model,'MaxStep'));
simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

% ---- flatten the dataset into plain arrays ------------------------------
ds = simOut.get('sigsOut');
ref = struct();
names = {};
for i = 1:ds.numElements
    e = ds.getElement(i);
    nm = e.Name;
    if isempty(nm), nm = sprintf('unnamed%d', i); end
    v = e.Values;
    field = matlab.lang.makeValidName(nm);
    ref.(field) = [v.Time, squeeze(v.Data)];
    names{end+1} = nm;                                     %#ok<AGROW>
    fprintf('  %-22s -> %-22s  [%d x %d]\n', nm, field, size(ref.(field),1), size(ref.(field),2));
end

if ~isfolder(refDir), mkdir(refDir); end
outFile = fullfile(refDir, opt.OutFile);
solver  = get_param(model, 'Solver');
maxstep = get_param(model, 'MaxStep');
reltol  = get_param(model, 'RelTol');
abstol  = get_param(model, 'AbsTol');
save(outFile, 'ref', 'names', 't_in', 'roll_ts', 'pitch_ts', 'yaw_ts', ...
     'throttle_ts', 'tEnd', 'solver', 'maxstep', 'reltol', 'abstol', '-v7');
fprintf('wrote %s\n', outFile);
end

% -------------------------------------------------------------------------
function closeNoSave(model)
% Close the in-memory model, discarding every change we made.
try
    if bdIsLoaded(model)
        close_system(model, 0);
        fprintf('closed %s without saving\n', model);
    end
catch ME
    fprintf(2, 'warning: could not close %s cleanly: %s\n', model, ME.message);
end
end
