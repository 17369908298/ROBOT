% Run from the project root after: python tests/make_fit_fixture.py
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'tools'));
r=identify_yaw_6020(fullfile(root,'build','host_tests','synthetic.csv'),Plot=false);
assert(abs(r.J/.012-1)<.05);
assert(abs(r.B/.02-1)<.15);
assert(abs(r.Fc/.06-1)<.15);
assert(r.R2>.99 && r.crc_rejected==0 && r.phase_shift_s==0);
assert(r.imu_duplicate_frames_rejected>0);
legacy=identify_yaw_6020(fullfile(root,'build','host_tests','synthetic_schema3.csv'),Plot=false);
old_compact=identify_yaw_6020(fullfile(root,'build','host_tests','synthetic_schema4.csv'),Plot=false);
assert(abs(legacy.J-old_compact.J)<1e-10 && abs(legacy.B-old_compact.B)<1e-10 && abs(legacy.Fc-old_compact.Fc)<1e-10);
% VOFA's host Time column is deliberately different from device timestamps.
T=readtable(fullfile(root,'build','host_tests','synthetic.csv'));
T.Properties.VariableNames=cellstr("I"+string(0:9));
T=addvars(T,(1000+(0:height(T)-1)'*.1),'Before',1,'NewVariableNames','Time');
vofa_file=fullfile(root,'build','host_tests','vofa_export.csv');
writetable(T,vofa_file);
vofa=identify_yaw_6020(vofa_file,Plot=false,VerifyCRC=false);
assert(abs(vofa.J-r.J)<1e-7 && abs(vofa.B-r.B)<1e-7);
assert(~vofa.crc_verified);
disp('MATLAB SYNTHETIC FIT PASS');
