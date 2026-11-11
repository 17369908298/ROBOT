function result = identify_yaw_6020(csv_files, options)
% Fit VOFA I0..I9 CSV or canonical schema-5/4/3 CSV. No serial port is opened.
% No phase shift, age-based alignment, or automatic sign reversal is applied.
% Example: r = identify_yaw_6020("../data/run.csv");
arguments
    csv_files string
    options.TrimSeconds (1,1) double {mustBeNonnegative} = 3
    options.SmoothSamples (1,1) double {mustBeInteger,mustBePositive} = 5
    options.Plot (1,1) logical = true
    options.VerifyCRC (1,1) logical = true
end
legacy_names = split("time_s tau_cmd_nm theta_rad omega_rad_s tau_base_nm tau_id_nm mode flags " + ...
    "motor_feedback_age_ms imu_age_ms frame_sequence motor_sequence imu_sequence tx_drops " + ...
    "can_submit_age_ms tau_meas_nm can_submit_torque_nm excitation_frequency_hz " + ...
    "excitation_amplitude_nm excitation_stage motor_temperature_c pitch_rad schema_version frame_crc24");
compact_names = split("time_s tau_cmd_nm theta_rad omega_rad_s mode flags frame_sequence " + ...
    "feedback_age_ms schema_version frame_crc24");
imu_names = split("time_s tau_cmd_nm theta_rad omega_rad_s mode flags frame_sequence " + ...
    "imu_time_s schema_version frame_crc24");
samples = []; raw_torque = []; source_file = []; rejected = 0; blocks = 0; duplicates = 0;
for file_index = 1:numel(csv_files)
    T = readtable(csv_files(file_index), VariableNamingRule="preserve");
    % VOFA export may add a host Time column; select named device channels only.
    variables=string(T.Properties.VariableNames);
    channels=nan(1,numel(variables));
    for column=1:numel(variables)
        token=regexp(variables(column),'(?i)^(?:I|CH|CHANNEL)[ _]?([0-9]+)$','tokens','once');
        if ~isempty(token), channels(column)=str2double(token{1}); end
    end
    if all(ismember(0:9,channels)) && ~ismember("time_s",variables)
        [~,order]=ismember(0:9,channels);
        serial_version=double(T{:,order(9)});
        serial_version=serial_version(isfinite(serial_version));
        assert(~isempty(serial_version),'No finite version channel in VOFA CSV.');
        version=median(serial_version);
        if version==5, mapped_names=imu_names; elseif version==4, mapped_names=compact_names;
        else, error('VOFA I8 does not contain schema version 5 or 4; check the selected columns.'); end
        T=T(:,order); T.Properties.VariableNames=cellstr(mapped_names);
    end
    timestamped=ismember("imu_time_s",T.Properties.VariableNames);
    compact=timestamped || ismember("feedback_age_ms",T.Properties.VariableNames);
    if timestamped
        names=imu_names; schema=5;
    elseif compact
        names=compact_names; schema=4;
    else
        names=legacy_names; schema=3;
    end
    assert(all(ismember(names,T.Properties.VariableNames)), ...
        'Expected VOFA I0..I9 or canonical schema-5/4/3 columns.');
    d = double(T{:,cellstr(names)});
    assert(size(d,1)>20,'Too few frames');
    if options.VerifyCRC
        crc_ok = frame_crc_ok(d);
        if ~any(crc_ok)
            error('No matching frame CRC. VOFA decimal export may round float32 values: export full precision, or explicitly use VerifyCRC=false for a checked VOFA CSV.');
        end
    else
        warning('Byte-level CRC checking is disabled for decimal CSV; state, version, finite-value and timestamp checks remain enabled.');
        crc_ok=true(size(d,1),1);
    end
    % Stop/invalid/stale/timing/CAN-submit failure. Keep slew/clamp/governor frames:
    % I1 already includes those changes, so their torque must not be subtracted.
    invalid_mask = uint32(1+4+8+16+512+1024+8192+16384);
    if timestamped
        mode=d(:,5); flags=d(:,6); measurement_time=d(:,8);
        feedback_age=(d(:,1)-measurement_time)*1000;
    elseif compact
        mode=d(:,5); flags=d(:,6); feedback_age=d(:,8);
        measurement_time=d(:,1);
    else
        mode=d(:,7); flags=d(:,8); feedback_age=max(d(:,9),d(:,10));
        measurement_time=d(:,1);
    end
    good = crc_ok & all(isfinite(d),2) & d(:,end-1)==schema & mode>0 & ...
        bitand(uint32(flags),invalid_mask)==0 & feedback_age>=-.1 & feedback_age<50;
    active = mode>0 & isfinite(d(:,1));
    begins = find(active & [true;~active(1:end-1) | diff(d(:,1))<=0]);
    for first = begins'
        last = first;
        while last<size(d,1) && active(last+1) && d(last+1,1)>d(last,1)
            last = last+1;
        end
        trim = d(:,1)>=d(first,1)+options.TrimSeconds & ...
               d(:,1)<=d(last,1)-options.TrimSeconds;
        window = (first:last)';
        if timestamped
            % Keep the first control sample of a received IMU frame. Repeated sensor
            % values with the same receive stamp are not independent measurements.
            repeated=[false;diff(measurement_time(window))==0] & good(window);
            duplicates=duplicates+sum(repeated);
            window=window(~repeated);
        end
        selected = good(window) & trim(window);
        dt = diff(d(window,1)); dt = dt(isfinite(dt) & dt>0);
        if isempty(dt), continue; end
        max_gap = 2.5*median(dt);
        imu_dt=diff(measurement_time(window)); imu_dt=imu_dt(isfinite(imu_dt) & imu_dt>0);
        if isempty(imu_dt), continue; end
        max_imu_gap=2.5*median(imu_dt);
        % Split at a missing/corrupt frame or a device-time gap, BEFORE filtering/differentiation.
        starts = find(selected & [true;~selected(1:end-1) | diff(d(window,1))>max_gap | ...
            diff(measurement_time(window))<=0 | diff(measurement_time(window))>max_imu_gap]);
        for head = starts'
            tail=head;
            while tail<numel(window) && selected(tail+1) && d(window(tail+1),1)-d(window(tail),1)<=max_gap && ...
                    measurement_time(window(tail+1))>measurement_time(window(tail)) && ...
                    measurement_time(window(tail+1))-measurement_time(window(tail))<=max_imu_gap
                tail=tail+1;
            end
            idx=window(head:tail); edge=max(3,options.SmoothSamples);
            if numel(idx)<2*edge+20, continue; end
            time=measurement_time(idx);
            % Identical centered filtering on both sides of the model; raw CSV remains untouched.
            omega=movmean(d(idx,4),options.SmoothSamples);
            torque=movmean(d(idx,2),options.SmoothSamples);
            accel=gradient(omega,time);
            use=(edge+1):(numel(idx)-edge);
            samples=[samples;accel(use),omega(use),torque(use)]; %#ok<AGROW>
            raw_torque=[raw_torque;d(idx(use),2)]; %#ok<AGROW>
            source_file=[source_file;repmat(file_index,numel(use),1)]; %#ok<AGROW>
            blocks=blocks+1;
        end
    end
    rejected=rejected+sum(~crc_ok);
end
assert(size(samples,1)>100,'Insufficient usable data after CRC, state, crop, and continuity checks.');
alpha=samples(:,1); omega=samples(:,2); torque=samples(:,3);
best=Inf;
for smooth_speed=logspace(log10(.02),log10(2),80)
    phi=[alpha,omega,tanh(omega/smooth_speed)];
    coeff=phi\torque;
    prediction=phi*coeff;
    mse=mean((prediction-torque).^2);
    if mse<best
        best=mse; result=struct('J',coeff(1),'B',coeff(2),'Fc',coeff(3), ...
            'omega_s',smooth_speed,'prediction',prediction,'phi',phi);
    end
end
result.RMSE=sqrt(best); result.MAE=mean(abs(result.prediction-torque));
result.R2=1-sum((result.prediction-torque).^2)/sum((torque-mean(torque)).^2);
result.cond=cond(result.phi); result.samples=size(samples,1); result.crc_rejected=rejected;
result.blocks=blocks; result.phase_shift_s=0;
result.crc_verified=options.VerifyCRC;
result.imu_duplicate_frames_rejected=duplicates;
result.physical=result.J>0 && result.B>=0 && result.Fc>=0;
result.raw_torque=raw_torque; result.source_file=source_file;
fprintf('6020 identification: %d samples / %d blocks; CRC rejected %d\n',result.samples,blocks,rejected);
fprintf('Phase shift = 0 s; feedback ages are not a command-response delay.\n');
fprintf('Schema 5 differentiates against IMU receive timestamps; repeated IMU frames removed: %d\n',duplicates);
fprintf('J = %.9g kg*m^2, B = %.9g N*m*s/rad, Fc = %.9g N*m, omega_s = %.9g rad/s\n', ...
    result.J,result.B,result.Fc,result.omega_s);
fprintf('RMSE = %.9g N*m, MAE = %.9g N*m, R2 = %.6f, cond(Phi) = %.6g\n', ...
    result.RMSE,result.MAE,result.R2,result.cond);
if ~result.physical
    warning('Nonphysical coefficients: do not copy this result into feedforward. Inspect signs, actual-current tracking, synchronization, speed coverage and residuals.');
end
if result.omega_s<=.020001 || result.omega_s>=1.99999
    warning('omega_s reached a search boundary; B and Fc may not be independently identifiable.');
end
if options.Plot
    figure('Name','6020 identification');
    tiledlayout(2,1);
    nexttile;plot(torque,result.prediction,'.');hold on;plot(xlim,xlim,'k--');
    xlabel('recorded total torque (N*m)');ylabel('predicted torque (N*m)');grid on;
    nexttile;plot(result.prediction-torque);xlabel('usable sample');ylabel('residual (N*m)');grid on;
end
end

function valid=frame_crc_ok(d)
% Vectorized CRC32/ISO-HDLC over all original float32 values except the final CRC.
[~,~,endian]=computer;
assert(endian=='L','This decoder expects a little-endian host.');
byte_count=(size(d,2)-1)*4;
bytes=reshape(typecast(reshape(single(d(:,1:end-1))',[],1),'uint8'),byte_count,[]);
crc=repmat(uint32(hex2dec('FFFFFFFF')),1,size(d,1));
for index=1:byte_count
    crc=bitxor(crc,uint32(bytes(index,:)));
    for bit=1:8
        crc=bitxor(bitshift(crc,-1),uint32(hex2dec('EDB88320')).*bitand(crc,uint32(1)));
    end
end
valid=(double(bitand(bitcmp(crc),uint32(hex2dec('FFFFFF'))))'==d(:,end));
end
