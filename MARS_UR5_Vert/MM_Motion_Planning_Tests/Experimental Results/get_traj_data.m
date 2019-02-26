function traj_data=get_traj_data(data_raw,stamp_start, duration)
N = size(data_raw,1);
start_time_found = false;
count = 0;
k = 1;
while k < N
    stamp_k = [data_raw(k,1) data_raw(k,2)];
    time_diff = duration_from_stamps(stamp_k, stamp_start);
    
    if time_diff >= 0
        if ~start_time_found
            fprintf('Starting time_diff:%f\n', time_diff);
            fprintf('Index:%d\n', k);
            start_time_found = true;
            k = k - 1;
            continue
        end
    end
    
    if start_time_found
        count = count+1;
        traj_data(:,count) = [time_diff data_raw(k,3:end)]';
    end
    
    if (time_diff > duration)
        fprintf('Final time_diff:%f\n', time_diff);
        fprintf('Index:%d\n', k);
        break
    end
    k = k + 1;
end
traj_data(1,1)=0.0;
traj_data(1,end)=duration;
end