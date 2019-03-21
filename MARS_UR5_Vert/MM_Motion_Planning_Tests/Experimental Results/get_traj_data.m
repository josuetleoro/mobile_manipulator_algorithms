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
            time_diff_prev = duration_from_stamps([data_raw(k-1,1) data_raw(k-1,2)], stamp_start);
            [start_time,idx] = min([time_diff_prev, time_diff]);
            %Move the idx of the starting time
            k = k + (idx-1);
            fprintf('Starting time_diff:%f\n', start_time);
            fprintf('Index:%d\n', k);            
            start_time_found = true;
            continue
        end
    end
    
    if start_time_found
        count = count+1;
        traj_data(:,count) = [time_diff data_raw(k,3:end)]';
    end
    
    if (time_diff > duration)
        final_time = time_diff;
        
        time_diff_prev = duration_from_stamps([data_raw(k-1,1) data_raw(k-1,2)], stamp_start);
        time_error = abs(time_diff-duration);
        time_error_prev = abs(time_diff_prev-duration);
        
        %Remove the final element
        if time_error_prev < time_error
            k = k - 1;
            temp = traj_data;
            traj_data = [];
            traj_data = temp(:,1:count-1);
            final_time = time_error_prev;
        end        
        
        fprintf('Final time_diff:%f\n', final_time);
        fprintf('Index:%d\n', k);
        break
    end
    k = k + 1;
end
traj_data(1,1)=0.0;
traj_data(1,end)=duration;
end