function error = build_error(desired, current)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUT: error 6x1
% INPUT: desired 6x1, orientation is in euler format
%         current 6x1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    error = [0;0;0;0;0;0];
    %% position error
    error(1:3) = desired(1:3)-current(1:3);
    
    %% orientation error
    % turn the euler angle into quaternion
    d_euler = desired(4:6);
    c_euler = current(4:6);

    dq = quaternion(flipud(d_euler)','euler','ZYX','point');
    cq = quaternion(flipud(c_euler)','euler','ZYX','point');
    
    % use quaternion to calculate error
    if (compact(cq)*compact(dq)'<0)
        cq = -cq;
    end
    
    error_q = conj(cq)*dq;
    error_q_matrix_format = compact(error_q);
    
    error(4) = error_q_matrix_format(2);
    error(5) = error_q_matrix_format(3);
    error(6) = error_q_matrix_format(4);
    
    % transform to base frame
    cm = rotmat(cq,'point');% current matrix
    error(4:6) = - cm * error(4:6);
end