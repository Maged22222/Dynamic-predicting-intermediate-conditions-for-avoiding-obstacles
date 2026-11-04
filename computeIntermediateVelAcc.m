function [v_middle, a_middle] = computeIntermediateVelAcc(P1, P2, P3, t1, t2, t3)


    % Directions
    dir_in  = P2 - P1;
    dir_out = P3 - P2;
    dir_in_norm  = dir_in / norm(dir_in);
    dir_out_norm = dir_out / norm(dir_out);

    % Average direction
    dir_avg = dir_in_norm + dir_out_norm;
    dir_avg = dir_avg / norm(dir_avg);

    % Segment lengths and durations
    d1 = norm(dir_in);
    d2 = norm(dir_out);
    dt1 = t2 - t1;
    dt2 = t3 - t2;

    % Average speeds
    v1 = d1 / dt1;
    v2 = d2 / dt2;
    v_mag = (v1 + v2) / 2;

    % Velocity at intermediate waypoint
    v_middle = v_mag * dir_avg;

    % Incoming and outgoing velocities
    v_in = v1 * dir_in_norm;
    v_out = v2 * dir_out_norm;

    % Accelerations
    a_in = (v_middle - v_in) / dt1;
    a_out = (v_out - v_middle) / dt2;

    % Average acceleration at the intermediate waypoint
    a_middle = (a_in + a_out) / 2;
end
