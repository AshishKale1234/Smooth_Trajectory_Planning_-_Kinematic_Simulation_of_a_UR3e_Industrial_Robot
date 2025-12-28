function [thetalist, success] = ECE569_IKinBody(Blist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector
%           position error less than ev.
% Returns thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%         success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.

thetalist = thetalist0;
i = 0;
maxiterations = 200;

% TODO: calculate Vb
% Hint: you will need to use four of the ECE569 functions from earlier
Tsb = ECE569_FKinBody(M, Blist, thetalist);              % current pose
Tbd = ECE569_TransInv(Tsb) * T;                          % body-frame error
Vb  = ECE569_se3ToVec(ECE569_MatrixLog6(Tbd));           % 6×1 twist error

err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;

while err && i < maxiterations
    % TODO: update thetalist
    % Hint: the psuedo-inverse is given in MATLAB by pinv()
    Jb = ECE569_JacobianBody(Blist, thetalist);          % body Jacobian
    thetalist = thetalist + pinv(Jb) * Vb;               % Newton step

    i = i + 1;

    % recompute error twist Vb with updated thetalist
    Tsb = ECE569_FKinBody(M, Blist, thetalist);
    Tbd = ECE569_TransInv(Tsb) * T;
    Vb  = ECE569_se3ToVec(ECE569_MatrixLog6(Tbd));

    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
end

success = ~err;
end
