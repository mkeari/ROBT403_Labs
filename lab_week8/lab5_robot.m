%-----------------------------------------------------------------------
% slCharacterEncoding('ISO-8859-1'); % If you use Matlab R2014a uncomment
% this line
%-----------------------------------------------------------------------

%MDL_PUMA560 Create model of Puma 560 manipulator
%
%      mdl_puma560
%
% Script creates the workspace variable p560 which describes the 
% kinematic and dynamic characteristics of a Unimation Puma 560 manipulator
% using standard DH conventions.
% The model includes armature inertia and gear ratios.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%
% Reference::
% - "A search for consensus among model parameters reported for the PUMA 560 robot",
%   P. Corke and B. Armstrong-Helouvry, 
%   Proc. IEEE Int. Conf. Robotics and Automation, (San Diego), 
%   pp. 1608-1613, May 1994.
%
% See also SerialRevolute, mdl_puma560akb, mdl_stanford, mdl_twolink.

%
% Notes:
%    - the value of m1 is given as 0 here.  Armstrong found no value for it
% and it does not appear in the equation for tau1 after the substituion
% is made to inertia about link frame rather than COG frame.
% updated:
% 2/8/95  changed D3 to 150.05mm which is closer to data from Lee, AKB86 and Tarn
%  fixed errors in COG for links 2 and 3
% 29/1/91 to agree with data from Armstrong etal.  Due to their use
%  of modified D&H params, some of the offsets Ai, Di are
%  offset, and for links 3-5 swap Y and Z axes.
% 14/2/91 to use Paul's value of link twist (alpha) to be consistant
%  with ARCL.  This is the -ve of Lee's values, which means the
%  zero angle position is a righty for Paul, and lefty for Lee.
%  Note that gravity load torque is the motor torque necessary
%  to keep the joint static, and is thus -ve of the gravity
%  caused torque.
%
% 8/95 fix bugs in COG data for Puma 560. This led to signficant errors in
%  inertia of joint 1. 
% $Log: not supported by cvs2svn $
% Revision 1.4  2008/04/27 11:36:54  cor134
% Add nominal (non singular) pose qn



% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

clear L
deg = pi/180;

% joint angle limits from 
% A combined optimization method for solving the inverse kinematics problem...
% Wang & Chen
% IEEE Trans. RA 7(4) 1991 pp 489-
L(1) = Revolute('d', 1, 'a', 0, 'alpha', pi/2, ...
    'I', [0, 0, 0], ...
    'r', [0, 0, 0], ...
    'm', 0, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-180 180]*deg );

L(2) = Revolute('d', 0, 'a', 1, 'alpha', 0, ...
    'I', [0.01, 0.8, 0.8], ...
    'r', [0.5, 0, 0], ...
    'm', 10, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-90 90]*deg );

L(3) = Revolute('d', 0, 'a', 1, 'alpha', 0,  ...
    'I', [0.01, 0.8, 0.8], ...
    'r', [0.5, 0, 0], ...
    'm', 10, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-90 90]*deg );


p560 = SerialLink(L, 'name', 'Puma 560', 'gravity', [0 -9.81 0]);

%
% some useful poses
%
home_pos = [0 0 -pi/2]; % zero angles, L shaped pose
hold_pos = [0 0 0]; % ready pose, arm up

p560.payload(1, [0 0 0]);

jm_home_pos = p560.jacob0(home_pos);
jm_hold_pos = p560.jacob0(hold_pos);

TAUG_home_pos = p560.gravload(home_pos);
TAUG_hold_pos = p560.gravload(hold_pos);

torque_home_pos = jm_home_pos .* TAUG_home_pos
torque_hold_pos = jm_hold_pos .* TAUG_hold_pos


figure(1)
p560.plot(home_pos)
p560.teach(hold_pos)



clear L