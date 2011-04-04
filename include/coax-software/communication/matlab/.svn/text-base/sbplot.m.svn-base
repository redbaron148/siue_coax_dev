% *************************************************************
% 
%  API and Communication library for the Skybotix AG
%  helicopters, and particularly COAX
% 
%  Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
%  Send modification or corrections as patches (diff -Naur)
%  Copyright: Skybotix AG, 2009-2012
% 
% All rights reserved.
% 
% Skybotix API is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% Skybotix API is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU Lesser General Public License
% along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
% 
%  
% *************************************************************
addpath '.'

global conn lplot
conn = SBConnection('localhost',5123);

sbLoadConstants

sl = GetSensorList(conn)

ConfigureTimeout(conn,SB_TIMEOUT_NONE, SB_TIMEOUT_NONE); % no timeout
%ConfigureTimeout(conn,1000, 3000); % 1s timeout on control, 3s timeout on commands
%ConfigureOAMode(conn,SB_OA_NONE); % no obst. avoid
%ConfigureControl(conn, SB_CTRL_NONE, SB_CTRL_NONE, ...
%	SB_CTRL_POS, SB_CTRL_POS, SB_CTRL_VEL ); % roll = pitch = none, yaw = alt = pose, horiz = vel
%
%ConfigureComm(conn, SB_COM_ONREQUEST); % state on request
SetNavMode(conn, SB_NAV_CTRLLED, 30.0); % request and wait controlled mode
disp 'Heli is controlled'

lplot = [];

disp 'Rotation'
for i=1:100
	SetControl(conn,0,0,sin(2*pi*i/100)*2.0, 0.5);
	S = ReadState(conn,1);
	% Set control roll,pitch: unused, yaw=0, alt=1.0, velocity on a circle
	lplot = [lplot;S.hranges];
	plot(lplot);
end


Disconnect(conn);
clear conn;

