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

global conn

%conn = SBConnection('localhost',5123);
conn = SBConnection('192.168.1.52',5123);

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

S = ReadState(conn,1);

disp 'Checking take-off'
while S.zrange < 0.4
	SetControl(conn,0,0,0, 0.5);
	S = ReadState(conn,1);
end

disp 'Rotation'
for i=1:100
	SetControl(conn,0,0,sin(2*pi*i/100)*2.0, 0.5);
	S = ReadState(conn,1);
end

disp 'Pitch'
for i=1:100
	SetControl(conn,0,sin(2*pi*i/100)*0.05,0.0, 0.5);
	S = ReadState(conn,1);
end

disp 'Roll'
for i=1:100
	SetControl(conn,sin(2*pi*i/100)*0.05,0,0.0, 0.5);
	S = ReadState(conn,1);
end


SetNavMode(conn, SB_NAV_STOP, 30.0); % request and wait controlled mode

% cleanup, not really necessary
disp 'Cleanup'
Disconnect(conn);
clear conn;
