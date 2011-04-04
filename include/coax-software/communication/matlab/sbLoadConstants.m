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

SB_TIMEOUT_NONE   = 65535;


SB_OA_NONE        = 00;
SB_OA_HORIZONTAL  = 01;
SB_OA_VERTICAL    = 02;


SB_COM_ONREQUEST  = 00;
SB_COM_CONTINUOUS = 01;

SB_CTRL_NONE      = 00;
SB_CTRL_POS       = 01;
SB_CTRL_REL       = 02;
SB_CTRL_VEL       = 03;
SB_CTRL_FORCE     = 04;
SB_CTRL_MANUAL    = 08;

SB_NAV_STOP       = 00;
SB_NAV_IDLE       = 01;
SB_NAV_TAKEOFF    = 02;
SB_NAV_LAND       = 03;
SB_NAV_HOVER      = 04;
SB_NAV_CTRLLED    = 05;
SB_NAV_SINK       = 06;

disp 'Loaded Heli constants'

