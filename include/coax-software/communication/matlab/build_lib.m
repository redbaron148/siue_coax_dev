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
if isunix()
	disp('Assuming linux/unix environment');
	LIBS     ='-L../../lib -lsbcom -lpthread -lrt';
	LIBCLASS ='-L../../../../lib -lsbcom -lpthread -lrt';
	DEFS     ='-DLINUX -DSBC_HAS_PTHREAD -DSBC_HAS_IO -DSBC_HAS_COMM';
else
	disp('Assuming windows environment');
	LIBS     ='-L../build-win32/sbwin/Debug -lsbwin -L"C:\Program Files\Microsoft SDKs\Windows\v6.0A\Lib" -lWs2_32';
	LIBCLASS ='-L../../../build-win32/sbwin/Debug -lsbwin  -L"C:\Program Files\Microsoft SDKs\Windows\v6.0A\Lib" -lWs2_32';
	DEFS     ='-DWIN32 -DSBC_HAS_IO -DSBC_HAS_COMM';
end

CLASSDIR= '@SBConnection/sources/';
UTILITIES='utilities.c';

SOURCES= {'mex_sbConnect', 'mex_GetExampleState'};

SRCCLASS= {'mex_ConfigureAckMode' , ...
	'mex_ConfigureControl' , ... 
	'mex_ConfigureOAMode' , ...
	'mex_ConfigureTimeout' , ...
	'mex_GetNavModeString' , ...
	'mex_GetSensorList' , ...
	'mex_KeepAlive' , ...
	'mex_ReadState' , ...
	'mex_SetControl' , ...
	'mex_SetNavMode' ...
	'mex_ConfigureComm' , ...
	'mex_Disconnect' , ...
};


for i=1:length(SOURCES)
	disp(['compiling ' cell2mat(SOURCES(i))])
	S = cell2mat(SOURCES(i));
	D = strrep(S,'sources','private');
	cmd = ['mex ' DEFS ' -I../include -I@SBConnection/sources -output ' D ...
		' ' S '.c ' CLASSDIR UTILITIES ' ' LIBS];
	%disp(cmd);
	eval(cmd)
end

cd(CLASSDIR);
for i=1:length(SRCCLASS)
	disp(['compiling ' cell2mat(SRCCLASS(i))])
	S = cell2mat(SRCCLASS(i));
	cmd = ['mex ' DEFS ' -I../../../include -I. -output ../private/' S ...
		' ' S '.c ' UTILITIES ' ' LIBCLASS];
	%disp(cmd);
	eval(cmd)
end

cd ../../


