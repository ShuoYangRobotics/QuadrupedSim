function blkStruct = slblocks
%SLBLOCKS Defines the block library for a specific Toolbox or Blockset.
%   SLBLOCKS returns information about a Blockset to Simulink.  The
%   information returned is in the form of a BlocksetStruct with the
%   following fields:
%
%     Name         Name of the Blockset in the Simulink block library
%                  Blocksets & Toolboxes subsystem.
%     OpenFcn      MATLAB expression (function) to call when you
%                  double-click on the block in the Blocksets & Toolboxes
%                  subsystem.
%     MaskDisplay  Optional field that specifies the Mask Display commands
%                  to use for the block in the Blocksets & Toolboxes
%                  subsystem.
%     Browser      Array of Simulink Library Browser structures, described
%                  below.
%
%   The Simulink Library Browser needs to know which libraries in your
%   Blockset it should show, and what names to give them.  To provide
%   this information, define an array of Browser data structures with one
%   array element for each library to display in the Simulink Library
%   Browser.  Each array element has two fields:
%
%     Library      File name of the library (model file) to include in the
%                  Library Browser.
%     Name         Name displayed for the library in the Library Browser
%                  window.  Note that the Name is not required to be the
%                  same as the model file name.
%
%   Example:
%
%      %
%      % Define the BlocksetStruct for the Simulink block libraries
%      % Only simulink_extras shows up in Blocksets & Toolboxes
%      %
%      blkStruct.Name        = ['Simulink' sprintf('\n') 'Extras'];
%      blkStruct.OpenFcn     = 'simulink_extras';
%      blkStruct.MaskDisplay = sprintf('Simulink\nExtras');
%
%      %
%      % Both simulink and simulink_extras show up in the Library Browser.
%      %
%      blkStruct.Browser(1).Library = 'simulink';
%      blkStruct.Browser(1).Name    = 'Simulink';
%      blkStruct.Browser(2).Library = 'simulink_extras';
%      blkStruct.Browser(2).Name    = 'Simulink Extras';
%

%   Copyright 1990-2015 The MathWorks, Inc.

%
% Name of the subsystem which will show up in the Simulink Blocksets
% and Toolboxes subsystem.
%
blkStruct.Name = ['SimMechanics Contact Forces Library'];
%
% The function that will be called when the user double-clicks on
% this icon.
%
blkStruct.OpenFcn = 'Contact_Forces_Lib';

%
% The argument to be set as the Mask Display for the subsystem.  You
% may comment this line out if no specific mask is desired.
% Example:  blkStruct.MaskDisplay = 'plot([0:2*pi],sin([0:2*pi]));';
% No display for Simulink Extras.
%
blkStruct.MaskDisplay = 'SimMechanics Contact Forces Library';

%
% Define the Browser structure array, the first element contains the
% information for the Simulink block library and the second for the
% Simulink Extras block library.
%

%blkStruct.Browser(1).Library = 'simulink';
%blkStruct.Browser(1).Name    = 'Simulink';
%blkStruct.Browser(2).Library = 'Contact_Forces_Lib';
%blkStruct.Browser(2).Name    = 'SimMechanics Contact Forces Library';

%blkStruct.Browser(1).Library = 'Contact_Forces_Lib';
%blkStruct.Browser(1).Name    = 'SimMechanics Contact Forces Library';

Browser(1).Library = 'Contact_Forces_Lib';
Browser(1).Name    = 'SimMechanics Contact Forces Library';

%Browser(1).IsTopLevel = 0;
%Browser(1).IsFlat  = 1;% Is this library "flat" (i.e. no subsystems)?

%Browser(2).Library = 'TNO_dtlib_2';
%Browser(2).Name    = 'Simulink';
%Browser(2).IsFlat  = 0;% Is this library "flat" (i.e. no subsystems)?

blkStruct.Browser = Browser;
clear Browser;


% Define information for model updater
blkStruct.ModelUpdaterMethods.fhDetermineBrokenLinks = @UpdateSimulinkBrokenLinksMappingHelper;
blkStruct.ModelUpdaterMethods.fhSeparatedChecks = @UpdateSimulinkBlocksHelper;

% End of slblocks


