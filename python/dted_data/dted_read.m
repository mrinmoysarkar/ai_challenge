function [Z, latitude, longitude, header] = dted_read(filename)
%DTED_READ Read Digital Terrain Elevation Data (DTED) files
%
%   [Z, LAT, LON] = DTED_READ(FILENAME) returns elevation data (Z) in the 
%   specified DTED file as a regular data grid with elevations in meters.  
%   LAT and LON provide 1D arrays with latitides and longitudes (in   
%   degrees) coresponding to columns and rows of the elevation grid.
%
%   DTED files usually follow specific filename and directory name format
%   ending with for example with "\e003\n15.dt0". If file is in the proper
%   format than additional verifications are performed to ensure
%   correctness. 
%
%   [Z, LAT, LON, HEADER] = DTED_READ(FILENAME) also returns HEADER struct with
%   metadata fields extracted from the header. Only minimal amount of
%   metadata is read.
%
% Author:
%    Jarek Tuszynski (jaroslaw.w.tuszynski@leidos.com)
%
% License: BSD license (http://en.wikipedia.org/wiki/BSD_licenses)
%
% Example:
%     figure(1)
%     colormap(jet);
%     [Z, lat, lon]=dted_read('n59.dt1');
%     [X,Y] = meshgrid(lon,lat);
%     surface(X,Y,Z,'EdgeColor','none')
%     zlim([0 5000])
%     view(0, 70)
%     xlabel('longitude')
%     ylabel('latitude')
%     zlabel('elevation')
%% Parse header and read header parameters
Z=[]; latitude=[]; longitude=[]; header=[];
fid = fopen(filename, 'rb');
assert(fid>=0, [filename ' does not exist. \n'])
% read User Header Label (UHL) section
header.sentinel      =            fscanf(fid, '%c', 4);
header.lonOrigin     = str2double(fscanf(fid, '%c', 3)); % longitude origin (in deg)
header.lonOriginEW   =            fscanf(fid, '%c', 5);  % E for East or W for West
header.latOrigin     = str2double(fscanf(fid, '%c', 3)); %  latitude origin (in deg)
header.latOriginNS   =            fscanf(fid, '%c', 5);  % N for North or S for south
header.lonInter      = str2double(fscanf(fid, '%c', 4))/10; % longitude spacing (in arc sec)
header.latInter      = str2double(fscanf(fid, '%c', 4))/10; %  latitude spacing (in arc sec)
header.vert_accuracy = str2double(fscanf(fid, '%c', 4)); % vertical accuracy in meters
header.security_code =            fscanf(fid, '%c', 3);  % U for "unclassified", etc.
header.reference_num =            fscanf(fid, '%c',12);
header.nLon          = str2double(fscanf(fid, '%c', 4)); % number of longitude lines
header.nLat          = str2double(fscanf(fid, '%c', 4)); % number of latitude points
header.lonOriginEW   = lower(header.lonOriginEW(end));
header.latOriginNS   = lower(header.latOriginNS(end));
tileSize = (header.nLon-1)*header.lonInter/3600;
%% If filename matches standard format than varify header parameters
match = regexp(filename,'[ns]\d\d\.dt\d', 'once');
if ~isempty(match) || 0
  %% Parse the filename
  fname = filename;
  fname(fname=='\') = '/';
  
  level = str2double(fname(end)); % DTED0, DTED1, or DTED2
  assert(level>=0 && level<=5, 'DTED level has to be between 0 and 5.')
  latOriginNS = lower(fname(end-6));
  latOrigin   = str2double(fname(end-5:end-4));
  
  if length(fname)>11 && fname(end-7)=='/' && fname(end-12)=='/'
    lonOriginEW = lower(fname(end-11));
    lonOrigin   = str2double(fname(end-10:end-8));
  end
  
  %% Predict header parameters
  % constants depending on DTED level (based on
  % http://fas.org/irp/program/core/dted.htm)
  TileSize = 1./[1, 1, 1, 12, 60, 1200];   % tile size in degrees
  nCOL     = [120, 1200, 3600, 900, 540, 810]; % number of columns in final file
  % latitude/longitude aspect ratio depends on latitude (based on
  % https://en.wikipedia.org/wiki/DTED)
  aspectRatio = [zeros(1,50)+1, zeros(1,20)+2, zeros(1,5)+3, zeros(1,5)+4, zeros(1,11)+6];
  nLat = nCOL(level+1) + 1;
  nLon = nCOL(level+1)/aspectRatio(latOrigin+1) + 1;
  tileSize = TileSize(level+1); % tile size in degrees
  
  %% Verify that predicted and read header parameters match
  assert(nLon==header.nLon, 'Unexpected number of longitude lines');
  assert(nLat==header.nLat, 'Unexpected number of latitude points');
  assert(latOrigin==header.latOrigin, 'Unexpected Longitude of origin');
  assert(latOriginNS==header.latOriginNS, 'Unexpected Hemisphere');
  if exist('lonOrigin','var')
    assert(lonOrigin==header.lonOrigin, 'Unexpected Latitude of origin');
    assert(lonOriginEW==header.lonOriginEW, 'Unexpected Hemisphere');
  end
  assert(tileSize==(header.nLon-1)*header.lonInter/3600, 'Unexpected tile size');
  assert(tileSize==(header.nLat-1)*header.latInter/3600, 'Unexpected tile size');
end
%% read the rest of the DTED file
nHeaderBytes = 3428;
nHeadRows = 4;
nTailRows = 2;
nRow = nLat + nHeadRows + nTailRows;
try
  fseek(fid, nHeaderBytes, 'bof');
  Z = fread(fid, [nRow inf], 'int16', 0, 'ieee-be');
  fclose(fid);
catch
  fprintf('Error reading %s\n',filename)
  return
end
%% Elevation data cleanup
Z = Z(nHeadRows + (1:header.nLat), :);   % Trim header and tail rows
Z(Z<0) = -((Z(Z<0) + 32767) + 1); % Correct sign-bit encoded negative numbers
Z(Z == -32767) = NaN; % Convert the standard DTED null data value to NaN
assert(all(Z(:)>-1000), 'DTED Elevations below -1,000 m detected')
assert(all(Z(:)<10000), 'DTED Elevations above 10,000 m detected')
%% create latitude and longitude grid
latitude  = header.latOrigin + (0:1/(header.nLat-1):1)*tileSize;
longitude = header.lonOrigin + (0:1/(header.nLon-1):1)*tileSize;
if header.latOriginNS=='s', latitude  = -latitude;  end
if header.lonOriginEW=='w', longitude = -longitude; end
