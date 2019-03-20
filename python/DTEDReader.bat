@echo off

REM ---------------------------------------------------------------------------------------
REM Runs the DTED converter that creates a LandSerf file from the given DTED elevation file.
REM To use, open a command prompt in Windows (Start menu -> Run... -> cmd) and then change 
REM to the directory in which this file is  located (e.g. cd "Program files"\landserf220\addons)
REM and type 'DTEDReader dted_file' where dted_file is the DTED file you wish to convert. 
REM Output will be a file with the same name but with a .srf extension. For example
REM DTEDReader c:\data\myRaster.dt1  
REM will create a new file called myRaster.srf in the same directory containing the elevation
REM data.
REM Version 1.0, 24th August, 2005.
REM Copyright Jo Wood, 2005.
REM ---------------------------------------------------------------------------------------

java -classpath .;../jars/landserf220.jar;../jars/utils.jar DTEDReader %1
