#!/bin/bash
# ---------------------------------------------------------------------------------------
# Runs the DTED converter that creates a LandSerf file from the given DTED elevation file.
# To use, open a shell window and then change to the directory in which this file is located
# (e.g. cd ~/landserf220/addons) and type 'DTEDReader.sh dted_file' where dted_file is the
# DTED file you wish to convert. Output will be a file with the same name but with a .srf 
# extension. For example
# DTEDReader.sh ~/data/myRaster.dt1  
# will create a new file called myRaster.srf in the same directory containing the elevation
# data.
# Version 1.0, 24th August, 2005.
# Copyright Jo Wood, 2005.

# ---------------------------------------------------------------------------------------

java -classpath .;../jars/landserf220.jar;../jars/utils.jar DTEDReader $1