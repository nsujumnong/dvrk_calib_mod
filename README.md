# dvrk_calib_mod
Modified dvrk calibration and mocap data

# Description
This is a part of an unfinished augmented reality implementation for dVRK project.
This repository contains the offline calibration code for dVRK PSM through motion capture
The code is the modification of Radian Azhar's (radianag) dvrk calibration project.
For more information regarding the equipment set up, please visit:
https://github.com/radianag/dvrk_calibrate

# Dependencies
- Python
- Python libraries
  - CloudPickle
  - Pandas

# Equipment
- Motion Capture System (with software that convert data to .csv file)
- dVRK system

# Installation
Only the python libraries in the dependencies are needed to be installed. And then you can run the
python file normally
```
python dvrk_offline_calibration.py [motioncapture_data_file.csv] [output_file_name.csv]
```

# Example
```
python dvrk_offline_calibration.py psm2_mocab.csv psm2_calibrated.csv
```
which will return the calibrated psm2 data as csv file

# Result
Once the calibration is calculated, the program will return the RCM frame position and orientation along with 
corresponding orientation of the rotation frame and save as a csv file

