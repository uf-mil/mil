when replaceing the WMM.COF file, make sure to convert it to unix style line ends.

Usually the file you can down load from https://www.ngdc.noaa.gov/geomag/WMM/soft.shtml
will come with carriage return characters (for windows users).
These need to be eliminated with `dos2unix WMM.COF` before the mangetometer code will work.
