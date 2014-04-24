# This file representation is used to pass binary data to the RoboEarthDB.
# As the endianess isn't stored, only files with a byte order mark (BOM) or
# an implicitly specified endianess should be transferred.
string name   # file name
int8[] data   # binary data 
