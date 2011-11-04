#Indicates no of color-based regions to be searched for
uint32 no_of_regions
#Indicates number of maximum possible objects in each region
uint32 max_no_of_objects
#Indicates the HSV-Limits configuration filenames for all the regions separated by space
#For example: ./redRoiConfig.cfg
string config_files
#Indicates the labels for each region to be used for broadcasting the transforms for the objects
#separated by space 
#For example: red 
string labels