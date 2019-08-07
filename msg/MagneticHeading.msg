Header header

# Magnetic sensor heading in degrees
float32 heading
# Magnetic deviation in degrees
float32 mag_dev
# Magnetic deviation direction (E = Easterly, W = Westerly)
string mag_dev_dir
# Magnetic variation in degrees
float32 mag_var
# Magnetic variation direction (E = Easterly, W = Westerly)
string mag_var_dir

# Magnetic heading represented as a quaternion
geometry_msgs/Quaternion quaternion
