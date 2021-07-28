import numpy as np
import argparse
import os

parser = argparse.ArgumentParser(description='5-Axis Gcode Generation')
parser.add_argument('toolpath', metavar='INPUT_FILE', help='Input toolpath file path')
args = parser.parse_args()

output_file = os.path.splitext(args.toolpath)[0] + ".gcode"

# Read toolpath from input file
toolpath = np.genfromtxt(args.toolpath)
num = len(toolpath)

# B axis rotate 360 degree
angles_total = np.linspace(0, 360, num, endpoint=False)
z_axis = np.array([0., 0., 1.])

file = open(output_file, 'w')
file.write("G21         ; Set units to mm\n")
file.write("G90         ; Absolute positioning\n")
file.write("M4 S0       ; Enable Laser (0 power)\n")
file.write("\n")

# Parameters
A_axis_offset = 92  # Machine home position to vertical z_axis offset
feed_rate = 500 # What is the best Feedrate for laser cutting?
laser_power = 100

rotary_center_to_rotary_top = 125  # mm
rotary_center_to_machine_x = 103.5
rotary_center_to_machine_y = 212.5
rotary_center_to_machine_z = 211.5
laser_focal_len = 23.0

def inv_kins(th1, th2, p):
    """
    Inverse kinematics of the 5-axis laser systems
    
    Parameters
    ----------
    th1: float
        Rotary A-axis
    th2: float
        Rotary B-axis
    p: numpy array
        point in {W} frame

    Returns
    -------
    : tuple
        Joint values X, Y, Z
    """
    p = np.append(p, 1) # Convert to homogeneous coord
    th1 = np.radians(th1)
    th2 = np.radians(th2)
    c1, s1 = np.cos(th1), np.sin(th1)
    c2, s2 = np.cos(th2), np.sin(th2)
    tf_O_W = np.matmul( np.array([[1, 0, 0, 0],
                            [0, c1, -s1, -s1 * rotary_center_to_rotary_top],
                            [0, s1, c1, c1 * rotary_center_to_rotary_top],
                            [0, 0, 0, 1]]).astype(np.float64),
                        np.array([[c2, -s2, 0, 0],
                            [s2, c2, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]).astype(np.float64) )
    # tf_W_O = np.linalg.inv(tf_O_W)

    tf_L_O = np.empty((4,4))
    tf_L_O[:3,:3] = np.array([[1, 0, 0],
                              [0, -1, 0],
                              [0, 0, -1]])  # Rotate about x-axis by 180 degree
    tf_L_O[:3,3] = np.array([-rotary_center_to_machine_x,
                            -rotary_center_to_machine_y,
                            rotary_center_to_machine_z-laser_focal_len])
    tf_L_O[3,:] = [0, 0, 0, 1]
    # tf_O_L = np.linalg.inv(tf_L_O)

    # Transform the point from {W} to {L}
    tf_L_W = np.matmul(tf_L_O, tf_O_W)
    p_in_L = tf_L_W.dot(p)

    return p_in_L[:3]
    
XYZAB_start = None

for n, th_2 in enumerate(angles_total):
    point_in_W = toolpath[n,:3]
    vec_in_W = toolpath[n,3:]

    # 1. Compute the A-axis value
    vec_unit = vec_in_W / np.linalg.norm(vec_in_W)
    th_1 = np.arccos(np.dot(vec_unit,z_axis))
    th_1 = 90 - np.degrees(np.pi/2 - th_1)

    # 2. Rotating along {W}_Z_axis clockwise
    th_2 = -th_2

    # 3. Get the X, Y, Z -axes joint values, need to invert Z
    XYZ = inv_kins(th_1, th_2, point_in_W)
    XYZ[-1] = -XYZ[-1]

    # 4. Append the A, B -axes values
    XYZAB = np.append(XYZ, [th_1 - A_axis_offset, th_2])

    if n == 0:
        # Save the starting point to add at the end to close the toolpath
        XYZAB_start = XYZAB
        line = "G0 X%.3f  Y%.3f  Z%.3f A%.3f B%.3f\n" %(XYZAB[0], XYZAB[1], XYZAB[2], XYZAB[3], XYZAB[4])
        file.write(line)
        line = "G1 F" + str(feed_rate) + "    ; Feed rate\n"
        file.write(line)
        line = "S" + str(laser_power) + "\n"
        file.write(line)
    else:
        line = "X%.3f  Y%.3f  Z%.3f A%.3f B%.3f\n" %(XYZAB[0], XYZAB[1], XYZAB[2], XYZAB[3], XYZAB[4])
        file.write(line)

        if n == num-1:
            line = "X%.3f  Y%.3f  Z%.3f A%.3f B%.3f\n" %(XYZAB_start[0], XYZAB_start[1], XYZAB_start[2], XYZAB_start[3], -360)
            file.write(line)

file.write("M5          ; Disable Laser\n")
file.write("G0 Y-280 Z0 A-90")

# Close the file after writing
file.close()