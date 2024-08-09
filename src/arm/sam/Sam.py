import numpy as np
import argparse

def IK(p_end):
    # knowns
    d1 = 77.5
    d2 = 89.5
    d3 = 74.5
    d4 = 276.5
    d_ee = 265
    l2 = 320

    # Rotation matrices
    Rz = np.array([
        [np.cos(p_end[3]), -np.sin(p_end[3]), 0],
        [np.sin(p_end[3]), np.cos(p_end[3]), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(p_end[4]), 0, np.sin(p_end[4])],
        [0, 1, 0],
        [-np.sin(p_end[4]), 0, np.cos(p_end[4])]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(p_end[5]), -np.sin(p_end[5])],
        [0, np.sin(p_end[5]), np.cos(p_end[5])]
    ])

    R06 = Rz @ Ry @ Rx

    p_x = p_end[0]
    p_y = p_end[1]
    p_z = p_end[2]

    # theta1 calculation
    theta1 = np.arctan2(-p_x, p_y) + np.arctan2(np.sqrt(abs(p_y**2 + p_x**2 - (d2 - d3)**2)), (d2 - d3))
    if abs(theta1) < 0.00872665:
        theta1 = 0

    # a calculation for theta3
    a = (d1**2 - 2*d1*p_z + p_x**2 + p_y**2 + p_z**2 - d2**2 + 2*d2*d3 - d3**2 - d4**2 - l2**2) / (2 * d4 * l2)
    theta3 = np.arctan2(a, np.sqrt(abs(1 - a**2)))
    if abs(theta3) < 0.00872665:
        theta3 = 0

    # Matrix C and IK_s1
    C = np.array([
        [l2 + d4 * np.sin(theta3), d4 * np.cos(theta3)],
        [d4 * np.cos(theta3), -d4 * np.sin(theta3) - l2]
    ])
    
    IK_s1 = np.array([
        p_x * np.cos(theta1) + p_y * np.sin(theta1),
        p_y * np.cos(theta1) - p_x * np.sin(theta1),
        p_z - d1
    ])

    # theta2 calculation
    a = C[1, 0]
    b = C[0, 0]
    c = IK_s1[2]
    d = IK_s1[0]
    theta2 = np.arctan2(a * d - b * c, a * c + b * d)
    if abs(theta2) < 0.00872665:
        theta2 = 0

    # R03 matrix
    R03 = np.array([
        [np.cos(theta1) * np.cos(theta2) * np.cos(theta3) - np.cos(theta1) * np.sin(theta2) * np.sin(theta3), -np.cos(theta1) * np.cos(theta2) * np.sin(theta3) - np.cos(theta1) * np.cos(theta3) * np.sin(theta2), -np.sin(theta1)],
        [np.cos(theta2) * np.cos(theta3) * np.sin(theta1) - np.sin(theta1) * np.sin(theta2) * np.sin(theta3), -np.cos(theta2) * np.sin(theta1) * np.sin(theta3) - np.cos(theta3) * np.sin(theta1) * np.sin(theta2), np.cos(theta1)],
        [-np.cos(theta2) * np.sin(theta3) - np.cos(theta3) * np.sin(theta2), np.sin(theta2) * np.sin(theta3) - np.cos(theta2) * np.cos(theta3), 0]
    ])

    R36 = R03.T @ R06

    r11, r12, r13 = R36[0, :]
    r21, r22, r23 = R36[1, :]
    r31, r32, r33 = R36[2, :]

    # theta6 calculation
    theta6 = np.arctan2(-r22, r21)
    if abs(theta6) < 0.00872665:
        theta6 = 0

    # theta5 calculation
    theta5 = np.arctan2(r21 * np.cos(theta6) - r22 * np.sin(theta6), -r23)
    if abs(theta5) < 0.00872665:
        theta5 = 0

    # theta4 calculation
    theta4 = np.arctan2(-(r12 * np.cos(theta6) + r11 * np.sin(theta6)), r32 * np.cos(theta6) + r31 * np.sin(theta6))
    if abs(theta4) < 0.00872665:
        theta4 = 0

    joints = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    return joints

def main():
    parser = argparse.ArgumentParser(description="Calculate inverse kinematics for a 6-DOF robot.")
    parser.add_argument('p_end', type=float, nargs=6, help="End effector position and orientation (x, y, z, roll, pitch, yaw)")
    args = parser.parse_args()

    p_end = np.array(args.p_end)
    joints = IK(p_end)
    print("Joint angles:", joints)

if __name__ == "__main__":
    main()