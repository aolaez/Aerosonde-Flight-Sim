"""
Docstring for ece163.Utilities.Rotations
Author: Aydan Olaez (aolaez@ucsc.edu)
File containing functions for rotation and matrix/attitude conversion
"""
import math
from . import MatrixMath

def euler2DCM(yaw, pitch, roll):
    # Define 321 matrices
    # Rotation about z
    yaw_matrix = [
        [math.cos(yaw), math.sin(yaw), 0],
        [-1 * math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ]

    # Rotation about y
    pitch_matrix = [
        [math.cos(pitch), 0, -1 * math.sin(pitch)],
        [0, 1, 0],
        [math.sin(pitch), 0, math.cos(pitch)]
    ]

    # Rotation about x
    roll_matrix = [
        [1, 0, 0],
        [0, math.cos(roll), math.sin(roll)],
        [0, -1 * math.sin(roll), math.cos(roll)]
    ]

    # Create rotation matrix
    R_yz = MatrixMath.multiply(pitch_matrix, yaw_matrix)
    rotation_matrix = MatrixMath.multiply(roll_matrix, R_yz)

    return rotation_matrix



def dcm2Euler(dcm):
    # Calculate pitch, needed to clamp bc of 1.00001 error
    pitch = -1 * math.asin(max(-1.0, min(1.0, dcm[0][2])))

    # For singularities, take all valus outside of 1, -1 bounds not just those ==
    if (abs(dcm[0][2])) >= 1:
        if (dcm[0][2] < 0):
            yaw = 0
            roll = math.atan2(dcm[1][0], dcm[1][1])
        else:
            yaw = 0
            roll = math.atan2(-1 * dcm[1][0], dcm[1][1])
        # special case
        if (math.isclose(abs(yaw), math.pi)):
            roll = 0.0


        return yaw, pitch, roll

    # yaw and roll calculations when no singularity
    yaw  = math.atan2(dcm[0][1], dcm[0][0])
    roll = math.atan2(dcm[1][2], dcm[2][2])

    if (math.isclose(abs(yaw), math.pi)):
        yaw = 0.0

    return yaw, pitch, roll

def ned2enu(points):
    rotation_matrix = [
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ]
    # Treat input points as columns
    col_points = MatrixMath.transpose(points)
    # Rotate
    enu_transpose = MatrixMath.multiply(rotation_matrix, col_points)
    # Undo transpose
    enu_out = MatrixMath.transpose(enu_transpose)

    return enu_out



