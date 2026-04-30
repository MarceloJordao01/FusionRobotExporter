# -*- coding: utf-8 -*-
"""
Transform utilities for SDF exporter
Based on FusionSDF by andreasBihlmaier
"""

import math
from typing import List


class Transform:
    def __init__(self, translation: List[float] = None, rotation: List[float] = None, matrix: List[List[float]] = None):
        if matrix is not None:
            self.matrix = matrix
        else:
            self.matrix = [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
            if translation is not None:
                self.set_translation(translation)
            if rotation is not None:
                self.set_rotation_rpy(*rotation)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Transform):
            return False
        for i in range(4):
            for j in range(4):
                if not math.isclose(self.matrix[i][j], other.matrix[i][j], abs_tol=1e-6):
                    return False
        return True

    def __mul__(self, other):
        if not isinstance(other, Transform):
            raise TypeError("Multiplication is only supported between Transform instances")
        return Transform(matrix=self.matrix_multiply(self.matrix, other.matrix))

    def inverse(self):
        R = [row[:3] for row in self.matrix[:3]]
        t = [row[3] for row in self.matrix[:3]]

        R_inv = [[R[j][i] for j in range(3)] for i in range(3)]
        t_inv_mat = self.matrix_multiply(R_inv, [[-t[i]] for i in range(3)])

        inv_matrix = [R_inv[i] + t_inv_mat[i] for i in range(3)]
        inv_matrix.append([0, 0, 0, 1])

        return Transform(matrix=inv_matrix)

    def get_translation(self) -> List[float]:
        return [self.matrix[i][3] for i in range(3)]

    def set_translation(self, translation):
        for i in range(3):
            self.matrix[i][3] = translation[i]

    def get_rotation_rpy(self) -> List[float]:
        R = [row[:3] for row in self.matrix[:3]]

        cos_pitch = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
        if cos_pitch < 1e-6:
            pitch = math.asin(-R[2][0])
            yaw = 0
            if pitch > 0:
                roll = math.atan2(R[0][1], R[1][1])
            else:
                roll = math.atan2(-R[0][1], R[1][1])
        else:
            pitch = math.atan2(-R[2][0], cos_pitch)
            yaw = math.atan2(R[1][0], R[0][0])
            roll = math.atan2(R[2][1], R[2][2])

        return [roll, pitch, yaw]

    def set_rotation_rpy(self, roll, pitch, yaw):
        Rx = [
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ]
        Ry = [
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ]
        Rz = [
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ]
        R = self.matrix_multiply(self.matrix_multiply(Rz, Ry), Rx)
        for i in range(3):
            for j in range(3):
                self.matrix[i][j] = R[i][j]

    @staticmethod
    def matrix_multiply(A, B):
        return [
            [
                sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))
            ] for i in range(len(A))
        ]

    def __str__(self):
        return '\n'.join([' '.join([f'{item: .2f}' for item in row]) for row in self.matrix])
