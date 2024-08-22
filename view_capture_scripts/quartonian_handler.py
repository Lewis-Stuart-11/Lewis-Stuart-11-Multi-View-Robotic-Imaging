from math import sqrt, cos, sin
from geometry_msgs.msg import Point, Quaternion
import numpy as np

# Handles all logic vector and quartonian calculations for view capturing
class QuartonianHandler():

    # Calculates vector length
    def Length(self, vector: Point):
        return sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z))

    # Normalises vector length
    def Normalize(self, vector: Point) -> Point:
        vectorLength = self.Length(vector)

        if vectorLength <= 0:
            return vector

        vector.x /= vectorLength
        vector.y /= vectorLength
        vector.z /= vectorLength

        return vector

    # Calculates the cross product between two vectors
    def Cross(self, vector1: Point, vector2: Point) -> Point:
        result = Point()

        result.x = (vector1.y * vector2.z) - (vector1.z * vector2.y)
        result.y = (vector1.z * vector2.x) - (vector1.x * vector2.z)
        result.z = (vector1.x * vector2.y) - (vector1.y * vector2.x)

        return result

    # Calculates the subtraction of two vectors
    def SubtractVectors(self, vector1: Point, vector2: Point) -> Point:
        result = Point()

        result.x = vector1.x - vector2.x
        result.y = vector1.y - vector2.y
        result.z = vector1.z - vector2.z

        return result

    # Calculates the addition of two vectors
    def AddVectors(self, vector1: Point, vector2: Point) -> Point:
        result = Point()

        result.x = vector1.x + vector2.x
        result.y = vector1.y + vector2.y
        result.z = vector1.z + vector2.z

        return result
    
    # Calculates the addition of two vectors
    def MultipleVectorByCoefficient(self, vector1: Point, value: float) -> Point:
        result = vector1

        result.x *= value
        result.y *= value
        result.z *= value

        return result

    def ArePointsEqual(self, vector1: Point, vector2: Point):
        if round(vector1.x, 10) != round(vector2.x, 10):
            return False
        if round(vector1.y, 10) != round(vector2.y, 10):
            return False
        if round(vector1.z, 10) != round(vector2.z, 10):
            return False
        return True
        

    # Calculates the quartonian for rotating one vector to point towards another vector
    def QuaternionLookRotation(self, forward: Point, up: Point) -> Quaternion:
        #forward = self.Normalize(forward)
    
        vector = self.Normalize(forward)
        vector2 = self.Normalize(self.Cross(up, vector))

        vector3 = self.Cross(vector, vector2)
        m00 = vector2.x
        m01 = vector2.y
        m02 = vector2.z
        m10 = vector3.x
        m11 = vector3.y
        m12 = vector3.z
        m20 = vector.x
        m21 = vector.y
        m22 = vector.z
    
        num8 = (m00 + m11) + m22

        quaternion = Quaternion()
        
        if num8 > 0.0:
            num = sqrt(num8 + 1.0)
            quaternion.w = num * 0.5
            num = 0.5 / num
            quaternion.x = (m12 - m21) * num
            quaternion.y = (m20 - m02) * num
            quaternion.z = (m01 - m10) * num

            return quaternion
        
        if ((m00 >= m11) and (m00 >= m22)):
            num7 = sqrt(((1.0 + m00) - m11) - m22)
            num4 = 0.5 / num7
            quaternion.x = 0.5 * num7
            quaternion.y = (m01 + m10) * num4
            quaternion.z = (m02 + m20) * num4
            quaternion.w = (m12 - m21) * num4

            """if abs(vector2.x) == 0 and abs(vector2.y) == 0 and abs(vector2.z) == 0:
                quaternion.x = 0.9999
                quaternion.y = 0.0
                quaternion.z = 0.0
                quaternion.w = 0.0001"""

            return quaternion
        
        if m11 > m22:
            num6 = sqrt(((1.0 + m11) - m00) - m22)
            num3 = 0.5 / num6
            quaternion.x = (m10+ m01) * num3
            quaternion.y = 0.5 * num6
            quaternion.z = (m21 + m12) * num3
            quaternion.w = (m20 - m02) * num3

            return quaternion; 
        
        num5 = sqrt(((1.0 + m22) - m00) - m11);
        num2 = 0.5 / num5
        quaternion.x = (m20 + m02) * num2
        quaternion.y = (m21 + m12) * num2
        quaternion.z = 0.5 * num5
        quaternion.w = (m01 - m10) * num2

        return quaternion
    

    # Converts a quartonian to a rotation matrix
    def convert_quart_to_rotation_matrix(self, q) -> list:

        qx = float(q[0])
        qy = float(q[1])
        qz = float(q[2])
        qw = float(q[3])

        sqw = qw*qw
        sqx = qx*qx
        sqy = qy*qy
        sqz = qz*qz

        invs = 1 / (sqx + sqy + sqz + sqw)
        m00 = ( sqx - sqy - sqz + sqw)*invs 
        m11 = (-sqx + sqy - sqz + sqw)*invs 
        m22 = (-sqx - sqy + sqz + sqw)*invs 
            
        tmp1 = qx*qy
        tmp2 = qz*qw
        m10 = 2.0 * (tmp1 + tmp2)*invs 
        m01 = 2.0 * (tmp1 - tmp2)*invs 
            
        tmp1 = qx*qz
        tmp2 = qy*qw
        m20 = 2.0 * (tmp1 - tmp2)*invs 
        m02 = 2.0 * (tmp1 + tmp2)*invs 
        tmp1 = qy*qz
        tmp2 = qx*qw
        m21 = 2.0 * (tmp1 + tmp2)*invs 
        m12 = 2.0 * (tmp1 - tmp2)*invs 
        
        r = np.array([
            [m00, m01, m02],
            [m10, m11, m12],
            [m20, m21, m22],
        ])

        return r
    
    def convert_euler_to_rotation_matrix(self, rot):
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         cos(rot[0]), -sin(rot[0]) ],
                        [0,         sin(rot[0]), cos(rot[0])  ]
                        ])
    
        R_y = np.array([[cos(rot[1]),    0,      sin(rot[1])  ],
                        [0,                     1,      0                   ],
                        [-sin(rot[1]),   0,      cos(rot[1])  ]
                        ])
    
        R_z = np.array([[cos(rot[2]),    -sin(rot[2]),    0],
                        [sin(rot[2]),    cos(rot[2]),     0],
                        [0,                     0,                      1]
                        ])
    
        return np.dot(R_z, np.dot( R_y, R_x ))


    # Calculates a transformation matrix from a 3D vector translation and quartonian rotation
    def get_tranformation_matrix_from_transform(self, trans: list, rot: Quaternion) -> list:
        
        transformation_matrix = np.identity(4)

        rotation_matrix = self.convert_quart_to_rotation_matrix(rot)

        for i in range(3):
            for j in range(3):
                transformation_matrix[i][j] = rotation_matrix[i][j]

            transformation_matrix[i][3] = trans[i]
        
        return transformation_matrix