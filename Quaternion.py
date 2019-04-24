import numpy as np
import math


def quaternion4(x, y, z, angle):
    sine = np.sin(np.radians(angle) / 2)
    return np.array([x * sine, y * sine, z * sine, np.cos(np.radians(angle) / 2)])


def quaternion(u, w):
    return quaternion4(u[0], u[1], u[2], w)


def quatMult(r, s):
    return np.append(
        r[3] * s[0:3] + s[3] * r[0:3] + np.cross(r[0:3], s[0:3]),
        r[3] * s[3] - np.inner(r[0:3], s[0:3]),
    )


def conjugate(q):
    return np.append(-q[0:3], q[3])


def encase(q, r):
    return quatMult(quatMult(r, q), conjugate(r))


def encase3(q, r):
    return encase(np.append(q, 0), r)[0:3]


def toMatrix(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    matrix = np.array(
        [
            [
                w ** 2 + x ** 2 - y ** 2 - z ** 2,
                2 * x * y - 2 * w * z,
                2 * x * z + 2 * w * y,
                0,
            ],
            [
                2 * x * y + 2 * w * z,
                w ** 2 - x ** 2 + y ** 2 - z ** 2,
                2 * y * z - 2 * w * x,
                0,
            ],
            [
                2 * x * z - 2 * w * y,
                2 * y * z + 2 * w * x,
                w ** 2 - x ** 2 - y ** 2 + z ** 2,
                0,
            ],
            [0, 0, 0, 1],
        ]
    )
    return matrix


def unitBasisVector(p):
    index = np.argmax(np.abs(p))
    result = np.array([0] * len(p))
    result[index] = np.sign(p[index])
    return result
