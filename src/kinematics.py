import numpy as np
from quadruped import*

# q = np.zeros((3,1))

def computeLegJacobianAndPosition(quad: Quadruped, q, leg: int):
    l1 = quad._abadLinkLength
    l2 = quad._hipLinkLength
    l3 = quad._kneeLinkLength
    l4 = quad._kneeLinkY_offset
    sideSign = quad.getSideSign(leg)

    s1 = np.sin(q[0])
    s2 = np.sin(q[1])
    s3 = np.sin(q[2])

    c1 = np.cos(q[0])
    c2 = np.cos(q[1])
    c3 = np.cos(q[2])

    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3

    J = np.zeros((3,3), dtype=np.double)
    J[0, 0] = 0
    J[0, 1] = l3 * c23 + l2 * c2
    J[0, 2] = l3 * c23
    J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1
    J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2
    J[1, 2] = -l3 * s1 * s23
    J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1
    J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2
    J[2, 2] = l3 * c1 * s23

    p = np.zeros((3,1), dtype=np.double)
    p[0] = l3 * s23 + l2 * s2
    p[1] = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1
    p[2] = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2
    
    return J, p
