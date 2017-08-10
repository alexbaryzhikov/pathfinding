import math


def distance(p1, p2):
    """ Euclidean distance between p1 and p2. """
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def ang_diff(a1, a2):
    """ Angle difference between a1 and a2 in range [-pi, pi]. """
    a = a2 - a1
    if a > math.pi:
        a -= math.pi * 2
    elif a < -math.pi:
        a += math.pi * 2
    return a


def lines_inter(A, B, C, D):
    """ Intersection point of lines AB and CD. """
    den = (A[0] - B[0]) * (C[1] - D[1]) - (A[1] - B[1]) * (C[0] - D[0])
    if den == 0:  # lines are parallel
        return []
    num_x = (A[0]*B[1] - A[1]*B[0]) * (C[0] - D[0]) - (A[0] - B[0]) * (C[0]*D[1] - C[1]*D[0])
    num_y = (A[0]*B[1] - A[1]*B[0]) * (C[1] - D[1]) - (A[1] - B[1]) * (C[0]*D[1] - C[1]*D[0])
    return [int(round(num_x/den)), int(round(num_y/den))]
