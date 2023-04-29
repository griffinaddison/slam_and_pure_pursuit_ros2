##function that takes a coordinate and returns bool
##if the coordinate is valid or not
def pointChecker(x,y):

    ## below are the coefficients a, b, and c for all of the outer track lines L and centerlines CL for standard form ax + by = c

    al1 = -2.2184
    bl1 = 1
    cl1 = 38.0722

    acl1 = -2.2824
    bcl1 = 1
    ccl1 = 36.6968

    al2 = 0.4424
    bl2 = 1
    cl2 = 10.6387

    acl2 = 0.4424
    bcl2 = 1
    ccl2 = 9.8021

    al3 = -2.4726
    bl3 = 1
    cl3 = -24.8661

    acl3 = -2.3683
    bcl3 = 1
    ccl3 = -21.6509

    al4 = 0.4513
    bl4 = 1
    cl4 = -0.4159

    acl4 = 0.4484
    bcl4 = 1
    ccl4 = 0.4599

    ## below is the max acceptable distance from the centerlines
    maxDistance = 0.2

    ## first, check if the point is within the outer track
    withinOutterTrack = (al1*x + bl1*y <= cl1) and (al2*x + bl2*y <= cl2) and (al3*x + bl3*y >= cl3) and (al4*x + bl4*y >= cl4)


    ## now calculate the distances from the point to each of the four centerlines
    d1 = abs(acl1*x + bcl1*y - ccl1) / ((acl1**2 + bcl1**2)**0.5)
    d2 = abs(acl2*x + bcl2*y - ccl2) / ((acl2**2 + bcl2**2)**0.5)
    d3 = abs(acl3*x + bcl3*y - ccl3) / ((acl3**2 + bcl3**2)**0.5)
    d4 = abs(acl4*x + bcl4*y - ccl4) / ((acl4**2 + bcl4**2)**0.5)

    ## now check if the point is within +- 0.25m of any of the centerlines
    withinCenterlines = (d1 <= maxDistance) or (d2 <= maxDistance) or (d3 <= maxDistance) or (d4 <= maxDistance)

    return withinOutterTrack and withinCenterlines

# print(pointChecker(-9.5778, 5.1599))