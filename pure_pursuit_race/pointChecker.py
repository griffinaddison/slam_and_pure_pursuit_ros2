##function that takes a coordinate and returns bool
##if the coordinate is valid or not
def pointChecker(x,y):
    if (x < 0 or x > 7) or (y < 0 or y > 7):
        return False
    else:
        return True