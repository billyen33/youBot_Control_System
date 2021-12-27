from math import pi

def testJointLimits(thetalist):
    '''
    lim1 = [-2*pi, 2*pi]
    lim2 = [-2*pi, 2*pi]
    lim3 = [-2*pi, 2*pi]
    lim4 = [-2*pi, 2*pi]
    lim5 = [-2*pi, 2*pi]
    
    lim1 = [-2*pi, 2*pi]
    lim2 = [-2*pi, 2*pi]
    lim3 = [-200, -0.2]
    lim4 = [-200, -0.2]
    lim5 = [-2*pi, 2*pi]
    
    '''
    lim1 = [-pi, pi]
    lim2 = [-2*pi, 2*pi] #max level weird here
    lim3 = [-9, 2*pi]
    lim4 = [-2*pi, 2*pi]
    lim5 = [-pi, pi]
    
    lim_list = [lim1, lim2, lim3, lim4, lim5]
    violated = []
    print(thetalist)
    print(lim_list)
    for ii in range(len(thetalist)):
        if thetalist[ii] > lim_list[ii][0] and thetalist[ii] < lim_list[ii][1]:
            violated.append(0)
        else:
            violated.append(1)
    return violated