ff: found legal plan as follows

step    0: RAMIFICATE
        1: DETECT_CHILDREN
        2: EMOTE ROBOT SOUND_HELLO WIGGLE_ERROR
        3: FOLLOW_CHILD CLOSEST_CHILD
        4: GOTO_VIEW_WAYPOINT
        5: EXAMINE_OBJECT
        6: OBSERVE-TOY_AT_RIGHT_BOX L0 L1 KB_TOY1
        7: RAMIFICATE
        8: EMOTE ROBOT SOUND WIGGLE
        9: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT PICKUP_WAYPOINT
       10: JUMP 2
       11: EMOTE ROBOT NO_SOUND WIGGLE
       12: PICKUP_OBJECT ROBOT OBJECT_WAYPOINT KENNY_WAYPOINT LATEST_OBJECT LATEST_TYPE
       13: GIVE_OBJECT ROBOT KENNY_WAYPOINT LATEST_OBJECT LATEST_TYPE
       14: DETECT_CHILDREN
       15: JUMP 2
     

time spent:    0.07 seconds instantiating 1743 easy, 45 hard action templates
               0.00 seconds reachability analysis, yielding 1141 facts and 148 actions
               0.00 seconds creating final representation with 318 relevant facts
               0.00 seconds building connectivity graph
               1.13 seconds searching, evaluating 11741 states, to a max depth of 22
               1.20 seconds total time

