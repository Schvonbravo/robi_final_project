#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")

        tuckarm1 = tuckarm()
        tuckarm2 = tuckarm()


        # go to Turn around 
        turnAround = pt.composites.Selector(
            name="Go turn around fallback",
            children=[counter(28, "At turn around?"), go("Go to around!", 0, -1)]
        )

        turnAround2 = pt.composites.Selector(
            name="Go turn around fallback",
            children=[counter(15, "At turn around2?"), go("Go to around2!", 0, -1)]
        )

        # go to Table B
        gotoTableB = pt.composites.Selector(
            name="Go to table B fallback",
            children=[counter(9, "At Table B?"), go("Go to Table B!", 1, 0)]
        )

        # go to Table A
        gotoTableA = pt.composites.Selector(
            name="Go to table A fallback",
            children=[counter(4, "At Table A?"), go("Go to Table A!", 1, 0)]
        )

        # back 3 steps
        back3step = pt.composites.Selector(
            name="Go to table B fallback",
            children=[counter(9, "At back 5 step?"), go("Go to back 5 step!", -1, 0)]
        )

        # lower head
        headDown = movehead("down")

        # become the tree
        tree = RSequence(name="Main sequence",
                         children=[tuckarm1, headDown, pickCube(), turnAround, gotoTableB, placeCube(), P_checker(), back3step, tuckarm2, turnAround2, gotoTableA])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()