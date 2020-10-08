#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")


		# BTTTTTTTTTTTT
		b0 = tuckarm() # from behavior_tree
        b1 = lowHead
        b2 = pickUpCube() # from behavior_tree
        b3 = turnAround
        b4 = gotoTableB
        b5 = placeDownCube() # from behavior_tree
        #b13 = newturnAround
        #b14 = gotoTableA

        # lower head
        lowHead = movehead("down")
        
        # Turn around
        turnAround = pt.composites.Selector(
            name="Go to turn around fallback",
            children=[counter(29, "At Turn around?"), go("Go to Turn around!", 0, -1)]
        )

        # new Turn around
        newturnAround = pt.composites.Selector(
            name="Go to new turn around fallback",
            children=[counter(29, "At new Turn around?"), go("Go to new Turn around!", 0, -1)]
        )

        # Go to table B
        gotoTableB = pt.composites.Selector(
            name="Go to table fallback",
            children=[counter(10, "At table B?"), go("Go to table B!", 1, 0)]
        )

        # Go to table A
        gotoTableA = pt.composites.Selector(
            name="Go to table fallback",
            children=[counter(10, "At table A?"), go("Go to table A!", 1, 0)]
        )



        # become the tree
        tree = RSequence(name="Main sequence",
                         children=[tuckarm(), lowHead, pickUpCube(), turnAround, gotoTableB, placeDownCube()])
                         #children=[b0, b1, b2, b3, b4, b5])
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