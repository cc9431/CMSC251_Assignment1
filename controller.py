'''Model-Based reactive agent style scribbler controller designed to seek out an orange pylon and push it into a wall'''
# Charles Calder <cc9431@bard.edu>
# HW Assignment 1
# September 7th 2017

#from myro import *
#from random import random
#from math import *

class Controller(object):

    def __init__(self, configureBlobbing=False):
        '''Create controller for object-finding robot.'''

        if configureBlobbing:
            # Set the values for detecting orange pylon and avoiding obstacles
            configureBlob(0, 200, 0, 140, 160, 254)
            setIRPower(160)
        else:
            pass
        self.win = takePicture()

        # init all behaviors and put them into a list
        self.message = "empty"
        self.goalStateBehavior = GoalStateReached()
        self.pushBehavior = PylonPush()
        self.avoidBehavior = Avoid()
        self.moveToPylonBehavior = MoveTowardsPylon()
        self.wanderBehavior = Wander()

        # Order in list sets the heirarchy of behaviors
        self.behaviors = [
            # Highest priority, the goal is most important
            self.goalStateBehavior,
            # Higher than avoid behavior so the robot doesn't turn away
            self.pushBehavior,
            # Higher than move-towards behavior so robot is aware of obstacles during pylon tracking
            self.avoidBehavior,
            # Higher than wander behavior because wander is only relevant while we havent seen the pylon
            self.moveToPylonBehavior,
            # Only when there is nothing else to do
            self.wanderBehavior
        ]

    def arbitrate(self):
        '''Decide which behavior, in order of priority, the robot will execute.'''
        for behavior in self.behaviors:
            wantToRun = behavior.check()
            if wantToRun:
                behavior.run()
                return # No other behavior runs

    def run(self):
        '''Running the model-based reactive agent.'''
        setForwardness('fluke-forward')
        # Run a 30 second timer 6 times, I do this to keep track of how much time has passed
        for times in xrange(8):
            for seconds in timer(30):
                self.arbitrate()
            print(str((times + 1) * 30) + " Seconds passed.")
        stop() # Stop the robot when complete

####################################################################################################
####################################################################################################
class Behavior(object):
    '''High level class for all behaviors.  Any behavior is a subclass of Behavior.'''
    NO_ACTION = 0

    def __init__(self):
        # Current state of the behavior. Governs how it will respond to percepts.
        self.state = None

    def check(self):
        '''Return True if this behavior wants to execute, Return False if it does not.'''
        return False

    def run(self):
        '''Execute whatever this behavior does.'''
        return

####################################################################################################
####################################################################################################
class GoalStateReached(Behavior):
    '''If the robot detects a high amount of orange,
    and getStall returns true, celebrate your victory.'''
    GOAL = 1
    THRESHOLD = 5000 # How many pixels need to be considered before the behavior is engaged

    def __init__(self):
        '''Initializer for the Goal State behavior.'''
        self.state = GoalStateReached.NO_ACTION

    def check(self):
        '''Check if the goal has been reached.'''
        # Get the relevant info for evaluating the goal status of the current state
        num, x, y = getBlob()
        wallStall = (getStall() == 1)
        if wallStall and num > GoalStateReached.THRESHOLD:
            self.state = self.GOAL
            return True
        else:
            self.state = GoalStateReached.NO_ACTION
            return False

    def run(self):
        '''If goal state has been reached, stop pushing pylon and celebrate.'''
        print ('goal reached')
        stop()
        beep(1, 400)
        turnLeft(1, 5)

####################################################################################################
class PylonPush(Behavior):
    '''If the pylon is right in front of the robot, then
    push the pylon forward, else center pylon in camera view'''
    PUSH_LEFT = -1
    PUSH_RIGHT = 1
    PUSH_STRAIGHT = 3
    PUSH_SPEED = 0.8
    TSPEED = 0.1
    THRESHOLD = 5000 # How many pixels need to be considered before the behavior is engaged

    def __init__(self):
        '''Initializer for the Pylon push behavior'''
        self.state = PylonPush.NO_ACTION
        self.lspeed = PylonPush.PUSH_SPEED
        self.rspeed = PylonPush.PUSH_SPEED
        self.imageWidth = 427 # Set the image width for normalization purposes

    def check(self):
        '''Check if pylon is directly in front of robot, and to which side it needs to drive.'''
        num, x, y = getBlob()
        blob_location = x/self.imageWidth # Normalize blob center within the image size

        # Rotate towards pylon if it is too far to either side otherwise push straight
        if num >= PylonPush.THRESHOLD:
            if blob_location < 0.35:
                self.state = PylonPush.PUSH_LEFT
            elif blob_location > 0.65:
                self.state = PylonPush.PUSH_RIGHT
            else:
                self.state = PylonPush.PUSH_STRAIGHT
            return True
        else:
            self.state = PylonPush.NO_ACTION
            return False

    def run(self):
        '''Push the pylon towards the wall.'''
        print ('push!')

        # Slowly rotate towards pylon or push straight ahead depending on state
        state = int(self.state)
        if state != PylonPush.PUSH_STRAIGHT:
            self.lspeed = PylonPush.TSPEED * state
            self.rspeed = PylonPush.TSPEED * -state
        else:
            self.lspeed = PylonPush.PUSH_SPEED
            self.rspeed = PylonPush.PUSH_SPEED

        motors(self.lspeed, self.rspeed)

####################################################################################################
class Avoid(Behavior):
    '''Behavior to avoid obstacles. Always turns left.'''
    TURN_LEFT = 1
    BACKUP_SPEED = 0.5
    BACKUP_DUR = 0.5
    OBSTACLE_THRESH = 1750 # How close an obstacle needs to be before the behavior is engaged

    def __init__(self):
        '''Initializer for the Avoid behavior.'''
        self.state = Avoid.NO_ACTION
        self.turnspeed = 0.6
        self.turndur = 0.8

    def check(self):
        '''See if there are any obstacles. If so turn left.

        I have found that getting two readings one after
        the other increases the chance of seeing an obstacle.'''
        L, C, R = getObstacle()
        avoid_pls = (L + C + R)/3.0 > Avoid.OBSTACLE_THRESH
        L2, C2, R2 = getObstacle()
        avoid_pls2 = (L2 + C2 + R2)/3.0 > Avoid.OBSTACLE_THRESH
        wallStall = (getStall() == 1)

        # Check multiple readings to see if obstacle is there or if robot is stalled
        if wallStall or avoid_pls or avoid_pls2:
            if wallStall: print('stall')
            self.state = Avoid.TURN_LEFT
            return True
        else:
            self.state = Avoid.NO_ACTION
            return False


    def run(self):
        '''See if there are any obstacles. If so turn left.'''
        # Back up, then turn left.
        print ('Avoid: turning left')
        backward(Avoid.BACKUP_SPEED, Avoid.BACKUP_DUR)
        turnLeft(self.turnspeed, self.turndur)

####################################################################################################
class MoveTowardsPylon(Behavior):
    '''If we have found the pylon in our view, center it in our camera view or move towards it.'''
    ATTACK_LEFT = -1
    ATTACK_RIGHT = 1
    ATTACK_STRAIGHT = 3
    ATTACK_SPEED = 0.6
    TSPEED = 0.2
    THRESHOLD = 2000 # How many pixels need to be considered before the behavior is engaged

    def __init__(self):
        '''Initializer for the Pylon attack behavior'''
        self.state = MoveTowardsPylon.NO_ACTION
        self.lspeed = MoveTowardsPylon.ATTACK_SPEED
        self.rspeed = MoveTowardsPylon.ATTACK_SPEED
        self.imageWidth = 427 # Set the image width for normalization purposes

    def check(self):
        '''Check if is in robot's view, and to which direction it needs to drive'''
        num, x, y = getBlob()
        blob_location = x/self.imageWidth

        # Rotate towards pylon if it is too far to either side, otherwise push straight
        if num >= MoveTowardsPylon.THRESHOLD:
            if blob_location < 0.35:
                self.state = MoveTowardsPylon.ATTACK_LEFT
            elif blob_location > 0.65:
                self.state = MoveTowardsPylon.ATTACK_RIGHT
            else:
                self.state = MoveTowardsPylon.ATTACK_STRAIGHT
            return True
        else:
            self.state = MoveTowardsPylon.NO_ACTION
            return False

    def run(self):
        '''If pylon is in view, drive towards it while adjusting for where the blob is on screen'''
        print ('move towards pylon')
        state = int(self.state)
        # Engage behavior, rotate towards pylon or drive directly at it
        if state != MoveTowardsPylon.ATTACK_STRAIGHT:
            self.lspeed = MoveTowardsPylon.TSPEED * state
            self.rspeed = MoveTowardsPylon.TSPEED * -state
        else:
            self.lspeed = MoveTowardsPylon.ATTACK_SPEED
            self.rspeed = MoveTowardsPylon.ATTACK_SPEED

        motors(self.lspeed, self.rspeed)

####################################################################################################
class Wander(Behavior):
    '''Behavior to wander, moves in an outward spiral from start point.'''
    WANDER = 1
    OBSTACLE_THRESH = 1750 # Avoid would pick up the obstacle if it worked perfectly but it doesn't
    L_SPEED = 0.55
    R_SPEED = 0.7

    def __init__(self):
        '''Initializer for the Wander behavior'''
        self.state = Wander.NO_ACTION

    def check(self):
        '''see if there are any possible obstacles.  If not, then wander.'''
        # Check for an obstacle, if there's nothing, wander in a spiral, else nothing
        L, C, R = getObstacle()
        if (L + C + R)/3.0 < Wander.OBSTACLE_THRESH:
            self.state = Wander.WANDER
            return True
        else:
            self.state = Wander.NO_ACTION
            return False

    def run(self):
        '''Drive in an outward spiral'''
        print ('Wander')
        motors(Wander.L_SPEED, Wander.R_SPEED)

####################################################################################################


if __name__ == "__main__":
    control = Controller(True)
    control.run()
