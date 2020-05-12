# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle
from collections import deque
import copy

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):
        tmpwaypoints = deque()
        tmpwaypoints = copy.deepcopy(self.currentPlannedPath.waypoints)
        waypoints_n = self.currentPlannedPath.numberOfWaypoints
        for cells in range(waypoints_n):
            cell = tmpwaypoints.popleft()
            x,y = cell.coords
            if(self.occupancyGrid.grid[x][y] == 1):
                self.controller.stopDrivingToCurrentGoal()
                return
        return

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()


    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):
        #Set the value of parameters
        lambda_b = 0.5
        Lw = 2
        p_b = 0.8

        #Create a list to store the travel cost values
        drivingCost = []

        #Iterate through each aisle and generate a path
        for aisle in Aisle:
            if not aisle == Aisle.B:
                replannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisle)
                replannedCost = replannedPath.travelCost
                drivingCost.append(replannedCost)
            else:
                #Path via aisle B with obstacle
                replannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisle)
                replannedCost = replannedPath.travelCost + (p_b/lambda_b) * Lw
                drivingCost.append(replannedCost)
        
        #Return the aisle with the minimum path cost
        return Aisle(drivingCost.index(min(drivingCost)))


    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.E

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):
        #Set the parameters
        lambda_b = 0.5
        Lw = 2
        currentAisle = self.aisleToDriveDown

        #The variabl to store the value of the cost-to-go of the current path
        remainingCost = 0

        #Iterate through the remaining part of the current path to get the remaining travel cost
        for i in range(len(self.currentPlannedPath.waypoints)):
            if startCellCoords == self.currentPlannedPath.waypoints[i].coords:
                for j in range(i+1,len(self.currentPlannedPath.waypoints)):
                    remainingCost += self.planner.computeLStageAdditiveCost(self.currentPlannedPath.waypoints[j].parent, self.currentPlannedPath.waypoints[j])
                break
        
        #Compute and add cost of waiting to the remaining cost
        waitCost = (1/lambda_b) * Lw 
        remainingCost += waitCost

        #Iterate through the remaining four aisles and compute the costs of re-planning
        #If the re-planning cost is lower than waiting, return false(change the path)
        for aisle in Aisle:
            if not aisle == currentAisle:
                replannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisle)
                replannedCost = replannedPath.travelCost

                if replannedCost < remainingCost:
                    self.aisleToDriveDown = None
                    return False

        return True

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):
        blocked = True

        #As long as any cell of the current planned path is obstucted by the obstacle 
        #keep iterating through the loop and checking
        while blocked:
            blocked = False
            for i in range(len(self.currentPlannedPath.waypoints)):
                x,y = self.currentPlannedPath.waypoints[i].coords
                if(self.occupancyGrid.grid[x][y] == 1):
                    blocked = True
    
    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle

        # Coordinates of the five 'checkpoints' for the five aisles 
        aisleCellCoords = {0:(32,36), 1:(48,36), 2:(64,36), 3:(74,32), 4:(90,32)}

        # Find path from the start to the checkpoint
        pathToAisleFound = self.planner.search(startCellCoords, aisleCellCoords[aisle.value])

        # If we can't reach the goal, give up and return
        if pathToAisleFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d) via Aisle %s", \
                            goalCellCoords[0], goalCellCoords[1], str(aisle))
            return None

        # Extract the path from the start to the checkpoint inside the aisle
        plannedPathToAisle = self.planner.extractPathToGoal()

        # Find path from the checkpoint to the goal
        pathToGoalFound = self.planner.search(aisleCellCoords[aisle.value], goalCellCoords)    

        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d) via Aisle %s", \
                            goalCellCoords[0], goalCellCoords[1], str(aisle))
            return None
        
        # Extract the path from the checkpoint to the goal
        plannedPathToGoal = self.planner.extractPathToGoal()

        # merge the paths and return as one
        plannedPathToAisle.addToEnd(plannedPathToGoal)

        return plannedPathToAisle

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False
            
            # Print the whole merged path in the search grid
            self.planner.searchGridDrawer.start = self.planner.searchGrid.getCellFromCoords(startCellCoords)
            self.planner.searchGridDrawer.drawStartAndGoalGraphics()
            self.planner.searchGridDrawer.drawPathGraphics(self.currentPlannedPath)
            self.planner.searchGridDrawer.window.update()
            self.planner.searchGridDrawer.window.flush()

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)

            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
