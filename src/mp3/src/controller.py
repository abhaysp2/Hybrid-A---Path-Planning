import math
import rospy
import numpy as np

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive


class bicycleModel():

    def __init__(self):

        self.length = 1.88
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()

        pt1.pose.position.x = -10
        pt1.pose.position.y = -10
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25
        #pt1.twist.angular = 0

        pt2.pose.position.x = 10
        pt2.pose.position.y = 10
        pt2.twist.linear.x = .25
        pt2.twist.linear.y = .25
        #pt2.twist.angular = 0

        pt3.pose.position.x = 0
        pt3.pose.position.y = 0
        pt3.twist.linear.x = .25
        pt3.twist.linear.y = .25
        #pt3.twist.angular = 0

        self.waypointList = []

        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)

        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

    def getModelState(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def rearWheelModel(self, ackermannCmd):
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        ###TODO
        thetaVelocity = self.quaternion_to_euler(currentModelState.pose.orientation.x,currentModelState.pose.orientation.y,currentModelState.pose.orientation.z,currentModelState.pose.orientation.w)[2]
        xVelocity = ackermannCmd.speed * math.cos(thetaVelocity)
        yVelocity = ackermannCmd.speed * math.sin(thetaVelocity)
        ###
        return [xVelocity, yVelocity, ackermannCmd.steering_angle_velocity]

    def rearWheelFeedback(self, currentPose, targetPose):

        #Gain Values
        k1 = 4
        k2 = 15
        k3 = 4

        #give targetVel and targetAngVel
        targetVel = math.sqrt((targetPose.twist.linear.x*targetPose.twist.linear.x) + ((targetPose.twist.linear.y*targetPose.twist.linear.y)))
        targetAngVel = targetPose.twist.angular.z
        #print (targetVel, targetAngVel)
        ###TODO
        ## TODO: Compute Error to current waypoint
        targetYaw = self.quaternion_to_euler(targetPose.pose.orientation.x,targetPose.pose.orientation.y,targetPose.pose.orientation.z,targetPose.pose.orientation.w)[2]
        targetX = targetPose.pose.position.x
        targetY = targetPose.pose.position.y

        currentYaw = self.quaternion_to_euler(currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z,currentPose.pose.orientation.w)[2]
        currentX = currentPose.pose.position.x
        currentY = currentPose.pose.position.y

        a = np.array([[math.cos(currentYaw),math.sin(currentYaw),0],[-math.sin(currentYaw),math.cos(currentYaw),0],[0,0,1]])
        b = np.array([[targetX-currentX],[targetY-currentY],[targetYaw-currentYaw]])
        error_vector = np.dot(a,b) #gives dot product


        ## TODO: Create new AckermannDrive message to return
        currentVel = targetVel * math.cos(error_vector[2]) + k1*error_vector[0]
        currentAngVel = targetAngVel + (targetVel)*(k2*error_vector[1] + k3*math.sin(error_vector[2]))
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = currentVel
        newAckermannCmd.steering_angle_velocity = currentAngVel

        ###
        #print(newAckermannCmd)
        return newAckermannCmd

    def setModelState(self, currState, targetState):

        #control = AckermannDrive()
        control = self.rearWheelFeedback(currState, targetState)
        values = self.rearWheelModel(control)

        newState = ModelState()
        newState.model_name = 'polaris'
        newState.pose = currState.pose
        newState.twist.linear.x = values[0]
        newState.twist.linear.y = values[1]
        newState.twist.angular.z = values[2]
        self.modelStatePub.publish(newState)

    def quaternion_to_euler(self, x, y, z, w):
        x, y, z, w = float(x), float(y), float(z), float(w)

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def __waypointHandler(self, data):
        self.waypointList.append(data)

    #add a list of points in ModelState
    def addPlanedPath(self, path):
        self.waypointList = path + self.waypointList

    def popNextPoint(self):
        if self.waypointList:
            return self.waypointList.pop(0)
        else:
            return None
