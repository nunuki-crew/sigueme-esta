#!/usr/bin/env python3
"""
This is the main script used for the EL5206 Robotics component. This script is
written Python 3 and defines a EL5206_Robot node for you to program. You will
find notes on how the code works, auxiliary functions and some for you to 
program.

Authors: Your dear 2022.Spring.teaching_staff
(Eduardo Jorquera, Ignacio Dassori & Felipe Vergara)
"""
import matplotlib.pyplot as plt
import math
import numpy as np
import rospkg
import rospy
import sys
import tf
import tf2_ros
import time
import yaml
#
import PIL
import cv2
from cv_bridge import CvBridge, CvBridgeError

# from PIL import Image

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

class EL5206_Robot:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('EL5206_Main_Node', anonymous=False)

        # Attributes
        self.bridge = CvBridge()
        self.currentImage = None
        #
        self.robot_frame_id = 'base_footprint'
        self.odom_frame_id  = 'odom'
        self.currentScan =  None
        self.odom_x   = None
        self.odom_y   = None
        self.odom_yaw = None
        self.gt_x     = None
        self.gt_y     = None
        self.gt_yaw   = None
        self.odom_lst = []
        self.gt_lst   = []
        self.poses_to_save = 300 
        self.target_x   = None
        self.target_y   = None
        self.target_yaw = None
        self.path = rospkg.RosPack().get_path('el5206_example')

        # Extra variable to print odometry
        self.odom_i = 0
        self.gtc_i = 0

        # Subscribers
        rospy.Subscriber("/odom",               Odometry,  self.odometryCallback)
        rospy.Subscriber("/ground_truth/state", Odometry,  self.groundTruthCallback)
        rospy.Subscriber("/scan",               LaserScan, self.scanCallback)
        rospy.Subscriber("/target_pose",        Pose2D,    self.poseCallback)
        self.imagesub = rospy.Subscriber("/stalker/camera_link_camera/camera_link_camera/color/image_raw",  Image,   self.image_callback)

        # Publishers
        self.vel_pub = rospy.Publisher('/stalker/cmd_vel', Twist, queue_size=1)

        # Timer
        self.update_timer = rospy.Timer( rospy.Duration(1.0), self.timerCallback )


    """
    Callback functions are executed every time a ROS messaage is received by the
    Subscirbers defined in the __init__ method. You have to be careful with 
    these methods, because they are executed several times a second and should 
    be quick enough to finnish before the next message arrives. 
    
    Be careful with the variables stored in a callback, because these methods 
    can be threaded, they may change some of the variables you are using in the 
    main code.
    """

    #uwu 
    # def imageCallback(self, msg):
    #     self.currentImage = msg
    def image_callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.currentImage = cv_image

    def scanCallback(self, msg):
        """
        Receives a LaserScan message and saves it self.currentScan.
        """
        self.currentScan = msg


    def odometryCallback(self, msg):
        """
        Receives an Odometry message. Uses the auxiliary method odom2Coords()
        to get the (x,y,yaw) coordinates and save the in the self.odom_x, 
        self.odom_y and self.odom_yaw attributes.
        """
        # self.odom_i += 1
        # if self.odom_i%30==0:
            # Print one every 30 msgs
            # print("This is the Odometry message:")

            #Print x,y, quaternion
            # print(msg.pose.pose.position.x)
            # print(msg.pose.pose.position.y)
            # # print(msg.pose.pose.orientation)

            #Obtencion de parametros yaw
            # x = msg.pose.pose.orientation.x
            # y = msg.pose.pose.orientation.y
            # z = msg.pose.pose.orientation.z
            # w = msg.pose.pose.orientation.w
            # print(tf.transformations.euler_from_quaternion([w,x,y,z]))

            #Verificación de odom2Coords
            # print(self.odom2Coords(msg))

        self.odom_x, self.odom_y, self.odom_yaw = self.odom2Coords(msg)
    

    def groundTruthCallback(self, msg):
        """
        Receives an Odometry message. Uses the auxiliary method odom2Coords()
        to get the (x,y,yaw) coordinates ans saves the in the self.gt_x, 
        self.gt_y and self.gt_yaw attributes
        """
        # self.gtc_i += 1
        # if self.gtc_i%30==0:
            # print("This is the GroundTruth message:")

            #Print x,y, quaternion
            # print(msg.pose.pose.position.x)
            # print(msg.pose.pose.position.y)
            # # print(msg.pose.pose.orientation)

            #Obtencion de parametros yaw
            # x = msg.pose.pose.orientation.x
            # y = msg.pose.pose.orientation.y
            # z = msg.pose.pose.orientation.z
            # w = msg.pose.pose.orientation.w
            # print(tf.transformations.euler_from_quaternion([w,x,y,z]))

            #Verificación de odom2Coords
            # print(self.odom2Coords(msg))
        self.gt_x, self.gt_y, self.gt_yaw = self.odom2Coords(msg)


    def poseCallback(self, msg):
        """
        This method is activated whenever a message in the /target_pose topic 
        is published. For the assignments that require a target pose, you can 
        call your assignment functions from here or from the __main__ section
        at the end.

        Hint: in your terminal you can use the following command to send a pose
        of x,y,theta coordinates of 1, 2 and 3 respectively.
        $ rostopic pub /target_pose geometry_msgs/Pose2D '1.0' '2.0' '3.0'
        """
        # START: YOUR CODE HERE
        self.target_x   = msg.x
        self.target_y   = msg.y
        self.target_yaw = msg.theta
            

        # END: YOUR CODE HERE


    def timerCallback(self,event):
        """
        This timer function will save the odometry and Ground Truth values
        for the position in the self.odom_lst and self.gt_lst so you can
        compare them afterwards. It is updated every 1 second for 300 poses 
        (about 5 mins).
        """
        if self.odom_x is not None and self.gt_x is not None and len(self.odom_lst)<self.poses_to_save:
            self.odom_lst.append( (self.odom_x, self.odom_y) )
            self.gt_lst.append( (self.gt_x, self.gt_y) )
                

    """
    Now let's define some auxiliary methods. These are used to solve the 
    problems in the assignments.
    """
    def odom2Coords(self, odom_msg):
        """
        This is an auxiliary method used to get the (x, y, yaw) coordinates from 
        an Odometry message. You can use the cmd "$ rostopic echo -n 1 /odom"
        in the terminal to check out the Odometry message attributes.

        Hint: Check the built-in functions in the tf.transformations package to
        see if there is one that handles the euler-quaternion transformation.
        http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
        """
        # START: YOUR CODE HERE

        #Extracción de parametros del quaternion de orientacion
        x_quat = odom_msg.pose.pose.orientation.x
        y_quat = odom_msg.pose.pose.orientation.y
        z_quat = odom_msg.pose.pose.orientation.z
        w_quat = odom_msg.pose.pose.orientation.w

        #Obtencion de x,y,yaw
        x   = odom_msg.pose.pose.position.x
        y   = odom_msg.pose.pose.position.y
        yaw = tf.transformations.euler_from_quaternion([w_quat,x_quat,y_quat,z_quat])[0]

        # END: YOUR CODE HERE
        return (x, y, yaw)
    
    def saveLaser(self): 
        """
        For the RANSAC experience, it is very common for students to save the
        Laser array and work form home with Jupyter Notebook or Google Colab.
        """
        # Wait for laser to arrive
        while self.currentScan is None:
            pass
        
        ranges = np.array(self.currentScan.ranges)
        a_min  = self.currentScan.angle_min
        a_inc  = self.currentScan.angle_increment
        angles = np.array([a_min + i*a_inc for i in range(len(ranges))])

        array_to_save = np.stack([ranges, angles], axis=1)
        with open(self.path+"/results/laser_ranges.npy", "wb") as f:
            np.save(f, array_to_save)
            print("Saved array of shape (%i, %i)"%array_to_save.shape)
            print("Look it up in the %s/results directory"%self.path)
    
    def plotOdomVsGroundTruth(self, name = "Test"):
        """
        Imports a map image and plots the trajectory of the robot according to 
        the Odometry frame and Gazebo's Ground Truth.
        """
        if len(self.odom_lst)>0:
            img = plt.imread(self.path +'/maps/map.pgm')
            print('Image imported')
            # Import map YAML (This is a text file with information about the map)
            with open(self.path+"/maps/map.yaml", 'r') as stream:
                data       = yaml.safe_load(stream)
                origin     = data['origin']
                resolution = data['resolution']
                height     = img.shape[0]
            
            odom_arr = np.array(self.odom_lst)
            gt_arr   = np.array(self.gt_lst)
            
            odom_x_px = ((odom_arr[:,0] - origin[0])/resolution).astype(int)
            odom_y_px = (height-1+ (origin[1]-odom_arr[:,1])/resolution).astype(int)
            gt_x_px = ((gt_arr[:,0] - origin[0])/resolution).astype(int)
            gt_y_px = (height-1+ (origin[1]-gt_arr[:,1])/resolution).astype(int)
            plt.imshow(img,cmap='gray')
            plt.plot(odom_x_px, odom_y_px, color="red", linewidth=1, label='Odometry')
            plt.plot(gt_x_px, gt_y_px, color="blue", linewidth=1, label='Ground Truth')
            plt.legend()
            plt.title('Trajectory of the Robot, ' + name)
            plt.axis('off')
            plt.savefig(self.path + '/results/' + name + '.png')
                

    def printOdomvsGroundTruth(self):
        """
        Prints the robot odometry and ground truth position/angle.
        """
        if self.odom_x is not None and self.gt_x is not None:
            print("                  Odometry         -        GroundTruth")
            print("(x,y,yaw):  (%6.2f,%6.2f,%6.2f) - (%6.2f,%6.2f,%6.2f)"%(self.odom_x,self.odom_y,self.odom_yaw,self.gt_x,self.gt_y,self.gt_yaw))

    def dance(self, timeout=30):
        """
        Demo function. Moves the robot with the vel_pub Publisher.
        """
        start_time = time.time()
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            # Move forward
            twist_msg = Twist()
            twist_msg.linear.x  = 0.2
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Move backward
            twist_msg = Twist()
            twist_msg.linear.x  = -0.6
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)
            

            # Turn left
            twist_msg = Twist()
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = 0.6
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Turn right
            twist_msg = Twist()
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = -0.2
            self.vel_pub.publish(twist_msg)
            time.sleep(1)
        self.plotOdomVsGroundTruth()

    def asssignment_1(self, timeout = 30):
        # You can use this method to solve the Assignment 1.
        # START: YOUR CODE HERE
        start_time = time.time()
        while time.time() - start_time < timeout and not rospy.is_shutdown():

            #Avanza por 1s (Para pasar como input inicial)
            twist_msg = Twist()
            twist_msg.linear.x  = 0.6
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            self.printOdomvsGroundTruth()  

            #Retrocede y gira por 10s
            twist_msg = Twist()
            twist_msg.linear.x  = -0.6
            twist_msg.angular.z = -0.2
            self.vel_pub.publish(twist_msg)
            time.sleep(10)

            self.printOdomvsGroundTruth()

            #Avanza por 10s
            twist_msg = Twist()
            twist_msg.linear.x  = 0.6
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(10)

            self.printOdomvsGroundTruth()

            #Rota por 10s
            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 1
            self.vel_pub.publish(twist_msg)
            time.sleep(10)

            self.printOdomvsGroundTruth()

        # END: YOUR CODE HERE
        self.printOdomvsGroundTruth()
        self.plotOdomVsGroundTruth()
    
    def asssignment_2(self, approach = 1):
        # You can use this method to solve the Assignment 2.

        time.sleep(0.1)

        while ((self.target_x is None)  or (self.odom_x is None)) and (not rospy.is_shutdown()):

        #Se define la posicion inicial
            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(0.1)

        #Se definen variables a utilizar
        xFinal = self.target_x #Objetivos
        yFinal = self.target_y
        yawFinal = self.target_yaw
        #
        x = self.odom_x #Posicion actual
        y = self.odom_y
        yaw = self.odom_yaw

        #Primer approach
        if approach == 1:
            print("Approach 1")
            name = "approach_1"

            #Se obtiene el angulo de rotacion
            rotate = np.arctan2((yFinal - y),( xFinal - x)) - (np.pi - abs(yaw)) * np.sign(yaw)

            # print(rotate)
            # print(yaw)
            # print(self.gt_yaw)
            # print(yawFinal)

            #Rota
            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0.3 * np.sign(rotate) #El signo indica la direccion
            self.vel_pub.publish(twist_msg)
            time.sleep(abs(rotate)/0.3) #Tiempo proporcional a cuanto debe rotar

            #Se detiene la rotacion
            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            #Se obtiene la distancia al punto
            traslate = np.linalg.norm([(xFinal - x), (yFinal - y)])

            #Traslacion
            twist_msg = Twist()
            twist_msg.linear.x  = 0.3
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(traslate/0.3) #Tiempo proporcional a cuanto debe avanzar
        
        #Segundo approach
        elif approach == 2:
            print("Approach 2")
            name = "approach_2"
            itCount = 0
            while ((x != xFinal) and (y != yFinal)) and (not rospy.is_shutdown()):
                #En cada iteracion se indica cuanto se debe mover
                traslate = np.linalg.norm([(xFinal - x), (yFinal - y)])
                rotate = (np.arctan2((yFinal - y), (xFinal - x))) - (np.pi - abs(yaw)) * np.sign(yaw)

                #print(traslate, rotate, yaw,(np.pi - abs(yaw)) * np.sign(yaw))

                #Se realizan ambos movimientos al mismo tiempo
                twist_msg = Twist()
                twist_msg.linear.x  = 0.1 * (traslate) if traslate < 2 else 0.2 #Si ya roto lo suficiente se da prioridad a traslado
                twist_msg.angular.z = 0.05 * rotate 
                self.vel_pub.publish(twist_msg)
                time.sleep(0.05) 

                #Pausa entre intrucciones
                twist_msg = Twist()
                twist_msg.linear.x  = 0
                twist_msg.angular.z = 0
                self.vel_pub.publish(twist_msg)
                time.sleep(0.01)

                #Actualizacion
                x = self.odom_x
                y = self.odom_y
                yaw = self.odom_yaw
                itCount +=1

                #Si ya itera demasiado o esta muy cerca se detiene
                if itCount>650 or traslate<0.1:
                    break

        #Se detienen las instrucciones
        twist_msg = Twist()
        twist_msg.linear.x  = 0
        twist_msg.angular.z = 0
        self.vel_pub.publish(twist_msg)
        time.sleep(1)

        #Se muestra el resultado y se genera el grafico
        self.printOdomvsGroundTruth()
        self.plotOdomVsGroundTruth(name = name)

    def asssignment_3(self):
        #Se realiza un escaneo laser
        msg = self.currentScan
        self.scanCallback(msg)
        #Se guarda la lectura
        self.saveLaser()

    def asssignment_4(self, Katt = 0.3, Rmax = 4, Krep = 0.0005):
        # You can use this method to solve the Assignment 4.
        # START: YOUR CODE HERE
        itCount = 0
        while ((self.target_x is None)  or (self.odom_x is None)) and (not rospy.is_shutdown()):
        
        #Se define la posicion inicial
            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(0.1)

        #Se definen variables a utilizar
        xFinal = self.target_x #Objetivos
        yFinal = self.target_y
        yawFinal = self.target_yaw
        #
        x = self.odom_x #Posicion actual
        y = self.odom_y
        yaw = self.odom_yaw
        R = np.sqrt( (x-xFinal)**2 + (y-yFinal)**2 )

        while (R > 0.3 and not rospy.is_shutdown()):
            
            #Se leen las posiciones actuales
            x = self.odom_x 
            y = self.odom_y
            yaw = self.odom_yaw
            #Escaneo
            msg = self.currentScan
            self.scanCallback(msg)

            # Wait for laser to arrive
            while self.currentScan is None:
                pass
            
            ranges = np.array(self.currentScan.ranges)
            a_min  = self.currentScan.angle_min
            a_inc  = self.currentScan.angle_increment
            angles = np.array([a_min + i*a_inc for i in range(len(ranges))])

            msgList = np.stack([ranges, angles], axis=1)
            #
            R = np.sqrt( (x-xFinal)**2 + (y-yFinal)**2 )
            print("R = " + str(R))
            #Fuerzas de atracción por eje
            X_att = (xFinal - x) * Katt
            Y_att = (yFinal - y) * Katt
            #Si superan un límite, se reduce la fuerza
            if R > Rmax:
                X_att = X_att * (Rmax/R)
                Y_att = Y_att * (Rmax/R)
                
            F = 0
            S = 0
            

            for scan in msgList:
                F += 1/(scan[0]**2) * np.cos(-scan[1])
                S += 1/(scan[0]**2) * np.sin(-scan[1])
                #Es menos el angulo pues según el manual el angulo se mide horario pero ross scanea anti-horario

            X_rep = Krep * F * np.cos(yaw) - Krep * S * np.sin(yaw)
            Y_rep = Krep * S * np.sin(yaw) + Krep * S * np.cos(yaw)

            print(f'Repulsion en X {X_rep}')
            print(f'Repulsion en Y {Y_rep}')
            #Path planning
            P_x = X_att - X_rep
            P_y = Y_att - Y_rep
            P = np.sqrt(P_x**2 + P_y**2)
            print("P = " + str(P))
            #
            #rotate = np.arctan2((yFinal - y),( xFinal - x)) - (np.pi - abs(yaw)) * np.sign(yaw)
            C_p = np.arctan2(P_y, P_x) - (np.pi - abs(yaw)) * np.sign(yaw)
            print("C_p = " + str(C_p))

            #Con esto tenemos el ángulo y la fuerza con la que debe moverse
            #Inicialmente se rota al ángulo deseado, y luego se avanza
            #El movimiento será proporcional a la fuerza calculada
            #ES COMO EL APPROACH 1 mas o menos

            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0.3 * np.sign(C_p) #El signo indica la direccion
            self.vel_pub.publish(twist_msg)
            time.sleep(abs(C_p)/0.3) #Tiempo proporcional a cuanto debe rotar

            #Se detiene la rotacion
            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(0.5)

            twist_msg = Twist()
            twist_msg.linear.x  = 0.1*P if P > 0.7 else 0.3*P
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(1.2) 

            twist_msg = Twist()
            twist_msg.linear.x  = 0
            twist_msg.angular.z = 0
            self.vel_pub.publish(twist_msg)
            time.sleep(0.5)

            #Se leen las posiciones actuales
            x = self.odom_x 
            y = self.odom_y
            yaw = self.odom_yaw

            itCount += 1
            print("Iteracion Nro " + str(itCount))
            if itCount > 50 or P< 0.1:
                print("Break")
                break
        
        #Se detiene todo
        twist_msg = Twist()
        twist_msg.linear.x  = 0
        twist_msg.angular.z = 0
        self.vel_pub.publish(twist_msg)
        time.sleep(1)


        self.printOdomvsGroundTruth()
        self.plotOdomVsGroundTruth(name = "Assignment 4")

    def get_collapsed_histogram(self, axis=0):
        while self.currentImage is None:
            pass
        img = self.currentImage

        # Apply the mask
        mask = cv2.inRange(img, (0, 50, 0), (50, 255, 50))
        return mask.sum(axis=axis)

    def get_center_of_mass(self):
        collapsed_histogram = self.get_collapsed_histogram()
        width = len(collapsed_histogram)

        # Calculate the index corresponding to the center of mass
        center_of_mass = self.calculate_center_of_mass_x(collapsed_histogram)

    def calculate_center_of_mass_x(self, arr):
        arr_length = len(arr)
        return np.round((np.arange(arr_length + 1)[1:] @ arr) / (np.ones(arr_length + 1)[1:] @ arr) - 1)


if __name__ == '__main__':
    node = EL5206_Robot()
    print("EL5206 Node started!")

    try:
        # node.dance()
        node.imagesave()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")