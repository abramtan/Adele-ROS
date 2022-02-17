#!/usr/bin/env python
import time
import signal
import socket
import sys
import rospy
import geometry_msgs.msg

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 1234        # Port to listen on (non-privileged ports are > 1023)

def signal_handler(signal, frame):
    print("exiting")
    sys.exit(0)

def move(text):
    distance = 2
    speed = 2

    if text == 'right':
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
    elif text == 'left':
        pose_goal.orientation.w = 2.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
    elif text == 'forward':
        pose_goal.orientation.w = 3.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
    elif text == 'backwards':
        pose_goal.orientation.w = 4.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
    elif text == 'down':
        pose_goal.orientation.w = 5.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
    elif text == 'up':
        pose_goal.orientation.w = 0.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
    elif text == 'to':
        translationOrientation = True

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        #Publish the velocity
        velocity_publisher.publish(pose_goal)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    # pose_goal.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish(pose_goal)

if __name__ == '__main__':
    # signal.signal(signal.SIGINT, signal_handler)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            conn, addr = s.accept()
            
            rospy.init_node('robot_cleaner', anonymous=True)
            velocity_publisher = rospy.Publisher('/iosCommands', geometry_msgs.msg.Pose, queue_size=10)
            pose_goal = geometry_msgs.msg.Pose()

            with conn:
                print('Connected by', addr)
                while True:
                    data = conn.recv(4096)
                    if not data:
                        break
                    conn.sendall(data)
                    text = data.decode("utf-8")
                    print(text)
                    move(text)

    except rospy.ROSInterruptException: pass

    except KeyboardInterrupt:
        print("\nCleaning up and exiting...")
        # sys.exit(0)

    finally:
        s.close()
        time.sleep(1)