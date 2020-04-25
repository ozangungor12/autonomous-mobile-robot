import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("I heard {0}".format(msg.data))
        
def listener():

    rospy.init_node("listener")
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass