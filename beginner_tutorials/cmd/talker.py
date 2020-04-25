import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker')
    publisher = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = 'hello world %s' % rospy.get_time()
        publisher.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass