import rospy 
import time
from std_msgs.msg import Float64MultiArray

rospy.init_node("testPwmNode")
pwmPublisher = rospy.Publisher("/PWM", Float64MultiArray, queue_size = 10)
pwmData = Float64MultiArray()

for i in range(100):
    pwmData.data = [1500+2*i, 1500-2*i]
    pwmPublisher.publish(pwmData)
    time.sleep(0.02)

for i in range(100, 0, -1):
    pwmData.data = [1500+2*i, 1500-2*i]
    pwmPublisher.publish(pwmData)
    time.sleep(0.02)


pwmData.data = [1500, 1500]
pwmPublisher.publish(pwmData)