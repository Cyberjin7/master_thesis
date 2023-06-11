import rospy
from sync_msgs.msg import TrialType, MassTrial

from pysinewave import SineWave
import time

def trialCallback(data):
    pass

def typeCallback(data):
    print(data.type)
    if data.type == "rest":
        sinewave = SineWave(pitch=10, decibels=10)
        sinewave.play()
        time.sleep(0.5)
        sinewave.stop()
    elif data.type =="load":
        sinewave = SineWave(pitch=12, decibels=10)
        sinewave.play()
        time.sleep(0.5)
        sinewave.stop()
    elif data.type == "unload":
        sinewave = SineWave(pitch=11, decibels=10)
        sinewave.play()
        time.sleep(0.5)
        sinewave.stop()




if __name__ == '__main__':
    rospy.init_node('unload_helper', anonymous=True)

    rospy.Subscriber("load_trial", MassTrial, trialCallback)
    rospy.Subscriber("load_type", TrialType, typeCallback)

    rospy.spin()