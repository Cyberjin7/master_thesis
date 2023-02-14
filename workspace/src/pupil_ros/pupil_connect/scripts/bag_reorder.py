import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse


def status(length, percent):
    sys.stdout.write('\x1B[2K')
    sys.stdout.write('\x1B[0E')
    progress = "Progress: ["
    for i in range(0, length):
        if i < length * percent:
            progress += '='
        else:
            progress += ' '
    progress += "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="Reorder bagfile based on pupil timestamps.", allow_abbrev=False)
    parser.add_argument('bagfile', type=str, nargs=1, help='Input bag file')
    args = parser.parse_args()

    bagfile = args.bagfile[0]

    if not os.path.exists(bagfile):
        print('File does not exist')
        sys.exit()
    # info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagfile], stdout=subprocess.PIPE).communicate()[0])
    info_dict = yaml.load(rosbag.Bag(bagfile, 'r')._get_yaml_info(), Loader=yaml.SafeLoader)
    duration = info_dict['duration']
    start_time = info_dict['start']
    destbag = os.path.splitext(bagfile)[0] + "_reordered.bag"

    topics = rosbag.Bag(bagfile).get_type_and_topic_info()[1].keys()
    print(topics)

    with rosbag.Bag(destbag, 'w') as outbag:
        last_time = time.process_time()
        for topic, msg, t in rosbag.Bag(bagfile).read_messages():

            if time.process_time() - last_time > .1:
                percent = (t.to_sec() - start_time) / duration
                status(40, percent)
                last_time = time.process_time()

            try:
                outbag.write(topic, msg, rospy.Time.from_sec(msg.timestamp))
            except AttributeError:
                print('Pupil timestamp could not be found. Printing header timestamp.')
                outbag.write(topic, msg, t)
        outbag.close()

    status(40, 1)
    print("\ndone")


if __name__ == '__main__':
    main()
