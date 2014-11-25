#!/usr/bin/env python

import rospy

from datetime import *
from dateutil.tz import tzutc, tzlocal
import thread
from routine_behaviours.marathon_routine import MarathonRoutine
from task_executor import task_routine
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from mongodb_store_msgs.msg import StringList
from mongodb_store.message_store import MessageStoreProxy
from std_srvs.srv import Empty

def clear_schedule_monitor(start_time, end_time):
    rostime_now = rospy.get_rostime()
    now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal()).time()
    print now, start_time
    if task_routine.time_less_than(start_time, now)\
            and task_routine.time_less_than(now, end_time):
        clear_schedule_srv_name = '/task_executor/clear_schedule'
        clear_schedule = rospy.ServiceProxy(clear_schedule_srv_name, Empty)
        try:
            clear_schedule()
        except rospy.ServiceException, e:
            rospy.loginfo('Empty service complaint occurs here. Should be safe: %s'% e)
    rospy.sleep(60)

def create_mongodb_store_task2(db, to_replicate, delete_after_move=True):
    task = Task()
    # no idea, let's say 2 hours for now -- this action server can't be preempted though, so this is cheating
    task.max_duration = rospy.Duration(60 * 30)
    task.action = 'move_mongodb_entries'
    task.start_node_id == 'ChargingPoint'

    # replicate from this db
    task_utils.add_string_argument(task, db)

    # add arg for collectionst o replication
    collections = StringList(to_replicate)
    msg_store = MessageStoreProxy()
    object_id = msg_store.insert(collections)
    task_utils.add_object_id_argument(task, object_id, StringList)

    # move stuff over 24 hours old
    task_utils.add_duration_argument(task, rospy.Duration(60 * 1))
    
    # and delete afterwards
    task_utils.add_bool_argument(task, delete_after_move)
    return task

if __name__ == '__main__':
    rospy.init_node("marathon_routine")

    # start and end times -- all times should be in a particular timezone - local has stopped working!
    # localtz = tzlocal()
    localtz = tzutc()

    now = datetime.now(localtz).time().replace(tzinfo=localtz)
    # useful for testing
    # start = now

    start = time(10,00, tzinfo=localtz)
    end = time(3,00, tzinfo=localtz)

    day_points = [
        'Axes',
        'Gold',
        'Boat',
        'PlayArea',
        'Tank',
        'Fossil',
        'Armour',
        'Centre'
    ]
    night_points = [
        'Podium',
        'LectureHallScreen',
        'LectureHallRight'
    ]

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    # how long do you want it to take to do a tour. this must be greater than the time you think it will take!
    # the number argument is in seconds
    tour_duration_estimate = rospy.Duration(60 * 50)

    routine = MarathonRoutine(daily_start=start, daily_end=end,
        idle_duration=idle_duration, tour_duration_estimate=tour_duration_estimate)


    # choose which nodes are visited at random on idle
    # routine.random_nodes = ['WayPoint2', 'WayPoint3']
    # or
    #routine.random_nodes = routine.all_waypoints_except(['WayPoint2', 'WayPoint3'])
    routine.day_random_nodes = day_points
    routine.night_random_nodes = night_points

    # go around every node every tour_duration_estimate
    # routine.create_patrol_routine()

    lock_in = time(15,55, tzinfo=localtz)
    start_midday_upload = time(16,00, tzinfo=localtz)
    night_start = time(19,00, tzinfo=localtz)
    routine.day_shift_end = lock_in
    routine.night_shift_start = night_start


    thirty_mins = timedelta(minutes = 30)
    sixty_mins = timedelta(minutes = 60)
    ninety_mins = timedelta(minutes = 90)


    # patrol just these selected waypoints every 30 minutes in the first part of the day
    #routine.create_patrol_routine(waypoints=day_points, daily_start=start, daily_end=lock_in, repeat_delta=timedelta(hours=6))

    # then all but these every 30 minutes in the second part part of the day
    #routine.create_patrol_routine(waypoints=night_points, daily_start=night_start, daily_end=end, repeat_delta=thirty_mins)

    # do 3d scans
    scan_waypoints = ['Boat', 'WayPoint33', 'WayPoint74']
    routine.create_3d_scan_routine(waypoints=scan_waypoints, repeat_delta=timedelta(hours=3), daily_start=start, daily_end=lock_in)
    scan_waypoints = night_points
    routine.create_3d_scan_routine(waypoints=scan_waypoints, repeat_delta=timedelta(hours=4), daily_start=night_start, daily_end=end)

    # where to stop and what to tweet with the image  TODO: Find waypoints
    twitter_waypoints = [['PlayArea', 'I hope everyone is having fun @collectionlinc #ERW14 #RobotMarathon'],
                         ['Centre', 'Exciting exhibitions @collectionlinc #ERW14 #RobotMarathon']]
    #routine.create_tweet_routine(twitter_waypoints, daily_start=start, daily_end=lock_in, repeat_delta=timedelta(hours=1))

    # Creat lock in upload before starting night patrols
    db = 'message_store'
    collections = ['heads','metric_map_data','rosout_agg','robot_pose','task_events','scheduling_problems','ws_observations','monitored_nav_events', 'people_perception']
    routine.message_store_entries_to_replicate(collections, db=db)
    mongodb_task = create_mongodb_store_task2(db, collections, True)
    routine.create_task_routine(tasks=mongodb_task, daily_start=start_midday_upload, daily_end=night_start, repeat_delta=timedelta(hours=2))

    db = 'roslog'
    collections = ['head_xtion_compressed_depth_libav', 'head_xtion_compressed_rgb_theora', 'head_xtion_compressed_rgb_compressed']
    routine.message_store_entries_to_replicate(collections, db=db)
    mongodb_task = create_mongodb_store_task2(db, collections, True)
    routine.create_task_routine(tasks=mongodb_task, daily_start=start_midday_upload, daily_end=night_start, repeat_delta=timedelta(hours=2))

    db = 'metric_maps'
    collections = ['data', 'summary']
    routine.message_store_entries_to_replicate(collections, db=db)
    mongodb_task = create_mongodb_store_task2(db, collections, True)
    routine.create_task_routine(tasks=mongodb_task, daily_start=start_midday_upload, daily_end=night_start, repeat_delta=timedelta(hours=2))

    thread.start_new_thread(clear_schedule_monitor, (lock_in, start_midday_upload, ))

    routine.start_routine()

    rospy.spin()
