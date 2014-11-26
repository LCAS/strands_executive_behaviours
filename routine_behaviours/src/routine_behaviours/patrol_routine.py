#!/usr/bin/env python

import rospy

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store_msgs.msg import StringList
from mongodb_store.message_store import MessageStoreProxy

from datetime import *
from dateutil.tz import tzlocal, tzutc

from routine_behaviours.robot_routine import RobotRoutine
from task_executor import task_routine

import random


def create_patrol_task(waypoint_name):
    return Task(start_node_id=waypoint_name, end_node_id=waypoint_name, max_duration=rospy.Duration(30))


class PatrolRoutine(RobotRoutine):
    """ Creates a routine which simply visits nodes. """

    def __init__(self, daily_start, daily_end, tour_duration_estimate=None, idle_duration=rospy.Duration(5), charging_point = 'ChargingPoint'):
        # super(PatrolRoutine, self).__init__(daily_start, daily_end)        
        RobotRoutine.__init__(self, daily_start, daily_end, idle_duration=idle_duration, charging_point=charging_point)
        localtz = tzutc()
        self.node_names = set()
        self.tour_duration_estimate = tour_duration_estimate
        #rospy.Subscriber('topological_map', TopologicalMap, self.map_callback)
        self.day_random_nodes = []
        self.night_random_nodes = []
        self.day_shift_end = time(16,00, tzinfo=localtz)
        self.night_shift_start = time(18,00, tzinfo=localtz)

    #def map_callback(self, msg):
        #print 'got map: %s' % len(msg.nodes)
        #self.node_names = set([node.name for node in msg.nodes if node.name != 'ChargingPoint'])
        #if len(self.random_nodes) == 0:
            #self.random_nodes = list(self.node_names)

    def get_nodes(self):
        while len(self.node_names) == 0:
            print 'no nodes'
            rospy.sleep(1)
        return self.node_names


    def all_waypoints(self):
        return self.get_nodes()

    def all_waypoints_except(self, exceptions = []):
        return self.all_waypoints() - set(exceptions)


    def create_patrol_routine(self, waypoints=None, daily_start=None, daily_end=None, repeat_delta=None):
        #if not waypoints: 
            #waypoints = self.get_nodes()

        if not repeat_delta:
            if not self.tour_duration_estimate:
                single_node_estimate = 180
                self.tour_duration_estimate=rospy.Duration(single_node_estimate * len(node_names))

            repeat_delta = timedelta(seconds=self.tour_duration_estimate.to_sec())

        tasks = [ create_patrol_task(n) for n in waypoints ]

        self.create_task_routine(tasks=tasks, daily_start=daily_start, daily_end=daily_end, repeat_delta=repeat_delta)



    def create_routine(self):
        
        self.create_patrol_routine()


    def is_day_shift(self,time):
        return task_routine.time_less_than(time, self.day_shift_end)
    def is_night_shift(self,time):
        return task_routine.time_less_than(self.night_shift_start, time)


    def on_idle(self):
        """
            Called when the routine is idle. Default is to trigger travel to the charging. As idleness is determined by the current schedule, if this call doesn't utlimately cause a task schedule to be generated this will be called repeatedly.
        """
        rostime_now = rospy.get_rostime()
        now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal()).time()
        if self.is_day_shift(now):
            random_nodes = self.day_random_nodes
        elif self.is_night_shift(now):
            random_nodes = self.night_random_nodes
        else:
            return

        if not isinstance(random_nodes, list):
            random_nodes = list(random_nodes)

        rospy.loginfo('Idle for too long, generating a random waypoint task')
        self.add_tasks([create_patrol_task(random.choice(self.random_nodes))])
    

