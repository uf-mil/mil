#!/usr/bin/env python
import Queue


class ThreadQueue(object):
    def __init__(self, time, max_size=0):
        '''
        Pass in a method that can be queried for the time
        '''
        self.max_size = max_size
        self.time = time
        self.run_queue = Queue.PriorityQueue(maxsize=self.max_size)
        self.is_running = False

    def put(self, funct, time_to_exec=-1):
        '''
        Given a function to run, a time to run it, add it to the queue of functions to run.
        This uses the `time_to_exec` as the priority in the queue.

        If `time_to_exec` is not defined, it will put `funct` at the beginning of the queue.
        '''
        item_to_add = (time_to_exec, funct)
        self.run_queue.put(item_to_add, block=True, timeout=.5)

    def next_if_possible(self):
        '''
        Check if there is an item that needs to be run (based on time). If there
            is, the function will be run.
        '''
        if self.is_running: return False

        self.is_running = True

        run_queue = self.run_queue
        if not run_queue.empty() and self.time() >= run_queue.queue[0][0]:
            print run_queue.queue[0][0], run_queue.queue[0][1].__name__
            run_queue.get(block=True, timeout=.5)[1]()

        self.is_running = False

        return True

    def clear(self):
        print "Clearing thread queue"
        self.run_queue = Queue.PriorityQueue(maxsize=self.max_size)


if __name__ == "__main__":
    import rospy
    from std_msgs.msg import Int8

    def funct(times, d):
        print "Running", d
        t = time.time()
        for i in range(times):
            time.sleep(.1)
        #print "Done in {}s".format(time.time() - t)

    r = lambda: funct(50, "a")
    r1 = lambda: funct(30, "b")

    tq = ThreadQueue()

    def add_thread(msg, tq, f):
        print "Adding thread: ", f, rospy.Time.now() + rospy.Duration(msg.data)
        tq.put(f, rospy.Time.now().to_sec() + msg.data)

    rospy.init_node("bleb")
    rospy.Subscriber("test_thread", Int8, lambda msg: add_thread(msg, tq, r))
    rospy.Subscriber("test_thread2", Int8, lambda msg: add_thread(msg, tq, r1))

    while not rospy.is_shutdown():
        tq.next_if_possible(rospy.Time.now().to_sec())
        time.sleep(.1)
