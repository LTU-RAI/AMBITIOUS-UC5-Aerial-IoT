class Agent:
    def __init__(self):
        self.waypoints = {}
        self.order = []
        self.ros_services = {}
        self.ros_topics = {}
        self.intruder_sub = {}
        self.str_pub = None