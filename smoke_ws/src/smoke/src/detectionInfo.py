class detectionInfo(object):
    def __init__(self, line):
        if isinstance(line, list):
            self.name = line[0]

            self.truncation = float(line[1])
            self.occlusion = int(line[2])

            # local orientation = alpha + pi/2
            self.alpha = float(line[3])

            # in pixel coordinate
            self.xmin = int(line[4])
            self.ymin = int(line[5])
            self.xmax = int(line[6])
            self.ymax = int(line[7])

            # height, weigh, length in object coordinate, meter
            self.h = float(line[8])
            self.w = float(line[9])
            self.l = float(line[10])

            # x, y, z in camera coordinate, meter
            self.tx = float(line[11])
            self.ty = float(line[12])
            self.tz = float(line[13])

            # global orientation [-pi, pi]
            self.rot_global = float(line[14])
            self.score = float(line[15])
        else: # if ros msg 
            self.load_ros_msg(line)
        self.bbox = [self.xmin, self.ymin, self.xmax, self.ymax]
        self.dimensions =  [self.h, self.w, self.l]
        self.location = [self.tx, self.ty, self.tz]
   
    def load_ros_msg(self,ros_msg):
        self.name = int(ros_msg.type)

        self.truncation = float(ros_msg.truncated)
        self.occlusion = float(ros_msg.occluded)

        # local orientation = alpha + pi/2
        self.alpha = float(ros_msg.alpha)

        # in pixel coordinate
        self.xmin = int(ros_msg.bbox[0])
        self.ymin = int(ros_msg.bbox[1])
        self.xmax = int(ros_msg.bbox[2])
        self.ymax = int(ros_msg.bbox[3])

        # height, weigh, length in object coordinate, meter
        self.h = float(ros_msg.dimensions[0])
        self.w = float(ros_msg.dimensions[1])
        self.l = float(ros_msg.dimensions[2])

        # x, y, z in camera coordinate, meter
        self.tx = ros_msg.location[0]
        self.ty = ros_msg.location[1]
        self.tz = ros_msg.location[2]

        # global orientation [-pi, pi]
        self.rot_global = ros_msg.rotation_y

        self.score = ros_msg.score