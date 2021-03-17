import os
import struct
import sys
import rospkg


class RosCppCommunicator():

    def __init__(self):

        self.rospack = rospkg.RosPack()
       
        #this is the path of the file that called this constructor.
        #we will use the name of this path to find the correct fd
        self.callers_path = sys._getframe(2).f_globals['__file__']
        #strip down to find the just the file
        split_path = self.callers_path.split("/")
        file_name_extension = split_path[-1]
        self.calling_file_name = file_name_extension[:-3]

        try:
            self._write_fd = int(os.getenv(self.calling_file_name + "_PY_WRITE_FD"))
            self.write_pipe = os.fdopen(self._write_fd, 'wb', 0)
        except:
            self.write_pipe = None
            print("Unable to create write pipe")


    def log_to_ros(self, msg):
        msg = str(msg)
        if self.write_pipe is None:
            print(msg)
        else:
            msg_size = struct.pack('<I', len(msg))
            self.write_pipe.write(msg_size)
            self.write_pipe.write(msg.encode("utf-8"))


    def get_package_path(self, packag_name):
        return self.rospack.get_path(packag_name)