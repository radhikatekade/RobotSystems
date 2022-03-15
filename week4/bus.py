from time import time
from  readerwriterlock  import  rwlock


class Bus:

    def __init__(self, msg_type=None):
        self.type = msg_type
        self.message = None
        self.time = None
        self.lock = rwlock.RWLockWriteD ()

    def read(self):
        with  self.lock.gen_rlock ():
            message = self.message
        return self.message, self.time

    def write(self, msg):
        
        with  self.lock.gen_wlock ():
            self.message = msg

        if self.type is not None:
            assert type(msg) is self.type,\
                "Message must be of type {}".format(self._type)

        self.message = msg
        self.time = time()


if __name__ == "__main__":

    # create bus with msg type of int
    bus = Bus(int)

    # write message to bus
    msg = 10
    bus.write(msg)

    # read message from bus
    msg, t = bus.read()
    print("Message: {}\nTimestamp: {}".format(msg, t))