import Pyro4
from multiprocessing import Pipe , Process
import time

@Pyro4.expose
class Robot(object):

    Dataencode = " this the data"
    Datascan = " "
    avilable = False

    @staticmethod
    def set_DataEncode(data):
        global Dataencode
        Dataencode = data
        # Data =
        print "set_Data"
        # print self.Data

    @staticmethod
    def set_DataScan(data):
        global Datascan
        Datascan = data
        # Data =
        print "set_Data"
        # print self.Data

    @staticmethod
    def is_avilable():
        global avilable
        return avilable

    @staticmethod
    def set_avilable():
        global avilable
        avilable = True

    @staticmethod
    def false_avilable():
        global avilable
        avilable = False

    @staticmethod
    def get_DataEncode():
        global Dataencode
        return Dataencode

    @staticmethod
    def get_DataScan():
        global Datascan
        return Datascan
# ..............................................................................
# ..............................................................................
def comuincation(conn):
    conn.send("bringing dispaly server up ............")  # give the message connection is up
    daemon = Pyro4.Daemon()  # make a Pyro daemon
    ns = Pyro4.locateNS()  # find the name server
    uri = daemon.register(Robot)  # register the greeting maker as a Pyro object
    ns.register("example.robot", uri)  # register the object with a name in the name server
    print("Ready.")
    daemon.requestLoop()  # start the event loop of the server to wait for calls
    # ..............................................................................

if __name__ == '__main__':
    #.................................................
    file1 ="robot4_motors.txt"
    file2 ="robot4_scan.txt"
    #................................................make the child process
    parent_conn, child_conn = Pipe()
    p = Process(target=comuincation, args=(child_conn,))
    p.start()
    #................................................print out ready message
    print parent_conn.recv()

    #................................................give time to ready the server
    time.sleep(0.02) # give time to stablish the server

    #................................................make greetmaker client for server side access
    try:
        self = Pyro4.Proxy("PYRONAME:example.robot")
    except: Pyro4.errors.CommunicationError
    #..................................................
    self.false_avilable()
    f1 = open(file1,"r")
    f2 = open(file2,"r")
    for l,n in zip(f1,f2):
        while self.is_avilable():
            print " . ",
            time.sleep(0.1)
        self.set_DataEncode(l)
        self.set_DataScan(n)
        self.set_avilable()

    p.join()
