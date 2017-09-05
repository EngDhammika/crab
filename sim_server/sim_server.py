from multiprocessing import Process, Pipe
import Pyro4,time,sys
import random


@Pyro4.expose
class GreetingMaker(object):

    Data = " this the data"
    avilable = False

    @staticmethod
    def set_Data(data):
        global Data
        Data = data
        #Data =
        print "set_Data"
        #print self.Data

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
    def get_Data():

        #if self.is_avilable():
        global Data
        return Data
        #else:
            #return 0
#..............................................................................

#..............................................................................
def comuincation1(conn):

    conn.send("bringing dispaly server up ............") # give the message connection is up
    daemon = Pyro4.Daemon()  # make a Pyro daemon
    ns = Pyro4.locateNS()  # find the name server
    uri = daemon.register(GreetingMaker)  # register the greeting maker as a Pyro object
    ns.register("example.greeting", uri)  # register the object with a name in the name server
    print("Ready.")
    daemon.requestLoop()  # start the event loop of the server to wait for calls
#..............................................................................

if __name__ == '__main__':
    #.................................................
    file ="fast_slam_counter.txt"
    #................................................make the child process
    parent_conn, child_conn = Pipe()
    p = Process(target=comuincation1, args=(child_conn,))
    p.start()
    #................................................print out ready message
    print parent_conn.recv()

    #................................................give time to ready the server
    time.sleep(0.02) # give time to stablish the server

    #................................................make greetmaker client for server side access
    try:
        greeting_maker = Pyro4.Proxy("PYRONAME:example.greeting")
    except: Pyro4.errors.CommunicationError
    #..................................................
    greeting_maker.false_avilable()
    count = 1
    f = open(file,"r")
    lines = f.readlines()
    while True:
        Data = (lines[(count-1)*6],lines[(count-1)*6+1], lines[(count -1)*6+2]\
                    , lines[(count-1)*6 + 3], lines[(count-1)*6+4],lines[(count -1)*6 +5]\

                     )
        #...................................setting up the result data
        #print l
        while greeting_maker.is_avilable():

            print ".",
            time.sleep(0.1)
        print greeting_maker.is_avilable()
        greeting_maker.set_Data(Data)
        #...................................make data avilable
        greeting_maker.set_avilable()
        print greeting_maker.is_avilable()
        #print greeting_maker.get_Data()
        #.....................................Dummy_work
        print "perent runing"
        time.sleep(0.2)
        count = count+1
        #.....................................
    p.join()