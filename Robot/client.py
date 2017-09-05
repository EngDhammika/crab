import Pyro4
import sys,time






robot = Pyro4.Proxy("PYRONAME:example.robot")    # use name server object lookup uri shortcut
print(robot)

while True:
    if robot.is_avilable():

        print robot.is_avilable()
        data_encode = robot.get_DataEncode();
        data_scan = robot.get_DataScan();
        robot.false_avilable()
        splited_encodedata = data_encode.split()
        spilted_scandata = data_scan.split()
        # ...........................
        encode = splited_encodedata[2:14]
        encode_data = []
        encode_data.append('M')
        for i in range(len(encode)):
            encode_data.append(int(encode[i]))
        # ...........................
        scan = spilted_scandata[2:]
        scan_data = []
        # scan_data.append('S')
        for i in range(len(scan)):
            scan_data.append(int(scan[i]))
        # .................................
        print(encode_data)
        print(scan_data)
        print("----------------------------------")


    time.sleep(0.5)

