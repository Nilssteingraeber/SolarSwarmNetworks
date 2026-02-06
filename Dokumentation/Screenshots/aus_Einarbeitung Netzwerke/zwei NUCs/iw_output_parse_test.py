iw_output = 'Station 38:00:25:52:f7:10 (on wlp0s20f3)\n\tinactive time:\t8 ms\n\trx bytes:\t783842\n\trx packets:\t16667\n\ttx bytes:\t1364\n\ttx packets:\t10\n\ttx retries:\t3\n\ttx failed:\t0\n\trx drop misc:\t0\n\tsignal:  \t-30 [-30, -32] dBm\n\tsignal avg:\t-29 [-29, -31] dBm\n\ttx duration:\t0 us\n\trx bitrate:\t1.0 MBit/s\n\trx duration:\t0 us\n\tauthorized:\tyes\n\tauthenticated:\tyes\n\tassociated:\tyes\n\tpreamble:\tlong\n\tWMM/WME:\tyes\n\tMFP:\t\tno\n\tTDLS peer:\tno\n\tDTIM period:\t0\n\tbeacon interval:100\n\tconnected time:\t1791 seconds\n\tassociated at [boottime]:\t783.050s\n\tassociated at:\t1758713521111 ms\n\tcurrent time:\t1758715312271 ms\n'

def parse_all(s):
    from datetime import datetime
    t1 = datetime.now()
    stations = s.split('Station ')
    print(stations)
    print()

    station_dict = {}
    for station in stations:
        if station:
            mac = station[0:17]
            station_dict[mac] = {}
            print('mac:', mac)
            lines = station[17:].split('\n\t')
            for line in lines:
                temp = line.partition(':')
                station_dict[mac][temp[0]] = temp[2].lstrip().replace('\t', '')

    for mac in station_dict.keys():
        temp = station_dict[mac].items()
        for item in temp:
            print(item)
    # print(station_dict)
    t2 = datetime.now()
    print('Finished after:', t2-t1)

def parse_iw_station_dump(s):
    stations = s.split('Station ')
    station_dict = {}
    
    for station in stations:
        if station:
            mac = station[0:17]
            lines = station[17:].split('\n\t')
            for line in lines:
                if 'signal avg' in line:
                    temp = line.partition(':')
                    # remove spaces, temove \t, get first value, parse float
                    try:
                        num = temp[2].lstrip().replace('\t', '').partition(' ')[0]
                        # print(num)
                        station_dict[mac] = float(num)
                    except:
                        station_dict[mac] = None
                    break
    # print(station_dict)
    return station_dict

iw_output = 'Station 38:00:25:52:f7:10 (on wlp0s20f3)\n\tinactive time:\t8 ms\n\trx bytes:\t783842\n\trx packets:\t16667\n\ttx bytes:\t1364\n\ttx packets:\t10\n\ttx retries:\t3\n\ttx failed:\t0\n\trx drop misc:\t0\n\tsignal:  \t-30 [-30, -32] dBm\n\tsignal avg:\t-29 [-29, -31] dBm\n\ttx duration:\t0 us\n\trx bitrate:\t1.0 MBit/s\n\trx duration:\t0 us\n\tauthorized:\tyes\n\tauthenticated:\tyes\n\tassociated:\tyes\n\tpreamble:\tlong\n\tWMM/WME:\tyes\n\tMFP:\t\tno\n\tTDLS peer:\tno\n\tDTIM period:\t0\n\tbeacon interval:100\n\tconnected time:\t1791 seconds\n\tassociated at [boottime]:\t783.050s\n\tassociated at:\t1758713521111 ms\n\tcurrent time:\t1758715312271 ms\n'
print(parse_iw_station_dump(iw_output))

class has_dict(object):
    def __init__(self):
        self.my_dict = dict()

def give_me_dict(d):
    d['b'] = 2
    print(hex(id(d)))
    d['c'] = 3

A = has_dict()
A.my_dict['a'] = 1
print(hex(id(A.my_dict)))
give_me_dict(A.my_dict)
print(A.my_dict)
