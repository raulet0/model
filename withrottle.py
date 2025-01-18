# withrottle.py
# Michel RAULET 2025
# Version 1.0

# 10-01-2025 first prototype
# 11-01-2025 first usable program
# 13-01-2025 refactoring
# 15-01-2025 adding Locomotive class (overall 675 lines)
# 17-01-2025 Complete (780 lines, comments included)


"""

Protocol specification best documentation:
https://www.jmri.org/help/en/package/jmri/jmrit/withrottle/Protocol.shtml

The 'M' multithrottle can handle more than one locomotive.
Engine Driver uses this feature, for example, to make it super easy to create a consist
on the fly with any set of engines (without using DCC consisting).
You can have one or more multithrottles,
and each multithrottle can have more than one locomotives attached to it.
The first character after the 'M' is used as a key for the instance of MultiThrottle to use.
Engine Driver uses '0' through '6' as its keys for multithrottle instances.
However, you can use other characters for the key, 'T' is used in the examples below.
Throttle key (caracter) can be choosen (but it doesn't really matter)
- P for wiThrottle iOS
- 0-6 for EngineDriver Android
- X for this pywithottle

Vocabulary:
This program is a client wiThrottleManager (a multithrottle manager) !
It aims to be connected with a running wiThrottle server like the JMRI one.
Throttle means one locomotive Remote Control hardware/software device.
Multithrottle means a way to control multiple locos/consists in the same time.
'dcc-address' is DCC address column in JMRI roster (decodeur adress).
Here, LocoID = S|L<dcc-address> (S for Short, L for Long DCC addresses).

"""

import re # regex
import socket # for network management
import time # for sleep
import sys # for exit
import threading # for Thread

#################################################
# Constantes

TCP_IP = '127.0.0.1'
TCP_PORT = 12090
BUFFER_SIZE = 1024 # be careful in PyWiThrottle class
CRED,CGRE,CORA,CBLU,CEND = '\033[91m','\033[32m','\033[33m','\033[94m','\033[0m'

#################################################
# Variables globales

# TODO: useless
scanning = True # False stops files and queue scanning

#################################################

class wiThrottleManager(object):

    # throttles = {} # class attribute

    def __init__(self,client_name,server_ip,server_port,verbose=True):
        self.client_name = client_name # ex. 'MraThrottle'
        self.server_ip = server_ip
        self.server_port = server_port
        self.cx = None # socket
        self.recv_buffer_size = 2048
        self.connected = False
        self.rosterlist = [] # raw data of all locomotives defined in the roster
        self.rosterdict = {} # all Locomotives defined in the roster
        self.rosterkeys = [] # all LocoIDs (all S|L<dcc-address>)
        self.registered_locos = [] # list of registered LocoIDs (S3 or L8504, etc)
        self.registered_locomotives = {} # list of Locomotive objects
        self.verbose = verbose
        self.power = None # layout power status
        # socket connection and wiThrottle server connection
        self.register_throttle()

    # @classmethod
    # def getThrottles(cls):
    #     return cls.throttles

    def display(self): # display Throttle Manager data
        print(f"{self.client_name}:")
        print(f"Layout Power {self.power}")
        for loco in self.registered_locomotives:
            self.registered_locomotives[loco].display()

    def hello(self):
        print(f"HELLO {self.client_name}")

    def set_verbose(self,verbose=True):
        self.verbose = verbose

    def print_trace(self,message):
        if self.verbose:
            print(message)

    def print_error(self,error_message):
        print(f"{CRED}ERROR {self.client_name}: {error_message}{CEND}")

    def sendwt(self,order):
        # we should send BYTES, not string
        # Bytes literals are always prefixed with 'b' or 'B';
        # they produce an instance of the bytes type instead of the str type.
        # They may only contain ASCII characters;
        # bytes with a numeric value of 128 or greater must be expressed with escapes.
        # if self.cx is not None:
        if self.connected:
            self.print_trace(f"{CORA}SENDING {order}{CEND}")
            order = order + '\n'
            self.cx.send(order.encode('ascii')) # Returns the number of bytes sent.
        else:
            self.print_error(f"sendwt: {self.client_name} not connected")
            # sendwt: MraThrottle not connected

    def readwt(self,evt):
        self.print_trace(f"Socket read loop with {self}")
        data = ""
        while self.connected and not evt.is_set():
            try:
                octets = self.cx.recv(BUFFER_SIZE)
                mystring = octets.decode('ascii') # utf-8 or ascii ?
                data = data + mystring
                # The rsplit() method splits a string into a list, starting from the right
                # setting the maxsplit parameter to 1, will return a list with 2 elements!
                bits = data.rsplit('\n',1)
                if len(bits) > 1:
                    data = bits[1]
                    lines = bits[0].split('\n')
                    for line in lines:
                        if line:
                            self.print_trace(f"{CBLU}{line}{CEND}")
                            if line.startswith("RL"):
                                self.roster(line)
                                continue
                            
                            if line.startswith("PPA"):
                                self.trackPowerNotification(line)
                                continue
                            
                            q = re.search(r"M(\w)A(\w*)<;>(\w)(.*)",line)
                            if q is not None:
                                self.print_trace(f"THROTTLE CHANGE NOTIFICATION: {q.groups()}")
                                self.throttleChangeNotification(q.groups())
                                continue

                            q = re.search(r"^PFT(\d+)<;>([0-9.]+)",line)
                            if q is not None:
                                self.fastClockNotification(q.group(1),q.group(2))
                                continue

                            # TODO:
                            # HMJMRI: address 'L23' not allowed as Long --> HMJMRI:...........
                            # other notifications

            except (socket.timeout):
                # we use non blocking socket so we have normal timeouts!
                pass
            except Exception as socket_error:
                self.print_error(f"readwt: {socket_error} ")
                # readwt: [Errno 54] Connection reset by peer
                self.connected = False
                break

#################################################
# Throttle Manager methods

    def connectwt(self):
        self.cx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.cx.connect((self.server_ip, self.server_port))
            self.cx.settimeout(0.5)
            self.connected = True
        except socket.error:
            self.print_error(f"connectwt: Failed to connect socket at {self.server_ip}:{self.server_port}")
            # connectwt: Failed to connect socket at 127.0.0.1:12090
            self.connected = False

    def disconnectwt(self):
        # removing/unregistering locos is not mandatory before disconnecting
        self.sendwt('Q')
        self.cx.close()
        self.connected = False

    def register_throttle(self):
        self.connectwt()
        self.sendwt(f"N{self.client_name}")

    def check_locoid(self,locoid):
        # locoid is a string like S2,S3,S8,L8504
        return locoid in self.rosterkeys

    def add_locomotive(self,locoid):
        # request to add a new locomotive to the multithrottle
        # MT+
        # locoid is a string like S2,S3,S8,L8504
        if self.check_locoid(locoid) == False:
            self.print_error(f"LocoID {locoid} is not part of the roster!")
            return
        if locoid in self.registered_locos:
            self.print_error(f"LocoID {locoid} is already registered!")
            return
        reg_data = f"MX+{locoid}<;>{locoid}"
        print(f"OK Registering Loco {locoid}")
        self.sendwt(reg_data)
        self.registered_locos.append(locoid)
        self.registered_locomotives[locoid] = Locomotive(locoid)

    def remove_locomotive(self,locoid):
        # MT-
        # locoid is a string like S2,S3,S8,L8504
        if self.check_locoid(locoid) == False:
            self.print_error(f"LocoID {locoid} is not part of the roster!")
            return
        if locoid not in self.registered_locos:
            self.print_error(f"LocoID {locoid} is not registered!")
            return
        reg_data = f"MX-{locoid}<;>r"
        print(f"OK Removing Loco {locoid} from registered locos")
        self.sendwt(reg_data)
        self.registered_locos.remove(locoid)
        self.registered_locomotives.pop(locoid)

    def add_locomotive_Notification(self):
        # After sending add command,
        # the WiThrottle Server will typically reply with this message
        # MT+L341<;>
        pass

    def remove_locomotive_Notification(self):
        # After sending release command,
        # the WiThrottle Server will typically reply with a removed message
        # MT-L341<;>
        pass

    def roster(self, data_in):
        # if re.match("RL(.*)",line):
        # examples:
        # RL0
        # RL2]\[RGS 41}|{41}|{L]\[Test Loco}|{1234}|{L
        # RL3]\[D&RGW 341}|{3}|{S]\[RGS 41}|{41}|{L]\[Test Loco}|{1234}|{L   
        self.rosterlist = []
        if data_in == 'RL0':
            return
        parse = re.search(r'RL([0-9]+)\]\\\[(.*)',data_in)
        if parse is not None:
            for entry in parse.group(2).split(r']\['):
                self.rosterlist.append(entry.split('}|{'))
        self.rosterdict = {}
        for loco in self.rosterlist:
            self.rosterdict[loco[2]+loco[1]] = loco[0]
        self.rosterkeys = list(self.rosterdict)
        self.print_trace(f"Roster list: {self.rosterlist}")
        self.print_trace(f"Roster dict: {self.rosterdict}")
        self.print_trace(f"Roster keys: {self.rosterkeys}")
        
    def print_roster(self):
        print(f"Roster: {self.rosterkeys}")

    def turnoutstate(self, data_in):
        # if re.match("PTT(.*)",line)
        # JMRI returns either zero, one, or two strings for this section.
        # The first string provides the labels for the different turnout states.
        # This string contains an array with two elements,
        # which provide the labels to use for each turnout state (except for Unknown)
        # PTT]\[Turnouts}|{Turnout]\[Closed}|{2]\[Thrown}|{4]\[Unknown}|{1]\[Inconsistent}|{8
        print("Turnout status:")
        self.turnoutstates = {}
        for entry in data_in[6:].split(r']\['):
            entrylist = entry.split('}|{')
            if entrylist[1] != 'Turnout':
                self.turnoutstates[entrylist[1]] = entrylist[0]
        print(self.turnoutstates)

    def turnout(self, data_in):
        # if re.match("PTL(.*)",line)
        # JMRI returns either zero, one, or two strings for this section.
        # The second string appears when you have turnouts defined in JMRI,
        # and contains the array of defined turnouts
        # PTL]\[LT12}|{Rico Station N}|{1]\[LT324}|{Rico Station S}|{2
        print("Turnout list:")
        self.turnouts = {}
        for entry in data_in[6:].split(r']\['):
            entrylist = entry.split('}|{')
            if entrylist[1]:
                self.turnouts[entrylist[1]] = {
                        'sysname': entrylist[0],
                        'state': entrylist[2]
                        }
            else:
                self.turnouts[entrylist[0]] = {
                        'sysname': entrylist[0],
                        'state': entrylist[2]}
        print(self.turnouts)

    def route(self, data_in):
        # if re.match("PRT(.*)",line)
        # JMRI returns either zero, one, or two strings for this section.
        # The first string is likewise a list of labels for the route values.
        # PRT]\[Routes}|{Route]\[Active}|{2]\[Inactive}|{4]\[Unknown}|{0]\[Inconsistent}|{8
        print("Route list:")
        routelist = []
        routes = {}
        for entry in data_in[6:].split(r']\['): # noqa W605
            entrylist = entry.split('}|{')
            routelist.append(entrylist)
            if entrylist[1]:
                routes[entrylist[1]] = entrylist[0]
            else:
                routes[entrylist[0]] = entrylist[0]
        print('\n'.join(map(str, routelist)))
        print(routes)
        # The second string contains an array of defined routes.
        # PRL]\[IR:AUTO:0001}|{Rico Main}|{2

    def fastClockNotification(self,timestamp,accel):
        # The 'PFT' command is used to synchronize the fast clock time.
        # It is sent from the server when time or rate change, indicating the "now" time
        # and the current time ratio (or 0 when clock stopped).
        # PFT1550686525<;>4.0
        # The first number is an integer number of seconds since 12:00 midnight, January 1st,
        # 1970 fast clock time.
        # The second number is the current fast time ratio. It may be an integer value (4)
        # or a floating point value (4.0). If this value is 0 (or 0.0), the clock is stopped.
        # Updates are sent on each change in time or rate, and when clock stopped or started,
        # so expect one message every fast clock minute.
        self.print_trace(f"PFT (fast clock message) {timestamp=} {accel=}")

    def alertInfoMessage(self,message):
        # The server can send a message string that the client can display.
        # These cannot have newlines in message.
        # HMJMRI: address 'L23' not allowed as Long
        pass

    def serverTypeMessage(self,message):
        # The server can send a "type" to identify itself to clients,
        # useful to enable specific behaviors.
        # HTJMRI and/or HtJMRI v4.19.8 My JMRI Railroad
        pass

    def stopSecondsMessage(self,message):
        # *10
        # The number after '*' indicates how often your throttle will need
        # to send a command or heartbeat (0 means a heartbeat is not required).
        pass

    def setHeartbeatMonitoring(self,state=True):
        # off by default in JMRI
        # *+ Turn heartbeat monitoring on
        # *- Turn heartbeat monitoring off
        pass

    def wiThrottleProtocolVersionMessage(self,message):
        # provides the version number of the WiThrottle protocol.
        # VN2.0
        pass

    def trackPowerNotification(self,message):
        # Returns information about the current state of track power. 
        if len(message) >= 4:
            if message[3] == '0':
                self.power = 'OFF'
            if message[3] == '1':
                self.power = 'ON'
            if message[3] == '2':
                self.power = 'UNKNOWN'
        self.print_trace(f"TRACK POWER IS {self.power}")

#################################################
# Turnout methods

    def turnoutRequest(self,turnout_name,turnout_state):
        # Turnout Request sent from client to server, requesting a new state for a turnout.
        # PTA C LT92
        pass

    def turnoutNotification(self,response):
        # Turnout Notification sent from server to client, advising the new state of a turnout.
        # This is sent for all changes to known turnouts, whether requested by this client or elsewhere.
        # PTA 2 LT92
        pass

#################################################
# Route methods

    def routeRequest(self,route_system_name):
        # Route Request sent from client to server, requesting the route be "set".
        # PRA 2 IO_RESET_LAYOUT
        # Always '2'=Set route
        pass

    def routeNotification(self,response):
        # Route Notification sent from server to client, advising the new state of a route.
        # This is sent for all changes to known routes, whether requested by this client or elsewhere.
        # PRA 2 IO_RESET_LAYOUT
        # Current state, can be '2'=Active, '4'=Inactive or '8'=Inconsistent
        pass

#################################################
# Consist methods

    @classmethod
    def addToConsist(cls,consistid,locoid):
        # Either creates a consist or adds a locomotive to an existing consist.
        # RC+<;>S74<;>My consist<:>L341<;>true
        # RC+<;>S74<;>My consist<:>L342<;>true
        pass

    @classmethod
    def removeConsist(cls,consistid):
        # Deletes a consist.
        # RCR<;>S74 deletes the consist with DCC address S74.
        pass

    def removeLocoFromConsist(self,consistid,locoid):
        # Removes a single locomotive from the consist.
        # RC-<;>S74<:>L341
        pass

    def changeLocoPositionInConsist(self,orderlist):
        # This command is used to change the order of locos in a consist.
        # it just has a list of locomotive DCC addresses.
        # RCP<;>S74<:>L346<;>L3374
        pass

    def setLeadLocoForConsist(self,locoid1,locoid2):
        # These two commands ('c' ou 'C') both allow you to set the lead loco to which
        # function commands will be sent.
        # 'C' consist
        # 'c' consist lead from roster entry
        # This is used in cases where you're not using CV21 and CV22 to determine
        # how locos in an advanced consist should respond to functions.
        # MTAL341<;>CL346
        pass

#################################################
# Loco methods
# MTA commands (Locomotive Action)

    def set_function(self,locoid,functionid,state):
        # locoid is a string like S2,S3,S8,L8504
        # functionid is INTEGER
        # state is INTEGER
        # MTAL8504<;>F02
        print(f"OK Setting function {functionid} of {locoid} to {state}")
        reg_data = f"MXA{locoid}<;>F{state}{functionid:02d}"
        self.sendwt(reg_data)

    def functionNotification(self,locoid,value):
        # locoid is a string like S2,S3,S8,L8504
        # value = sring 0|1n or 0|1nn
        # MTAL8504<;>F02
        # self.print_trace(f"SETTING FUNCTION OF LOCO {locoid} TO {value}")
        state = value[0] # '0' or '1' (string)
        idfun = 'F'+value[1:] # one or two digits from '0' to '68' (string)
        self.print_trace(f"FUNCTION {idfun} OF LOCO {locoid} IS {state}")
        self.registered_locomotives[locoid].set_function(idfun,state)

    def set_speed(self,locoid,speed):
        # Set the speed to a value between 0 and 126.
        # locoid is a string like S2,S3,S8,L8504
        # speed is INTEGER (requested speed)
        # MTA*<;>V30
        speed = max(0,min(126,speed))
        reg_data = f"MXA{locoid}<;>V{speed}"
        print(f"OK Setting speed of {locoid} to {speed}")
        self.sendwt(reg_data)
        # necessary because JMRI sends notifications to all other devices
        # with the same loco registered, except us!
        self.registered_locomotives[locoid].speed = str(speed) # string

    def query_speed(self,locoid='*'):
        # Used to request the current values of speed (V)
        # locoid is a string like S2,S3,S8,L8504
        # locoid could be 'star' to address all locomotives
        # M0A*<;>qV
        print(f"OK Querying speed of {locoid}")
        reg_data = f"MXA{locoid}<;>qV"
        self.sendwt(reg_data)

    def get_speed(self,locoid):
        return self.registered_locomotives[locoid].speed # string

    def speedNotification(self,locoid,value):
        # locoid is a string like S2,S3,S8,L8504
        # value is string
        # The server responds with the Throttle Change Notification
        # M0AS3<;>V25
        # If the value received from the WiThrottle Server is negative
        # (e.g., during the initialization information when an address is selected),
        # that indicates the address is in Emergency Stop mode.
        self.print_trace(f"SPEED OF LOCO {locoid} IS {value}")
        self.registered_locomotives[locoid].speed = value

    def set_direction(self,locoid,direction):
        # locoid is a string like S2,S3,S8,L8504
        # direction is INTEGER
        # - Forward = 1
        # - Reverse = 0
        if direction in [0,1]:
            print(f"OK Changing direction of {locoid} to {direction}")
            reg_data = f"MXA{locoid}<;>R{direction}"
            self.sendwt(reg_data)
        # not necessary to update data in Locomotive object because JMRI
        # is always sending notifications to us

    def query_direction(self,locoid='*'):
        # Used to request the current values of direction (R)
        # locoid is a string like S2,S3,S8,L8504
        # locoid could be 'star' to address all locomotives
        # M0A*<;>qR
        print(f"OK Querying direction of {locoid}")
        reg_data = f"MXA{locoid}<;>qR"
        self.sendwt(reg_data)

    def get_direction(self,locoid):
        direction = self.registered_locomotives[locoid].direction # string
        return "forward" if direction == '1' else "reverse"

    def directionNotification(self,locoid,value):
        # locoid is a string like S2,S3,S8,L8504
        # value in ['0','1']
        # The server responds with the Throttle Change Notification
        # M0AS3<;>R1
        self.print_trace(f"DIRECTION OF LOCO {locoid} IS {value}")
        self.registered_locomotives[locoid].direction = value # string
    
    def emergency_stop(self,locoid='*'):
        # locoid is a string like S2,S3,S8,L8504
        # no documentation provided
        # MTAL8504<;>X
        print(f"OK EMERGENCY STOP requested for {locoid}")
        reg_data = f"MXA{locoid}<;>X"
        self.sendwt(reg_data)

    def throttleChangeNotification(self,tuple):
        # tuple is (client_code,locoid,property,value)
        # There are cases where throttle property changes in JMRI cause a message to be sent.
        # These messages are not a response to a command, but rather are sent at any time
        # (from the perspective of your WiThrottle protocol client).
        # Format: M0AL341<;>F10
        # All property change messages begin with M0A (where '0' could be a different throttle
        # character) and then the DCC address of the locomotive.
        # -Function state: Fsnn
        # -Speed: Vnnnn
        # -Direction: Rd
        # -Speed step mode: sn
        if tuple[2] == 'V':
            self.speedNotification(tuple[1],tuple[3])
        if tuple[2] == 'F':
            self.functionNotification(tuple[1],tuple[3])
        if tuple[2] == 'R':
            self.directionNotification(tuple[1],tuple[3])
        if tuple[2] == 's':
            pass

#################################################

class Locomotive(object):

    def __init__(self,locoid):
        self.locoid = locoid # string like S2,S3,S8,L8504
        self.speed = None # string
        self.direction = None # string in ['0','1']
        self.functions = {} # {'F0': '0', 'F1': '0', 'F2': '0', 'F3': '0',...}
        self.f069 = f"{'x'*69}" # string representation of 69 functions managed by JMRI

    def set_function(self,function_name='F0',function_state='0'):
        self.functions[function_name] = function_state
        function_index = int(function_name[1:])
        self.f069 = self.f069[:function_index] + function_state + self.f069[function_index+1:]

    # DCC-EX example : LocoId:8504 Dir:Reverse Speed:64 F0-28:11000000000000000000000000000
    def display(self):
        direction = 'Forward' if self.direction == '1' else 'Reverse'
        print(f"|{self.locoid:.<5s}| {direction} |{self.speed:.>5s}| {self.f069}")
        # print(f"{self.functions}")

    def __repr__(self) -> str:
        return f"Locomotive({self.locoid})"

#################################################

def myThreads():
    return list(map(lambda t: t.name, threading.enumerate())) # map basicaly returns an object

def help(pwt):
    print(f"Commands for {pwt.client_name}:")
    print(f'{'-'*50}')
    print(f"q:   Quit")
    print(f"h:   Help")
    print(f"v:   Verbose")
    print(f"t:   display Throttle manager")
    print(f"r:   display Roster")
    print(f"add: (+) Add/register Loco within the throttle")
    print(f"rem: (-) Remove Loco from the throttle")
    print(f"cs:  Change Speed")
    print(f"cd:  Change Direction")
    print(f"pb:  Push Button function ")
    print(f"es:  Emergency Stop")
    print(f"po:  Pouet")
    print(f"qs:  Query Speed (all locos)")
    print(f"qd:  Query Direction (all locos)")
    print(f"gs:  Get Speed value")
    print(f"gd:  Get Direction value")
    print(f"speed S|L<dcc-address> <requested-speed>")
    print(f'{'-'*50}')
    # print(f"1:  stop wiThrottle thread (socket remains connected)")

#################################################

if __name__ == '__main__':

    myverbose = False
    pwt = wiThrottleManager("MraThrottle",TCP_IP,TCP_PORT,myverbose)
    # pwt.register_throttle()
    ev0 = threading.Event()
    threading.Thread(target=wiThrottleManager.readwt,args=(pwt,ev0),name="wiThrottle").start()
    # t4 = threading.Thread(target=readwt,args=(pwt,),name="wiThrottle").start()
    # t4 = threading.Thread(target=wiThrottleManager.test,args=(pwt,),name="wiThrottle").start()
    help(pwt)
    try:
        while True:
            print("")
            # print(f"{scanning=}, event_is_set={ev0.is_set()}, threads={myThreads()}")
            if len(pwt.registered_locos) == 0:
                print(f"{CRED}No locomotive registered!{CEND}")
            else:
                print(f"{CGRE}Registered locomotives: {pwt.registered_locos}{CEND}")
            cmd = input(">>> cmd (q,h,v,t,r,add,+,rem,-,cs,cd,pb,es,po,qs,qd,gs,gd,speed) >>>\n")

            if len(cmd) > 0:
                print(f"cmd is {cmd}")
            if cmd in ['q','quit','exit']:
                break
            if cmd in ['h','?','help']:
                help(pwt)
            if 'v' == cmd: # verbose
                myverbose = not myverbose
                print(f"OK Setting verbose to {myverbose}")
                pwt.set_verbose(myverbose)
            if '1' == cmd:
                scanning = False
                ev0.set()
                print("Stopping wiThrottle Thread...")
            if 't' == cmd: # throttle
                pwt.display()
            if 'r' == cmd: # roster
                pwt.print_roster()

            if cmd in ['add','+']: # register from roster
                if len(pwt.rosterkeys) == 0:
                    pwt.print_error(f"OUPS Roster empty!")
                    continue
                pwt.print_roster() # TODO: roster w/o registered locos...
                txt = input(f"-> Loco Index [1-{len(pwt.rosterkeys)}]: ")
                isn = re.match(r"\d+",txt)
                if isn is None:
                    pwt.print_error(f"OUPS Index not integer!")
                    continue
                num = int(txt)
                if num < 1 or num > len(pwt.rosterkeys):
                    pwt.print_error(f"OUPS Index {num} out of bounds!")
                    continue
                pwt.add_locomotive(pwt.rosterkeys[num-1])

            if 'speed' in cmd: # speed L8504 64
                regex = re.search(r"^speed\s+([SL]\d+)\s+(\d+)",cmd)
                if regex is not None and len(regex.groups()) == 2:
                    rdccad = regex.group(1) # requested loco dcc-address
                    rspeed = int(regex.group(2)) # requested speed
                    if rdccad not in pwt.rosterkeys:
                        pwt.print_error(f"OUPS LocoID {rdccad} does not exist in the roster!")
                        continue
                    if rdccad not in pwt.registered_locos:
                        pwt.print_error(f"OUPS LocoID {rdccad} not registered!")
                        continue
                    if rspeed < 0 or rspeed > 126:
                        pwt.print_error(f"OUPS Requested speed {rspeed} out of bounds [0-126]!")
                        continue
                    else:
                        pwt.set_speed(rdccad,rspeed)
                else:
                    pwt.print_error(f"OUPS Bad request! (use speed S|L<dcc-address> <requested-speed>)")

            if 'qs' == cmd: # query speed all locos
                # could ne necessary to get up-to-date speed
                # JMRI does not send regularly data about speed...
                pwt.query_speed('*')

            if 'qd' == cmd: # query direction all locos
                # JMRI sends regularly data about direction changes so is it really useful?
                pwt.query_direction('*')

            if cmd in ['rem','-','cs','cd','pb','es','po','gs','gd']:
                if len(pwt.registered_locos) == 0:
                    print("Please, register at least one locomotive!")
                else:

                    txt = input(f"-> Loco Index [1-{len(pwt.registered_locos)}]: ")
                    isn = re.match(r"\d+",txt)
                    if isn is None:
                        pwt.print_error(f"OUPS Index not integer!")
                        continue
                    else:
                        num = int(txt)
                        if num < 1 or num > len(pwt.registered_locos):
                            pwt.print_error(f"OUPS Index {num} out of bounds!")
                            continue

                    onelocoid = pwt.registered_locos[num-1]
                    
                    if cmd in ['rem','-']: # remove loco from throttle
                        pwt.remove_locomotive(onelocoid)
                    if 'cs' == cmd: # change speed
                        txt = input(f"-> Speed [0-126]: ")
                        isn = re.match(r"\d+",txt)
                        if isn is None:
                            pwt.print_error(f"OUPS Speed not integer!")
                            continue
                        else:
                            num = int(txt)
                            if num < 0 or num > 126:
                                pwt.print_error(f"OUPS Speed {num} out of bounds!")
                                continue
                        pwt.set_speed(onelocoid,num)
                    if 'pb' == cmd: # push button (to switch-on AND to switch-off)
                        # SET (1) is relevant for switch buttons AND for push buttons !
                        # if one send push two times, JMRI does send push first and release second
                        # the same for switchs and push buttons
                        txt = input(f"-> Push button [0-28] : ")
                        isn = re.match(r"\d+",txt)
                        if isn is None:
                            pwt.print_error(f"OUPS Button not integer!")
                            continue
                        else:
                            num = int(txt)
                            if num < 0 or num > 28:
                                pwt.print_error(f"OUPS Button {num} out of bounds!")
                                continue
                        pwt.set_function(onelocoid,num,1)
                        time.sleep(1)
                        pwt.set_function(onelocoid,num,0)
                    if 'po' == cmd: # pouet on/off
                        pwt.set_function(onelocoid,2,1)
                        time.sleep(1)
                        pwt.set_function(onelocoid,2,0)
                    if 'gs' == cmd: # get speed
                        print(pwt.get_speed(onelocoid))
                    if 'gd' == cmd: # get direction
                        print(pwt.get_direction(onelocoid))
                    if 'es' == cmd: # emergency stop
                        pwt.emergency_stop(onelocoid)           
                    if 'cd' == cmd: # change direction
                        txt = input(f"-> Direction (f:Forward or r:Reverse) : ")
                        if txt not in ['f','r']:
                            pwt.print_error(f"OUPS Direction \'{txt}\' out of bounds!")
                            continue
                        # rdir = 1 if txt == 'f' else 0
                        pwt.set_direction(onelocoid,1 if txt == 'f' else 0)

    except KeyboardInterrupt:
        print("\nCTRL-C received")
    scanning = False
    ev0.set()
    pwt.disconnectwt()
    if pwt.connected is False:
        print(f"Throttle disconnected and socket closed")
    else:
        pwt.print_error(f"WARNING: socket not closed")
    time.sleep(1)
    # print(f"{scanning=}, threads={myThreads()}")
    print(f"{scanning=}, event_is_set={ev0.is_set()}, threads={myThreads()}")
    sys.exit("EXIT")
