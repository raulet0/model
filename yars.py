# yars.py

# 28-05-2023 premier squelette multi-threads
# 14-06-2023 premier programme multi-threads fonctionnel
# 28-07-2023 execution d'un scenario en boucle
# 29-07-2023 debut et fin des scenarios au triage
# 29-07-2023 gares fantomes
# 06-08-2023 premier test reel avec 2 ILS
# 07-08-2023 protection contre double clic/rebond ILS (empeche inversion du sens)
# 08-08-2023 suppression log http
# 09-08-2023 fonction trace
# 11-08-2023 gestion variable instation et retournement possible
# 13-08-2023 inclusion du retour en coulisse dans le scenario
# 27-11-2024 myprog devient yars
# 27-11-2024 homepage / affiche gare courante, /log affiche evenements
# 30-11-2024 tableau de supervision du scenario (comme dans Horloge5)
# 01-12-2024 score simpliste et fuel/ravitaillement
# 01-12-2024 traiter_fonction : traitement de la fonction allumage des feux des locos (score)
# 02-12-2024 action triage
# 03-12-2024 traiter_throttle : traitement de la vitesse/direction temps-réel
# 03-12-2024 ravitaillement possible uniquement quand vitesse nulle
# 05-12-2024 classe Canton
# 06-12-2024 refactoring traiter_capteur
# 07-12-2024 le profil BAR (5556) remplace YAB (ILS)
# 09-12-2024 refactoring et supression ILS (750 lines comments included)
# 11-12-2024 refactoring (790 lines)
# 14-12-2024 control Power et Overload dans traiter_capteur
# 17-12-2024 PriorityQueue
# 18-12-2024 PriorityQueue suite et iFrame (920 lines comments included)
# 21-12-2024 refactoring (930 lines comments included)
# 26-12-2024 get_clock (930 lines)
# 28-12-2024 differenciation anti-rebond entree/sortie zone de detection
# 02-12-2025 asservissement lineraire debouncing
# 02-12-2025 ex. $ egrep -i 'check ds5 . delta' yars.txt
# 03-12-2025 inhibition provisoire critere vitesse nulle pour refueling
# 21-01-2025 integration wiThrottle, menu help (1035 lines)
# 23-01-2025 threading.Event remplace variable globale scanning
# 25-01-2025 refactoring mode trace,debug,error (1080 lines)
# 29-01-2025 calibration mode

# TODO:
# - isoler un module pour le serveur web
# - rapprocher locoid du throttle manager (ex. L8504) et celui de DCC-EX (8504)
# - gerer la loco principale -> impact calcul dmi/dme + pouet + panne fuel
# - methodes privees __method_(self):

"""
https://stackoverflow.com/questions/5419888
I would recommend looking at David Beazley's Generator Tricks for Python,
especially Part 5: Processing Infinite Data.
It will handle the Python equivalent of a tail -f logfile command in real-time
http://www.dabeaz.com/generators/

For scanning DCC++ commands:
----------------------------
When launching PanelPro:
Open JMRI DCC++ Traffic Monitor window:
- Show timestamps = YES
- Show raw data = NO
- Push Choose log file... -> Users/michel/JMRI/BAR.jmri/monitorLog.txt
- Push Start logging
Check: tail -f Users/michel/JMRI/BAR.jmri/monitorLog.txt
Don't close Window ! (since release 5.8)
(re)start yars.py only after setting up Traffic Monitor logging file.

For interactive testing in a terminal:
--------------------------------------
import yars
import importlib
importlib.reload(yars)
d=yars.Date(ss=59)

Pomme-Option-K or 'clear' command for cleaning Terminal history!

"""

import time # for sleep
import datetime # for current time
import sys # for exit
import os # for path.basename
import threading # for Thread
import http.server # for BaseHTTPRequestHandler, HTTPServer
import math # for trunc
# import http.client # standard library
# import requests # $ python -m pip install requests ### ou $ conda install requests ?
import queue
import requests # $ python -m pip install requests ### ou $ conda install requests ?

from withrottle import wiThrottleManager

import numpy as np
import matplotlib.pyplot as plt

#################################################
# Constantes

HOST = "127.0.0.1" # access limited to local browser
HOST = "192.168.1.15" # to give access outside to iPad or Mac after allowing MacOS
PORT = 58080
REPSCENARIO = "/Users/michel/Documents/modelrailroad/simulateur_ligne_zero_turbo_pascal/"
FICSCENARIO = "SCENAR10.DAT" # Be careful encoding utf-8 in BBEdit
LOGFILE1 = "/Applications/JMRI/FOOTHEBAR.TXT" # created by GlobalVariable.py (see JMRI BAR profile)
LOGFILE2 = "/Users/michel/JMRI/BAR.jmri/monitorLog.txt"
POSHHAR = 3 # dans le fichier scenario
POSMMAR = 6
POSPTAR = 9
POSHHDE = 11
POSMMDE = 14
POSPTDE = 17
POSGARE = 19
TRACKLENGTH = 3.1415 * 800 + 2 * 250 # 3013 mm
KA = 12 # acceleration factor
# NBMAXGARES = 20
NBFANTOMES = 1 # nombre de gares fantomes entre deux gares reelles (0 possible)
ERROR = True
DEBUG = False
TRACE = False
TRACE_EVENT = False # input event received + event processing
REFRESHRATE = 1 # pages html (secondes)
PLEINFUEL = 500 # increment en litres a chaque remplissage
FREQ_SCANNER = 0.1 # 100 ms = 10 Hz; 0.01 pour 10 ms = 100 Hz
FREQ_QMERGER = 0.1 # 100 ms = 10 Hz
URLCLOCK = "http://localhost:12080/json/memory/IMCURRENTTIME"
# =================================================================================
# TODO: ASSERVIR LES DELAIS A LA VITESSE DU TRAIN -> traiter_capteur
# =================================================================================
DEBOUNCING0 = 30001 # ms
DEBOUNCING1 =  5001 # ms
# =================================================================================
# https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
CRED,CGRE,CORA,CBLU,CEND = '\033[91m','\033[32m','\033[33m','\033[94m','\033[0m'
KPENTE = 96 # value in [1,126] => pente asservissement anti-rebond vs vitesse (speed_dcc)
WTS_HOST = "localhost" # wiThrottle Server served by JMRI
WTS_PORT = 12090
RECURRENT_EVENTS = (6,) # Power status events each 30 sec

#################################################
# Variables globales

flagerror = ERROR
flagdebug = DEBUG
flagtrace = TRACE
flagevent = TRACE_EVENT
withrottle_verbose = TRACE
processing = True # False stops event processing from Priority Queue
clock = "00:00"
duree_min_interieur = DEBOUNCING1 # when sensor state is 1 so train entering
duree_min_exterieur = DEBOUNCING0 # when sensor state is 0 so train exiting
calibration_mode_5556 = False # locomotive calibration mode with one 5556
calibration_mode_2ILS = False # locomotive calibration mode with 2x ILS
calibration_loop = 0 # number of loops needed for acceleration before getting calibration timestamp
calibration_delay = 3 # secondes to wait before changing loco speed
calibration_speed = 40 # DCC speed step in [0-126]
benchmark = None # instance of Benchmark
pwt = None # instance of Python wiThrottleManager

# SIMULATEUR
logtable = [] # list containing all raw lines coming from both logfiles
instation = False # des le depart, les trains arrivent de la coulisse
triage = False # True implique debrayage de traiter_capteur # shunting yard, fiddle yard, staging yard
dcc_speed = 0 # vitesse fournie par DCC-EX
diesel = 0 # fuel disponible dans le reservoir de la loco principale en litres
scoring = 0 # score du joueur
overload = False
power = True

# SCENARIO
numero_gare = 0 # should be zero at startup
numero_fantome = 0 # compteur de gare fantome entre deux gares
distance_cumul = 0 # distance parcourue cumulee
scenario_ended = False
tank_capacity = 1000 # capacite maxi du reservoir en litres (cf. scenario)

#################################################

class Date:
    # jj,hh,mm,ss,ms = 0,0,0,0,0 # class attributes
    def __init__(self,jj=0,hh=0,mm=0,ss=0,ms=0) -> None:
        self.jj = jj
        self.hh = hh
        self.mm = mm
        self.ss = ss
        self.ms = ms
    def __repr__(self) -> str:
        return "<%02d:%02d:%02d>" % (self.hh,self.mm,self.ss)
    def hhmm(self) -> str:
        return "%02d:%02d" % (self.hh,self.mm)
    def heure_ms(self): # return milliseconds
        return (self.jj * 86400 + self.hh * 3600 + self.mm * 60 + self.ss) * 1000 + self.ms
    def heure_diff(self,date): # return +/- milliseconds
        return self.heure_ms() - date.heure_ms()
    @classmethod
    def heure2date(cls,h): # returns a Date object from 'heure' in milliseconds
        hplus = abs(h)
        heure_ss,ms = hplus // 1000, hplus % 1000
        hplus = heure_ss
        jj = hplus // 86400
        hplus = hplus % 86400
        hh = hplus // 3600
        hplus = hplus % 3600
        mm,ss = hplus // 60, hplus % 60
        return cls(jj,hh,mm,ss,ms) # cls could fit with subclass
    @classmethod
    def timestamp_ms(cls,ts): # returns milliseconds in the current day
        # input string like '22:09:16.287' hour, minute, second, millisecond
        hms,ms = ts.split('.')
        h,m,s = hms.split(':')
        sec = 3600 * int(h) + 60 * int(m) + int(s)
        return 1000 * sec + int(ms)

class Gare:
    gares = [] # class attribute
    def __init__(self,ligne) -> None:
        self.titre = ligne[POSGARE-1:255] # bigramme + nom + points (extrait brut du scenario)
        self.nom = Gare.titre_gare(self.titre) # nom standard
        hhar = ligne[POSHHAR-1:POSHHAR-1+2]
        mmar = ligne[POSMMAR-1:POSMMAR-1+2]
        self.date_entree = Date(hh=int(hhar),mm=int(mmar))
        hhde = ligne[POSHHDE-1:POSHHDE-1+2]
        mmde = ligne[POSMMDE-1:POSMMDE-1+2]
        self.date_sortie = Date(hh=int(hhde),mm=int(mmde))
        self.contact_entree = ligne[POSPTAR-1:POSPTAR-1+1] # OBSOLETE avec 5556
        self.contact_sortie = ligne[POSPTDE-1:POSPTDE-1+1] # OBSOLETE avec 5556
    def __repr__(self) -> str:
        output = [self.titre,self.date_entree.hhmm(),self.contact_entree,self.date_sortie.hhmm(),self.contact_sortie]
        return "<Gare -> " + ','.join(output) + ">"
    @classmethod
    def append(cls,gare):
        cls.gares.append(gare)
    @classmethod
    def affiche(cls):
        if len(cls.gares) > 0:
            print(F"{FICSCENARIO} {len(cls.gares)} gares")
            for gare in cls.gares:
                print(gare)
    @classmethod
    def titre_gare(cls,lieu): # fonction utilisee uniquement par le constructeur de Gare
        return lieu[3:].strip('.')[:-1] if len(lieu) > 3 else "ERREUR-FORMAT-GARE"

class Passage:
    passages = [] # class attribute
    def __init__(self,capteur_id,lieu,gate,date,tita,datu,dist,vmoy,fuel,score) -> None:
        self.capteurid = capteur_id
        self.lieu = lieu # integer {numero de gare}
        self.gate = gate # string in {'ENTREE','SORTIE'}
        self.date = date # Date {heure acceleree transmise par JMRI dans les messages format Date}
        self.tita = tita # Date {heure depart/arrivee TImeTAble}
        self.datu = datu # real {heure systeme courante format universel en millisec}
        self.dist = dist # real {distance cumulee parcourue en millimetres}
        self.vmoy = vmoy # vitesse depuis le precedent passage
        self.fuel = fuel # integer, fuel restant
        self.score = score # integer {score courant cumule}
        self.delta = tita.heure_diff(date) # real {difference +/- date scenario et date reelle}
    def __repr__(self) -> str:
        delay = ('-' if self.delta < 0 else '+') + Date.heure2date(self.delta).hhmm()
        # return "<Passage %s %s %s %s %s %.1fkm %.2fkm/h %dpts %dL %dms>" % \
        return "<Passage %02d %s %s %s %s %03.1fkm %03.1fkm/h %03dpts %03dL %dms>" % \
            (self.lieu,str(self.date),str(self.tita),delay,self.gate,km(self.dist),self.vmoy,self.score,self.fuel,self.datu)
    def report(self):
        station = Gare.gares[self.lieu].titre # bigramme + nom + points
        gate = self.gate[0]
        time = self.date.hhmm()
        delay = ('-' if self.delta < 0 else '+') + Date.heure2date(self.delta).hhmm()
        scoring = self.score
        speed = self.vmoy
        tank = self.fuel
        distance = km(self.dist)
        return "%s%s : %s %s %3d pts %3.0f km/h %4dL %5.1f km" % \
            (station, gate, time, delay, scoring, speed, tank, distance)
    @classmethod
    def append(cls,passage):
        cls.passages.append(passage)
    @classmethod
    def affiche(cls):
        for passage in cls.passages:
            print(passage)
    def maj_penalite_horaire(self):
        # self.score = max(0, self.score - math.trunc(abs(self.delta)/60000))
        pass
    def maj_gain_gare(self):
        self.score = self.score + 5
    def maj_penalite_vitesse(self):
        pass
    def maj_fuel(self,inside_station):
        if inside_station:
            # en moyenne 300 litres pour 100 km, soit 3 litres/km pour une 63000.
            # considérer la vitesse pour avoir une variabilité, sinon pas intéressant !
            # retirer du fuel = vitesse / 3 pour une boucle de 6km entre 25 et 45 km/h en moyenne.
            # plus on roule vite, plus on consomme...
            # de la coulisse a la gare de départ, la vitesse est zero, donc pas de consommation
            self.fuel = max(self.fuel - (self.vmoy / 3), 0)
        # sinon de rien decompter pendant la presence en gare...
        # en attendant de gerer la sortie de gare pour aller faire des manoeuvres de wagons...

class Capteur:
    capteurs = {} # class attribute
    def __init__(self,idcapteur,idsection,timestamp=0) -> None:
        self.id = idcapteur # string 'DS1', 'DS2', etc.
        self.section = idsection # string 'GARE', 'EP1', etc.
        self.heure = timestamp # date/heure du dernier passage en millisecondes
        self.state = None
        self.delay = 1499 # ms (retard R)
    def __repr__(self) -> str:
        return "<Capteur %s %s %s %s %s>" % (self.id,self.section,self.heure,self.state,self.delay)

#################################################

def setup():
    Capteur.capteurs = {"DS5":Capteur('DS5','GARE'),"DS6":Capteur('DS6','EP1')}

def error05(txt):
    if flagerror:
        print(F'{chr(0x20)*5}{CRED}ERROR: {txt} ****{CEND}')

def debug05(txt):
    if flagdebug:
        print(F'{chr(0x20)*5}DEBUG: {txt}')

def trace05(txt,warning=False):
    if flagtrace and not warning:
        print(F'{chr(0x20)*5}TRACE: {txt}')
    if flagtrace and warning:
        print(F'{chr(0x20)*5}{CORA}TRACE: **** WARNING: {txt}{CEND}')

def print05(txt,warning=False):
    if not warning:
        print(F'{chr(0x20)*5}{txt}')
    else:
        print(F'{chr(0x20)*5}{CORA}**** WARNING: {txt}{CEND}')

def event03(txt):
    if flagevent:
        print(F'\n{chr(0x20)*3}{txt}')

def raz():
    global numero_gare,numero_fantome,distance_cumul,scoring,diesel,dcc_speed # numbers
    global scenario_ended,instation,triage,overload,power # booleans
    numero_gare = numero_fantome = distance_cumul = scoring = 0
    dcc_speed = 0 # not zero for testing purpose only
    scenario_ended = instation = triage = overload = False # le train ne peut pas partir depuis la gare
    # power = True # TODO: we should request JMRI or it's refreshed every 30sec
    Passage.passages = []
    Capteur.dernier_capteur = None
    diesel = 100 # 10 % of tank_capacity

def lire_scenario(nom_fichier):
    global tank_capacity
    Gare.gares = []
    try:
        with open(nom_fichier, 'r') as fichier:
            lignes = fichier.readlines()
            for ligne in lignes:
                print(ligne.strip())  # Utilise .strip() pour supprimer les sauts de ligne supplémentaires
                if ligne[0] == 'G': # Gare de passage
                    Gare.append(Gare(ligne.strip()))
                if ligne[0] == 'R': # capacite maxi du Reservoir de fuel machine
                    tank_capacity = int(ligne[1:])
                if ligne[0] == 'V': # Vitesse maxi autorisee
                    pass
    except FileNotFoundError:
        print(F"Erreur : Le fichier {nom_fichier} est introuvable.")
    except IOError:
        print(F"Erreur : Impossible de lire le fichier {nom_fichier}.")

def print_mergerq(pq):
    pqsize = pq.qsize()
    print(F"Merger Queue size: {pqsize}")
    if pqsize == 0:
        return
    # If empty() returns True it doesn’t guarantee that a subsequent call to put() will not block. 
    copypq = queue.PriorityQueue()
    while not pq.empty():
        item = pq.get()
        print(item)
        copypq.put(item) # put return None
    while not copypq.empty():
        pq.put(copypq.get())
    if pqsize >= 20:
        print(F"Merger Queue size: {pqsize}")

def follow(thefile,stop):
    thefile.seek(0,2)
    while not stop.is_set():
        line = thefile.readline() # returns empty string if end of file
        if not line:
            time.sleep(FREQ_SCANNER) # 0.1
            continue
        yield line

def compute(filename,pq,stop):
    logfile = open(filename,"r")
    loglines = follow(logfile,stop)
    for line in loglines:
        if "FOO" in line: # keyword to stop thread (backdoor)
            print("break")
            break
        # logtable.append(line) # return None
        # print(logtable)
        filter_event(line,pq)

def prochaine_gare(num_gare):
    return (num_gare + 1) % len(Gare.gares) # 0..nbgares-1

def prochaine_gare_fantome(num_fantome):
    return (num_fantome + 1) % (NBFANTOMES + 1) # 0..NBFANTOMES

def myThreads():
    return list(map(lambda t: t.name, threading.enumerate())) # map basicaly returns an object

def filter_event(line,pq):
    intype,inname = 0,"unknown"
    if ("Throttle" in line):
        intype,inname = 1,"THROTTLE" # TX, not RX => be aware of delay between TX and RX !
        # TX cmd is always there, sometimes no response !!!
        # RX response is 'Loco State' but the same reponse for Throttle and Function
    if ("Function" in line):
        intype,inname = 2,"FUNCTION"
    if ("MYSENSOR" in line):
        intype,inname = 3,"MYSENSOR"
    if ("POWER OVERLOAD" in line):
        intype,inname = 4,"OVERLOAD"
    if ("POWER RESTORE" in line):
        intype,inname = 5,"RESTORED"
    if ("Power Status:" in line): # one message each 30 sec ! (keepalive ?)
        intype,inname = 6,"DCCPOWER"
    if intype != 0:
        process_event(line,intype,inname,pq)

def process_event(line,intype,inname,pq):
    vector = line.split()
    event_timestamp = vector[0][:-1] # timestamp string without last caracter (':')
    # c'est maintenant qu'on peut faire une vraie date...
    today = datetime.datetime.now() # datetime object datetime.datetime(2024, 12, 21, 22, 52, 35, 769681)
    timenow = today.strftime("%H:%M:%S.%f")[:-3] # string '22:52:35.769'
    # event_datestamp = F"{today.year}-{today.month}-{today.day} {event_timestamp}" # string '2024-12-21 20:02:10.888'
    # event_uts = datetime.datetime.fromisoformat(event_datestamp).timestamp() # unix time float like time.time() 1734807730.888
    # event_ums = int(event_uts * 1000) # unix time integer in milliseconds
    if intype not in RECURRENT_EVENTS: # exception for recurrent messages
        event03(F"{timenow} {inname} {vector}")
        # print(line, end='')
    # TODO: appending dcc_speed,scoring,etc. to be considered
    pq.put((event_timestamp,[intype,vector,time.time()])) # tuple in the form: (priority_number, data)

def process_event_queue(pq,stop):
    while not stop.is_set():
        try:
            # cmd 'x' for pausing process to see how it works
            # cmd 'y' for Queue listing
            if not processing:
                # TODO: time.sleep(0.01) # 10 ms
                continue # next boucle without processing

            filter_restore(pq) # TODO

            # get the next item, this one has the lower timestamp so this is the older
            # blocks 100 ms waiting for an item and raises the Empty exception if no item was available
            # Prior to 3.0 on POSIX systems, and for all versions on Windows, if block is true and timeout is None,
            # this operation goes into an uninterruptible wait on an underlying lock.
            # This means that no exceptions can occur, and in particular a SIGINT will not trigger a KeyboardInterrupt.
            timestamp,record = pq.get(timeout=0.1) # Remove and return an item from the queue.
            # timestamp_msec,intype,vector,put_time = record
            intype,vector,put_time = record
            timestamp_msec = Date.timestamp_ms(timestamp) # la seule utilisation de Date.timestamp_ms !!!
            current_time = time.time()

            if (current_time - put_time) > FREQ_QMERGER:

                timenow = datetime.datetime.fromtimestamp(current_time).strftime("%H:%M:%S.%f")[:-3]
                timeput = datetime.datetime.fromtimestamp(put_time).strftime("%H:%M:%S.%f")[:-3]
                if intype not in RECURRENT_EVENTS: # exception for recurrent messages
                    event03(F"{timenow} PROCESSQ {vector} {timeput} [Q was {pq.qsize()+1}]") # +1 before getting record
                logtable.append(' '.join(vector)) # return None
                if intype == 3: # MYSENSOR
                    # ['20:07:14.637:', 'MYSENSOR', 'DS5', '1', '8:12', 'AM']
                    traiter_capteur(vector[2],int(vector[3]),vector[4],vector[5],timestamp_msec)
                if intype == 2: # FUNCTION
                    # ['19:27:23.270:', 'TX:', 'Function', 'Cmd:', 'CAB:', '8504,', 'FUNC:', '0,', 'State:', '1']
                    traiter_fonction(vector[5][:-1],vector[7][:-1],vector[9]) # some without last caracter
                if intype == 1: # THROTTLE
                    # ['14:05:55.002:', 'TX:', 'Throttle', 'Cmd:', 'Address:', '8504,', 'Speed:', '43,', 'Direction:', 'Forward']
                    traiter_throttle(vector[5][:-1],int(vector[7][:-1]),vector[9])
                if intype == 4: # OVERLOAD
                    # 19:22:56.861:  RX: DIAG: TRACK A POWER OVERLOAD 1556mA (max 1497mA) detected after  100msec. Pause   80msec 
                    traiter_overload(1,vector[7],vector[14])
                    last_overload = timestamp_msec
                if intype == 5: # RESTORED
                    # 19:22:56.940:  RX: DIAG: TRACK A POWER RESTORE (after   80msec)
                    # if (timestamp_msec - last_overload) > 100:
                    traiter_overload(0)
                if intype == 6: # POWER
                    # 19:26:28.517:  RX: Power Status: ON ou 19:26:28.517:  RX: Power Status: OFF
                    traiter_power(vector[4])
            else:
                # Réinsérer le record si le délai n'est pas atteint
                pq.put((timestamp,record))
                # attention la reinsertion du record va automatiquement reactiver le get suivant...
                # avec ce délai, le record est testé 10 fois et reste entre 100 ms et 110 ms
                time.sleep(0.01) # 10 ms
        except queue.Empty:
            # code with exception to be compatible with Windows systems
            continue
    
def filter_restore(pq):
    # lire tous les records et les remettre sauf si c'est un RESTORE
    # si en fin de queue il n'y a pas de nouvel OVERLOAD alors remettre le restore
    # a cause du FREQ_QMERGER on est sur qu'il s'y trouve tous les records pour 
    # une duree d'au moins 100 ms entre restore et overload suivant...
    # ATTENTION : il faut un bon reglage des frequences...
    pass

def traiter_overload(state,current=0,duration=0):
    # on pourrait se contenter du champ #6 du vecteur : OVERLOAD ou RESTORE
    global overload
    error05(F"OVERLOAD {state} {current} {duration}") if state == 1 else error05(F"OVERLOAD FINISHED")
    overload = True if state == 1 else False

def traiter_power(state):
    global power
    # trace(F"POWER {state}")
    power = True if state == 'ON' else False

def traiter_throttle(loco,vitesse,direction):
    global dcc_speed,duree_min_exterieur,duree_min_interieur
    dcc_speed = max(0,vitesse) # dcc_speed could be -1 whenever emergency stop requested
    # asservissement LINEAIRE anti-rebond avec la vitesse du train
    # DEBOUNCING0 >> DEBOUNCING1
    #### duree_min_exterieur = DEBOUNCING0 - (DEBOUNCING0-1001)/126 * dcc_speed
    #### duree_min_interieur = DEBOUNCING1 - (DEBOUNCING1-1001)/126 * dcc_speed
    duree_min_exterieur = max(1001,round(DEBOUNCING0 - (DEBOUNCING0-1001)/KPENTE * dcc_speed))
    duree_min_interieur = round(DEBOUNCING1 - (DEBOUNCING1-1001)/126 * dcc_speed)
    # trace05(F"CAB {loco} {vitesse=} {direction=}")
    trace05(F"CAB {loco} {vitesse=} {direction=} {duree_min_exterieur=}ms {duree_min_interieur=}ms")

def traiter_fonction(loco,fonction,state):
    global scoring
    # loco = 8504, 63951, etc.
    # fonction = numero de fonction du throttle
    # state = string 0 ou 1
    trace05(F"CAB {loco} fonction={fonction} state={state}")
    # la fonction 0 est l'allumage des feux
    # un bonus de 100 pts si les feux de la loco sont allumés !
    if fonction == '0' and state == '1': # feux
        scoring = scoring + 100
    if fonction == '0' and state == '0': # feux
        scoring = max(0, scoring - 100)
    if fonction == '4' and state == '1' and instation: # Horn+ # TODO: ghoststation
        scoring = scoring + 10

class Benchmark:
    def __init__(self,locoid,datetime) -> None:
        self.locoid = locoid # string S2,S3,S8,L8504,etc
        self.time = datetime # string HH:MM:SS
        self.speed = {}
        self.looptime = {}
        self.dmi = {}
        self.dme = {}
    def __repr__(self) -> str:
        return F"Benchmark({self.locoid},{self.time})"
    def display(self):
        print05(self.locoid)        
        print05(self.time)        
        print05(F"SPEED (km/h)..: {self.speed}")
        print05(F"LOOPTIME (ms).: {self.looptime}")
        print05(F"DMI (ms)......: {self.dmi}")
        print05(F"DME (ms)......: {self.dme}")
    def get(self):
        return [self.locoid,self.time,self.speed,self.looptime,self.dmi,self.dme]

    def generate_graph(self):
        if len(self.speed) == 0:
            print(F"No data in {self}")
            return
        
        # *** TESTS DE MISE AU POINT ***

        # jeu de données #1
        # self.speed = {10: 61, 50: 44, 90: 50, 126: 49}
        # self.looptime = {10: 15400, 50: 21207, 90: 18768, 126: 19112}
        # self.dmi = {10: 4684, 50: 3414, 90: 2144, 126: 1001}
        # self.dme = {10: 26980, 50: 14897, 90: 2814, 126: 1001}

        # jeu de données #2
        # self.speed = {18: 6, 35: 12, 51: 23, 65: 36, 83: 70, 88: 80, 102: 110, 111: 125, 126: 150}
        # self.looptime = {18: 154000, 35: 89000, 51: 42000, 65: 30000, 83: 15000, 88: 14000, 102: 10000, 111: 9000, 126: 7500}
        # self.dmi = {18: 4430, 35: 3890, 51: 3382, 65: 2969, 83: 2366, 88: 2207, 102: 1763, 111: 1477, 126: 1001}
        # self.dme = {18: 25858, 35: 21945, 51: 18263, 65: 15271, 83: 10898, 88: 9747, 102: 6525, 111: 4453, 126: 1001}

        KSPEED = 1000
        x_speed = list(self.speed.keys())
        y_speed = [value * KSPEED for value in self.speed.values()]  # Multiplier les vitesses par kspeed
        x_looptime = list(self.looptime.keys())
        y_looptime = list(self.looptime.values())
        x_dmi = list(self.dmi.keys())
        y_dmi = list(self.dmi.values())
        x_dme = list(self.dme.keys())
        y_dme = list(self.dme.values())
        # Créer le graphique
        plt.figure(figsize=(8, 6))  # Taille de l'image : 800x600 pixels
        # Ajouter courbes
        # linestyle='--' pour courbe pointillée
        plt.plot(x_speed, y_speed, label=f'Vitesse du train (km/h * {KSPEED})', color='deepskyblue', linewidth=1, linestyle='--')
        plt.plot(x_looptime, y_looptime, label='Durée de boucle (ms)', color='red', linewidth=1)
        # plt.plot(x_dmi, y_dmi, label=f'Durée DMI (ms)', color='hotpink', linewidth=1)
        # plt.plot(x_dme, y_dme, label=f'Durée DME (ms)', color='royalblue', linewidth=2)
        # Configurer les limites des axes
        plt.xlim(0, 128)
        plt.ylim(0, max(max(y_speed), max(y_looptime), max(y_dmi), max(y_dme)))
        plt.title(F'Calibration {self.locoid} du {self.time}')
        plt.xlabel('Vitesse DCC (x)')
        plt.ylabel(F'Valeurs (km/h x {KSPEED} ou ms)')
        plt.grid(True)
        plt.legend()
        # Sauvegarder le graphique en tant qu'image PNG
        plt.savefig('yars.png', dpi=100)  # Résolution de 100 DPI pour 800x600 pixels
        # Afficher un message pour confirmer la sauvegarde
        print("Le graphique a été enregistré sous le nom 'yars.png'.")
        

def increase_speed(locoid,step):
    trace05(F"Increasing speed of {locoid}")
    pwt.set_speed(locoid,dcc_speed+step)
    # pwt.set_function_push_release(locoid,4) # pouet
    pass


def traiter_capteur(capteurname,state,fastclock,am_pm,new_timestamp):
    # capteurname == contact.id in {'DS1','DS2',etc}
    # state integer in [0,1] certainly
    # contact.section in {'GARE','EP1',etc}
    global scenario_ended
    global duree_min_exterieur,duree_min_interieur
    global calibration_mode_5556, calibration_loop
    
    trace05(F"--[{new_timestamp}]--")

    ### ----- LOGIQUE GENERALE APPLICABLE A TOUT CAPTEUR (Gares, EP, etc.)
    
    # TODO: check if get is the correct way to pick an item up
    # Capteur.capteurs[capteurname] ?
    contact = Capteur.capteurs.get(capteurname,None) # instance of Capteur

    if contact == None:
        error05(F"No sensor {capteurname}!")
        return

    if contact.state == state:
        trace05(F"{capteurname} state is the same twice!",warning=True)
        return

    # very first priority: the object's state must be always consistent with the real device
    contact.state = state

    #
    # TODO: use 2x ILS with DS5 as STATION entry and DS6 as YARD entry ! (or 2x 5556)
    #
    # CAPTEUR_ENTRY_STATION = 'DS5' # only DS5 or DS6 on Derek MotorShield
    # CAPTEUR_ENTRY_YARD = 'DS6'
    #
    if calibration_mode_2ILS:
        pass

    #
    # Here is the code for one 5556 linked with the STATION
    # =====================================================
    # not possible to have both station time and yard time without debouncing :-(
    #
    if calibration_mode_5556:
        trace05(F"MODE CALIBRATION (5556)",warning=True)
        if benchmark is None:
            print(F"**** FATAL ERROR: NO BENCHMARK")
            return
        CAPTEUR_ENTRY_STATION = 'DS5'
        if state == 0 and capteurname == CAPTEUR_ENTRY_STATION:
            if calibration_loop == 3:
                print05(F"Waiting {calibration_delay} seconds...")
                calibration_loop = 0
                # increase dcc speed
                if dcc_speed < 126:
                    threading.Timer(calibration_delay,increase_speed,args=(benchmark.locoid,calibration_speed)).start() 
                else:
                    print(F"OK {benchmark} FINISHED, end of calibration!")
                    pwt.set_speed(benchmark.locoid,0)
                    calibration_mode_5556 = False
            return
        if state == 1 and capteurname == CAPTEUR_ENTRY_STATION:
            # waiting few loops before setiing timestamp attribute of Capteur...
            trace05(f"{calibration_loop}")
            if calibration_loop <= 0: # starting from zero
                print05(F"Loop {calibration_loop} : acceleration...")
                calibration_loop = calibration_loop + 1
                return
            if calibration_loop == 1:
                # time to get timestamp
                print05(F"Loop {calibration_loop} : getting data...")
                calibration_loop = calibration_loop + 1
                contact.heure = new_timestamp # store for next time
                return
            if calibration_loop == 2:
                print05(F"Loop {calibration_loop} : storing data...")
                calibration_loop = calibration_loop + 1
                # compute all data
                # ATTENTION : calcul faux quand on passe minuit ! et dtransition peut devenir negatif...
                loop_duration = new_timestamp - contact.heure
                # ATTENTION : dcc_speed could change while feeding dictionaries below
                # because speed is acquired by another thread!
                mydcc = dcc_speed
                mylocospeed = vitesse(TRACKLENGTH,loop_duration) # km/h au 1/87
                if mydcc not in list(benchmark.speed):
                    benchmark.speed[mydcc] = int(mylocospeed) 
                    benchmark.looptime[mydcc] = loop_duration
                    benchmark.dmi[mydcc] = duree_min_interieur
                    benchmark.dme[mydcc] = duree_min_exterieur
        return # end of calibration task, exit from traiter_capteur
    
    # very important for state 0 to be sure of power state:
    # we don't want to deal with state 0 erroneously if power is off or overload on-going
    if (power == False or overload == True): # and state == 0:
        trace05(F"**** WARNING: NO DCC {state=} {power=} {overload=} ****")
        return

    if triage:
        print05(F"MODE TRIAGE",warning=True)
        return

    if scenario_ended:
        trace05(F"SCENARIO FINISHED",warning=True)
        return
    
    if len(Gare.gares) == 0:
        print('\a',end='') # beep
        error05(F"No scenario loaded!")
        return
    
    if diesel <= 0:
        error05(F"No more fuel!") # GAME OVER
        scenario_ended = True
        return        

    # process debouncing
    # ------------------
    # ajusting new_timestamp without 5556 delay (retard R) to be considered
    # Be careful: depending on each Capteur !
    dtransition = new_timestamp - contact.heure
    # ATTENTION : calcul faux quand on passe minuit ! et dtransition peut devenir negatif...
    contact.heure = new_timestamp # store for next time
    # trace(F"DELTA {contact.id}:{dtransition}ms")
    debug05(F"{CBLU}CHECK {contact.id} {state} DELTA {round(dtransition/1000)} secondes ({dtransition} ms){CEND}")
    # asservissement LINEAIRE anti-rebond avec la vitesse du train
    # DEBOUNCING0 >> DEBOUNCING1
    # duree_min_exterieur = round(DEBOUNCING0 - (DEBOUNCING0-1001)/126 * max(dcc_speed,0))
    # dcc_speed could not be -1 because YARS is changing -1 to 0 when receiving event (see traiter_throttle)
    #### ==============================================================================================================
    # duree_min_exterieur = max(1001,round(DEBOUNCING0 - (DEBOUNCING0-1001)/KPENTE * dcc_speed))
    # duree_min_interieur = round(DEBOUNCING1 - (DEBOUNCING1-1001)/126 * dcc_speed)
    #### ==============================================================================================================
    # TODO: BE CAREFUL dcc_speed should come from Queued event like new_timestamp -> process_event
    debug05(F"CHECK {contact.id} {state} speed {dcc_speed} => dmi {duree_min_interieur} ms dme {duree_min_exterieur} ms")
    # TODO: check if calculation is correct for EP
    duree_min = duree_min_exterieur if state == 1 else duree_min_interieur
    if dtransition < duree_min:
        print05(F"{CORA}CHECK {contact.id} {state} Faux contact {contact.section} ({dtransition} < {duree_min} ms)!{CEND}",warning=True)
        return
    else:
        trace05(F"{CGRE}CHECK {contact.id} {state} OK ({dtransition} >= {duree_min} ms){CEND}")

    if contact.section == 'GARE':
        traiter_capteur_gare(capteurname,state,fastclock,am_pm,new_timestamp)
    elif contact.section == 'EP1':
        traiter_capteur_ep(capteurname,state,fastclock,am_pm,new_timestamp)
    else:
        trace05(F"**** WARNING: Section {contact.section} not managed! ****")

def vitesse(distance,temps): # km/h au 1/87
    # distance en mm
    # temps en ms
    return 313.2*distance/temps if temps > 0 else 1234.8 # km/h au 1/87 (or Mach 1)

def km(distance):
    return (distance * 87 * KA) / 1000000 # km reels simules

def traiter_capteur_gare(capteurname,state,fastclock,am_pm,new_timestamp):
    # capteurname == contact.id in {'DS1','DS2',etc}
    # state integer in [0,1]
    # contact.section in {'GARE','EP1',etc}
    # old_timestamp = contact.heure
    global numero_gare,numero_fantome,distance_cumul,scenario_ended,instation,diesel,scoring

    contact = Capteur.capteurs.get(capteurname) # instance of Capteur

    ### ----- LOGIQUE SPECIFIQUE POUR LES GARES

    if state == 1 and instation == True:
        print05(F"INCONSISTENCY - Train already inside the station!",warning=True)
        return
    if state == 0 and instation == False:
        print05(F"INCONSISTENCY - Train already away!",warning=True)
        return

    my_instation = state == 1 # global variable

    my_numero_gare = numero_gare # global variable
    my_numero_fantome = numero_fantome # global variable
    my_departurestation = numero_gare == 0
    my_ghoststation = numero_fantome != 0
    my_arrivalstation = numero_gare == len(Gare.gares)-1 and not my_ghoststation
    my_scenario_ended = my_arrivalstation and not my_instation # global variable
    my_distance_cumul = distance_cumul # global variable
    my_scoring = scoring # global variable
    my_diesel = diesel # global variable
    my_passage = None

    debug05(F"{my_numero_gare=}")
    debug05(F"{my_numero_fantome=}")
    debug05(F"{my_departurestation=}")
    debug05(F"{my_arrivalstation=}")
    debug05(F"{my_ghoststation=}")
    debug05(F"{my_instation=}")
    debug05(F"{my_scenario_ended=}")

    # process station
    my_porte = "ENTREE" if my_instation else "SORTIE"
    if not my_ghoststation:
        print(F"===> {my_porte} GARE {Gare.gares[my_numero_gare].titre} capteur:{contact.id}")
    else:
        print(F".... {my_porte} GARE FANTOME {my_numero_fantome}/{NBFANTOMES}")

    # process distance (be aware if not ghoststation)
    if not my_ghoststation:

        if my_departurestation:
            my_distance = 0
        else:
            # TODO: consider station length between entry/exit timestamps incl. 5556 delay (R) !
            # distance below is oversized increasing calculated speed
            my_distance = TRACKLENGTH * (NBFANTOMES + 1) # one boucle

        # only when entering the station
        if my_instation:
            my_distance_cumul = my_distance_cumul + my_distance

        debug05(F"{int(my_distance)}mm/{km(my_distance):.1f}km,cumul={int(my_distance_cumul)}mm/{km(my_distance_cumul):.1f}km")

    # process speed (be aware if not ghoststation)
    if not my_ghoststation:

        if my_departurestation:
            my_speed = 0
        else:
            # based on Passage in previous station and current timestamp
            # or same speed as entry one
            index = len(Passage.passages) - 1 # last Passage
            if index >= 0:
                old_timestamp = Passage.passages[index].datu
                old_speed = Passage.passages[index].vmoy
                old_lieu = Passage.passages[index].lieu
            else:
                print(F"**** FATAL ERROR: BAD OFFSET")
                my_scenario_ended = True
                return
            force_speed_computation = False
            if not my_instation:
                if old_lieu != my_numero_gare:
                    error05(F"**** WARNING: entry Passage for station #{my_numero_gare} not found => bad speed!")
                    force_speed_computation = True
                else:
                    my_speed = old_speed
            if my_instation or force_speed_computation:
                my_temps = new_timestamp - old_timestamp
                debug05(F"distance={my_distance:.1f}")
                debug05(F"new_timestamp={new_timestamp}")
                debug05(F"old_timestamp={old_timestamp}")
                debug05(F"temps={my_temps}")
                my_speed = vitesse(my_distance,my_temps) # km/h au 1/87

        trace05(F"SPEED: {my_speed:.1f}km/h au 1/87 ({dcc_speed=})")

    # process a new Passage
    if not my_ghoststation:

        # process accelerated time coming with sensor event
        # 12am = midnight
        # 12pm = noon/midday
        # --------------------------
        # 12am -> minus 12
        # 1am-11am -> stay as is
        # 12pm -> stay as is
        # 1pm-11pm -> add 12
        # --------------------------
        # trace(F"{fastclock} {am_pm}")
        time_vector = fastclock.split(':')
        time_hh = int(time_vector[0]) % 12 + (12 if am_pm == 'PM' else 0)
        time_mm = int(time_vector[1])
        time_date = Date(hh=time_hh, mm=time_mm) # instance of Date

        if my_instation: # le train vient d'entrer en gare
            date1 = Gare.gares[my_numero_gare].date_entree # instance of Date
        else:
            date1 = Gare.gares[my_numero_gare].date_sortie # instance of Date

        my_passage = Passage(contact.id,my_numero_gare,my_porte,time_date,date1,new_timestamp,\
            my_distance_cumul,my_speed,my_diesel,my_scoring)
        # update score pour ce passage
        my_passage.maj_gain_gare()
        my_passage.maj_penalite_horaire()
        my_passage.maj_penalite_vitesse()
        my_scoring = my_passage.score # update score courant
        my_passage.maj_fuel(my_instation)
        my_diesel = my_passage.fuel # update reserve de fuel de la loco
        trace05(F"PASSAGE {my_passage}")

        # TODO: deal with empty tank in Terminal and in web board

    # process exit and next station
    if my_scenario_ended:
        # not instation == True (by design)
        print(F"//// GARE TERMINUS //// SCENARIO TERMINE ////")
    else:
        if not my_instation and not my_ghoststation:
            # on sort d'une vraie gare
            my_numero_gare = prochaine_gare(my_numero_gare) # 0..nbgares-1
        if not my_instation:
            my_numero_fantome = prochaine_gare_fantome(my_numero_fantome) # 0..NBFANTOMES
            trace05(F"PROCHAINE GARE = {Gare.gares[my_numero_gare].nom} (#{my_numero_gare})")

    if True:
        numero_gare = my_numero_gare
        numero_fantome = my_numero_fantome
        instation = my_instation
        scenario_ended = my_scenario_ended
        distance_cumul = my_distance_cumul
        scoring = my_scoring
        diesel = my_diesel
        if my_passage is not None:
            Passage.append(my_passage)

    # Old idea:
    # create and push a transformation to be applied in a FIFO
    # contains the instance of Passage and values of global variables :
    # - deltas for numbers (numero_gare, numero_fantome, scoring, tank, disti_cumul, etc.)
    # - values for booleans (instation, scenario_ended)
    # use a FIFO's thread to apply with 1000ms delay
    # what about traces ?
    # what about multi-threading for traiter_capteur ?

    debug05(F"Done")

def traiter_capteur_ep(capteurname,state,fastclock,am_pm,new_timestamp):
    # capteurname == contact.id in {'DS1','DS2',etc}
    # state integer in [0,1]
    # contact.section in {'GARE','EP1',etc}
    contact = Capteur.capteurs.get(capteurname) # instance of Capteur
    error05(F"{capteurname} {contact.section} not managed!")
    # process station
    # process distance (be aware if not ghoststation)
    # process speed (be aware if not ghoststation)
    # process a new Passage
    # process exit and next station

def plein_fuel():
    # impossible de faire le plein en roulant
    # return min(diesel + PLEINFUEL, tank_capacity) if dcc_speed <= 0 else diesel
    return min(diesel + PLEINFUEL, tank_capacity)

#################################################

def html(o):
    return str(o).replace('<','&lt;').replace('>','&gt;')

def body_log():
    content = "<p>" + ' ; '.join(map(lambda t: t.name, threading.enumerate())) + '</p>\n'
    for item in logtable:
        content = content + "<li>" + item
    return content

def body_station():
    if len(Gare.gares) == 0:
        return "<p>PAS DE SCENARIO</p>"
    if scenario_ended:
        return "<p>Retournez en coulisse : scénario terminé (%s) !</p>" % FICSCENARIO
    if diesel <= 0:
        return "<p>RESERVOIR VIDE !</p>"

    if not instation:
        ligne1 = "avancez..."
        ligne2 = "RDV à %s à %s" % (Gare.gares[numero_gare].nom, Gare.gares[numero_gare].date_entree.hhmm())

    else :# so we are inside a station...
        ghoststation = numero_fantome > 0 # boolean -> please use a function...
        nextistation = prochaine_gare(numero_gare) # numero ou indice de la prochaine gare

        if ghoststation:
            ligne1 = "GARE FANTOME"
            prochaine_etape = Gare.gares[numero_gare]
        else:
            ligne1 = "<font color=blue>%s</font>" % Gare.gares[numero_gare].nom
            prochaine_etape = Gare.gares[nextistation] # savoir la prochaine gare avant traiter_capteur en sortie
 
        ligne2 = "Prochaine gare %s à %s" % (prochaine_etape.nom, prochaine_etape.date_entree.hhmm())
        if not ghoststation and nextistation == 0:
            ligne2 = "Retournez en coulisse"
        if not ghoststation:
            ligne2 = ligne2 + " : partez à %s" % Gare.gares[numero_gare].date_sortie.hhmm()
            
    if triage:
        ligne1 = ligne1 + "&nbsp;<font color=red>(attention mode triage)</font>"

    return "<h1>%s</h1><h2>%s</h2>" % (ligne1,ligne2)

def body_allpassages():
    output = "<pre>"
    for passage in Passage.passages:
        output = output + passage.report() + "\n"
    output = output + "</pre>" 
    return output

def body_messages():
    output = "<!-- infos -->"
    # si le reservoir est vide c'est trop tard le joueur a perdu !...
    if diesel > 0 and diesel < tank_capacity / 10:
        output = output + "<font color=red>Attention : fuel inférieur à 10%</font>"
    return output

def phtml(outputstream,string):
    outputstream.write(bytes(string,"utf-8"))

class MyServer(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8") # text/plain
        self.end_headers()
        phtml(self.wfile,"<!DOCTYPE html><html><head><title>Yars</title>")
        if self.path == "/":
            phtml(self.wfile,"<style>")
            phtml(self.wfile,"body {margin:0;padding:0;}")
            phtml(self.wfile,"* {box-sizing:border-box;}")
            phtml(self.wfile,".cont1 {position:relative;float:left;border:0px;overflow:auto;width:100%;min-height:100vh;}")
            phtml(self.wfile,"div iframe {position:absolute;top:0;left:0;width:100%;height:100%;border:none;}")
            phtml(self.wfile,"</style></head>")
            phtml(self.wfile,"<body>")
            phtml(self.wfile,"<div class=\"cont1\"><iframe name=\"yars\" src=\"/board\"></iframe></div>")
        else:
            if self.path == "/board": # http://192.168.1.15:58080/board
                phtml(self.wfile,"<meta http-equiv=\"refresh\" content=\"%d\">" % REFRESHRATE)
            phtml(self.wfile,"</head><body>")
            phtml(self.wfile,"<p>%s Request: %s</p>" % (self.date_time_string(),self.path))
            if self.path == "/board": # http://192.168.1.15:58080/board
                phtml(self.wfile,body_station())
                phtml(self.wfile,body_allpassages())
                phtml(self.wfile,body_messages())
            if self.path == "/log":
                phtml(self.wfile,body_log())
        phtml(self.wfile,"</body></html>")
    def log_request(self,format,*args):
        # super().log_request(format,*args)
        # print("COUCOU") # print something in the terminal for each http request
        pass
            
def mywebstart(myserver):
    myserver.serve_forever() # Handle requests until an explicit shutdown() request.

#################################################

def get_clock(url_api):
    global clock
    try:
        response = requests.get(url_api)
        if response.status_code == 200:
            data = response.json()
            print(F"Clock: {data['data']['value']}")
            clock = data['data']['value']
        else:
            print(F"ERROR {response.status_code}: {url_api}")
        # print(json.dumps(response.json(),indent=2))
    except Exception as e:
        # print("Fatal error:",str(e))
        print(F"{CRED}*** JMRI WebServer not found ***{CEND}")
        print(F"{CGRE}*** JMRI WebServer not found ***{CEND}")
        print(F"{CORA}*** JMRI WebServer not found ***{CEND}")
        print(F"{CBLU}*** JMRI WebServer not found ***{CEND}")

#################################################

def help():
    print(f"Commands for YARS:")
    print(f'{'-'*50}')
    print(f"q:   Quit")
    print(f"h:   Help")
    print(f"v:   Verbose")
    print(f"vwi: Verbose Throttle Manager")
    print(f"d:   Debug")
    print(f"w:   stop HTTPServer")
    print(f"1:   stop File and Queue Threads")
    print(f"2:   stop wiThrottle Thread")
    print(f"b:   display Benchmark")
    print(f"c:   Calibration")
    print(f"f:   reFueling")
    print(f"g:   calibration Graph")
    print(f"p:   display Passages")
    print(f"r:   Read scenario file")
    print(f"s:   display Scenario")
    print(f"t:   switch Triage on/off")
    print(f"var: display global Variables")
    print(f"x:   PriorityQueue processing on/off")
    print(f"y:   display PriorityQueue")
    print(f"z:   RAZ")
    print(f"add: (+) Add/register Loco within the throttle")
    print(f"rem: (-) Remove Loco from the throttle")
    print(f"tm:  display Throttle Manager")

    print(f'{'-'*50}')

#################################################

if __name__ == '__main__':
    setup()
    raz()
    pq = queue.PriorityQueue() # without maxsize limit
    # A threading.Event object wraps a boolean variable that can either be “set” (True) or “not set” (False).
    # The event can be marked as “not set” (whether it is currently set or not) via the clear() function.
    # stop_event = threading.Event() # “not set” state by default.
    ev1 = threading.Event() # stop files and Queue threads
    ev2 = threading.Event() # stop wiThrottle
    threading.Thread(target=compute,args=(LOGFILE1,pq,ev1),name=os.path.basename(LOGFILE1)).start()
    threading.Thread(target=compute,args=(LOGFILE2,pq,ev1),name=os.path.basename(LOGFILE2)).start()
    threading.Thread(target=process_event_queue,args=(pq,ev1),name="DataMerger").start()
    print(F"--> Scanning File {LOGFILE1}")
    print(F"--> Scanning File {LOGFILE2}")
    pwt = wiThrottleManager("MraThrottle",WTS_HOST,WTS_PORT,withrottle_verbose)
    threading.Thread(target=wiThrottleManager.readwt,args=(pwt,ev2),name="wiThrottle").start()
    
    try:
        httpd = http.server.HTTPServer((HOST, PORT), MyServer)
        t4 = threading.Thread(target=mywebstart,args=(httpd,),name="HTTP").start() # Be careful HTTP is key
        print(F"--> HTTPServer started at http://{HOST}:{PORT}")        
    except IOError:
        print(F"ERROR: Failed to start HTTPServer (port {PORT} might be already in use)")
    get_clock(URLCLOCK)
    try:
        while True:
            if len(pwt.get_registered_locos()) == 0:
                print(f"{CRED}No locomotive registered!{CEND}")
            else:
                print(f"{CGRE}Registered locomotives: {pwt.get_registered_locos()}{CEND}")
            cmd = input(">>> cmd (type h,?,help for help menu) >>>\n")
            print(f"cmd is {cmd}")
            if cmd in ['q','quit','exit']:
                break
            if cmd in ['h','?','help']:
                help()
            if 'v' == cmd: # Verbose
                flagtrace = not flagtrace
                print(f"OK Setting trace to {flagtrace}")
            if 'd' == cmd: # Debug
                flagdebug = not flagdebug
                print(f"OK Setting debug to {flagdebug}")
            if 'vwi' == cmd: # Verbose wiThrottle manager
                withrottle_verbose = not withrottle_verbose
                print(f"OK Setting verbose to {withrottle_verbose}")
                pwt.set_verbose(withrottle_verbose)
            if 'w' == cmd:
                httpd.shutdown() # stop thread
                httpd.server_close() # clean resources
                print("HTTPServer stopped")
            if '1' == cmd:
                ev1.set()
                print("File and Queue Threads stopped")
            if '2' == cmd:
                ev2.set()
                print("wiThrottle Thread stopped")
            if 'b' == cmd: # display Benchmark
                print(F"No benchmark available") if benchmark is None else benchmark.display()
            if 'c' == cmd: # Calibration
                calibration_mode_5556 = not calibration_mode_5556
                if calibration_mode_5556 is False:
                    print(f"OK Setting calibration to {calibration_mode_5556}")
                if calibration_mode_5556 is True:
                    if len(pwt.get_registered_locos()) == 0:
                        print(F"Please, register one locomotive for calibration!")
                        pwt.add_locomotive_keyboard()
                    if len(pwt.get_registered_locos()) == 0:
                        # print(F"Sorry, you failed to register one locomotive!")
                        print(F"Calibration aborted!")
                        calibration_mode_5556 = False
                    else:
                        calibration_locoid = pwt.get_registered_locos()[0]
                        print(F" 1) Place {calibration_locoid} in the staging yard")
                        txt = input(F" 2) Hit 'y' to proceed when ready (locomotive will start at speed 10): ")
                        if txt in ['y','yes','Y']:
                            benchmark = Benchmark(calibration_locoid,datetime.datetime.now().strftime("%H:%M:%S"))
                            print(F"OK Setting up {benchmark}")
                            pwt.set_speed(calibration_locoid,10)
                        else:
                            print(F"Calibration aborted!")
                            calibration_mode_5556 = False
            if 'f' == cmd: # reFueling
                if True: # dcc_speed <= 0:
                    diesel = plein_fuel()
                    print(F"Fuel {diesel:.0f}L ({(diesel / tank_capacity):.0%})")
                else:
                    print(F"Please stop for refueling ({dcc_speed=}/126)")
            if 'g' == cmd: # generate benchmark Graph
                print(F"No benchmark available") if benchmark is None else benchmark.generate_graph()
            if 'p' == cmd: # display passages
                Passage.affiche()
            if 'var' == cmd: # display global variables
                print(F"THREADS {myThreads()}")
                print(F"FILES/QUEUE stop={ev1.is_set()} WITHROTTLE stop={ev2.is_set()}")
                print(F"QUEUE {processing=}")
                print(F"Capteur.capteurs={Capteur.capteurs}")
                print(F"{clock=}")
                print(F"{duree_min_interieur=} ms")
                print(F"{duree_min_exterieur=} ms")
                print(F"{triage=}")
                print(F"{numero_gare=}")
                print(F"{numero_fantome=}")
                print(F"{scenario_ended=}")
                print(F"{instation=}")
                print(F"{scoring=:.1f}")
                print(F"{diesel=:.1f} / {tank_capacity}")
                print(F"{dcc_speed=}")
                print(F"DCC {overload=}")
                print(F"DCC {power=}")
                print(F"{calibration_mode_5556=}")
            if 'r' == cmd: # read scenario file
                raz()
                print(REPSCENARIO)
                sfile = input(F"Scenario ({FICSCENARIO}) : ")
                if len(sfile) > 0:
                    FICSCENARIO = sfile
                lire_scenario(REPSCENARIO + FICSCENARIO)
            if 's' == cmd: # display scenario
                Gare.affiche()
            if 't' == cmd: # switch triage on/off
                triage = not triage
                print(F"Triage is {triage}")
            if 'x' == cmd: # PriorityQueue processing switch
                processing = not processing
                print(F"{processing=}")
            if 'y' == cmd: # display PriorityQueue
                print_mergerq(pq)
            if 'z' == cmd:
                print(F"RAZ")
                raz()
            if cmd in ['add','+']: # register from roster
                pwt.add_locomotive_keyboard()
            if cmd in ['rem','-']: # remove loco from throttle
                pwt.remove_locomotive_keyboard()
            if 'tm' == cmd: # throttle manager
                pwt.display()
    except KeyboardInterrupt:
        print("\nCTRL-C received")
    ev1.set()
    ev2.set()
    pwt.disconnectwt()
    print("File and Queue Threads stopped") # TODO: if threads were not stopped before
    if 'HTTP' in myThreads():
        httpd.shutdown() # stop thread
        httpd.server_close() # clean resources
        print("HTTPServer stopped")
    time.sleep(1)
    print(f"stop Files/Queue={ev1.is_set()}, stop wiThrottle={ev2.is_set()}, threads={myThreads()}")
    sys.exit("EXIT")
