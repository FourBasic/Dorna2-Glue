#import modules
from __future__ import print_function
import sys
import json
sys.path.append('..')
from dorna2 import dorna
from array import *
import threading
from time import sleep

#Global Variables
robot = dorna()
sim = 0
usePLC = 1
ioCmd = -1
lastIoCmd = -1
#homePosition = {"cmd":"joint","j0":52.217,"j1":179.982,"j2":-142.029,"j3":-30.679,"j4":-176.299}
#homePosition = {"cmd":"joint","j0":52.217,"j1":179.982,"j2":-142.029,"j3":-30.679,"j4":-176.299,"id":5007} #Robot 1 Home Position
homePosition = {"cmd":"joint","j0":52.3,"j1":180,"j2":-142,"j3":135,"j4":0,"id":5007}
posData = [["Reserved Start", ""], ["Reserved End", ""]]
homingComplete = 0
stepperStat = 0
rCyclePos = 999
rLastposID = -1
whenCompleteID = 0;whenCompleteCyclePos = 0; whenCompleteMess = "Complete" #Containers for Expected Motion Completion ID's 


#-------------------------------Initialization-----------------------------------------------------
def init():
    #Read position files into position registers
    # i is the current index position in posData    
    i = loadPosFile("HomeToPerch",1); i = loadPosFile("PerchToPounce",i); i = loadPosFile("PounceToGlueS1",i); i = loadPosFile("GlueToPerch",i)      

#Places position commands from .txt file into posData
def loadPosFile(fileName="", i=0):
    print("Load " + fileName)
    with open(fileName + ".txt", 'r') as f:
        lines = f.readlines()
        for l in lines:
            if l != "\n":
                cmdWithID = cmdAppendID(l, i)
                posData.insert(i, [fileName,cmdWithID])               
                i += 1
    return i

#Append Command with id
def cmdAppendID(cmd="", idNum=0):
    cmd = cmd.replace("{", "{\"id\":" + str(idNum) + ",")    
    return cmd
#///////////////////////////////////////////////////////////////////////////////////////////////////



#-------------------------------Thread Functions-----------------------------------------------------
#Thread Function for Monitoring
def thread_Monitor(name):
    print("Thread Monitor Started")
    #Init Global Variables Used to Output Status to PLC
    global usePLC, ioCmd, lastIoCmd, homingComplete, rID, rSTAT, rCyclePos, rLastposID; rID = -1; rSTAT = -1;    
    #Main Monitor Loop
    while True:
        #Obtain Command from Inputs
        if usePLC and (sim == 0):
            ioCmd = readCmd()            
        if ioCmd == 10:#(0-Halt) (4-Wait When Done) (7-Homing Request) (1-Recipe 1 Sequence) (2-Recipe 2 Sequence) (3-Recipe 3 Sequence) 
            if haltStatus < 4:#Issue Multiple Halt Commands to Robot
                haltStatus += 1
                arg = {"cmd":"halt","accel":2,"id":5000}; robot.play(**arg)                
                print("------------ ROBOT Halt SENT- Last Known Position ID:", rLastposID, "  ------------")
                sleep(0.05)
        elif ioCmd == 0:
            ioCmd = 4
        elif ioCmd == 1 and not homingComplete:
            ioCmd = 7
        elif ioCmd == 1:
            ioCmd = 1
        else:
            haltStatus = 0 

        if ioCmd != lastIoCmd:
            lastIoCmd = ioCmd
            print(ioCmd)
      
        
        #Obtain Status from Robot Controller
        if all(["id" in robot.sys, "stat" in robot.sys]):
            rID = robot.sys["id"]; rSTAT = robot.sys["stat"]            
        else: 
            rID = -1; rSTAT = -1        
        #Save Last Valid Position ID Reflected Back From Robot
        if (rSTAT > 0) and (rID > 0) and (rID < len(posData)) and (rID != rLastposID):
            rLastposID = rID       

#Thread function for simulating command while offline
def thread_simIO(name):
    global ioCmd, rLastposID, rCyclePos
    while True:       
        sleep(1)
        print("Cycle:",str(rCyclePos)," - Last Command",str(ioCmd), " - Last ID:",rLastposID)
        print("Enter PLC Command")
        try:
            ioCmd = int(input())
        except:
            print("Invalid Entry")
        if (ioCmd == 0) and (rLastposID == -1):
            sleep(1)
            print("Enter Override Last Position ID")
            try:
                rLastposID = int(input())
            except:
                print("Invalid Entry")
    
#Interpret Digital Input Command         
def readCmd():    
    # first check if the keys exist - IF it does not go to Sim
    if all(["in0" in robot.sys, "in1" in robot.sys, "in2" in robot.sys]):        
        #cmd = int(robot.sys["in0"] + robot.sys["in1"] * 2 + robot.sys["in2"] * 4)
        cmd = int(robot.sys["in4"] + robot.sys["in5"] * 2 + robot.sys["in6"] * 4)
        #print(cmd)
        sleep(0.5)
        #Check to see if command value is still the same (relay latency)
        #if cmd == int(robot.sys["in0"] + robot.sys["in1"] * 2 + robot.sys["in2"] * 4):
        if cmd == int(robot.sys["in4"] + robot.sys["in5"] * 2 + robot.sys["in6"] * 4):
            return cmd
        else:
            return -1       
    else:
        cmd = int(False + True * 2 + True * 4)
        return cmd
#///////////////////////////////////////////////////////////////////////////////////////////////////



#-------------------------------Robot Commands------------------------------------------------------
#Execute a sequence of position commands
def playSeq(seqName, posID = 0):
    l_id = -1
    for p in posData:
        if p[0] == seqName and ((posID == 0) or (posData.index(p) >= posID)):
            rPlayTrk(p[1])
            l_id = posData.index(p)
    return l_id
    
#Robot Play Track with additional simulate option to avoid errors while offline testing   
def rPlayTrk(cmd=""):    
    if sim == 0:
        print("Send: " + str(json.loads(cmd)))
        robot.play(True, **json.loads(cmd))          
    else:
        print("Simulate: " + str(json.loads(cmd)))
        
#Set robot controller default home positions      
def home():
   global rCyclePos, homePosition
   print("Homing Init ->")
   if sim == 0:
       #Set robot position in controller to home
       print("Joint Set ->")
       arg = homePosition
       trk = robot.play(True, **arg)        
       trk.complete()
   rCyclePos = 1
   print("Homing Complete ->", "CyclePos:", str(rCyclePos))
   return True

#Turn stepper motors on/off  
def stepperMotor(set):
    global stepperStat
    if set:
       arg = {"cmd":"motor","motor":1,"id":5009}
    else:
       arg = {"cmd":"motor","motor":0,"id":5008}  
    if sim == 0:
        robot.play(True, **arg)
    stepperStat = set
    print("Steppers ", set)
#/////////////////////////////////////////////////////////////////////////////////////////////////// 



#-------------------------------Robot Motion Sequence-----------------------------------------------
#Resume from a Position ID and complete reamining sequence motions
def resumeFromPosID(posID=0):
    print("Resuming From Last ID:", posID)
    l_id = -1
    seqName = posData[posID][0]
    if seqName == "HomeToPerch":
        homeToPerch(posID)
    elif seqName == "PerchToPounce":
        perchToPounce(posID)
    elif seqName == "PounceToGlueS1":
        pounceToGlueS1(posID)
    elif seqName == "GlueToPerch":
        glueToPerch(posID)

#Check Robot Sys for Matching Values that indicate Task completed
def checkMotionComplete(ID, cyclePos, message="Complete"):
    global sim
    if sim:
        print(message)
        return cyclePos
    else:
        if all(["id" in robot.sys, "stat" in robot.sys]):        
            if (robot.sys["id"] == ID) and (robot.sys["stat"] == 2):
                print(message)
                return cyclePos
            else:
                return 999

#Hold robot at cycle complete until PLC gives Wait When Done command
def waitForFreshCommand():
    if ioCmd != 4:
        print("Waiting For Fresh Command")
    while ioCmd != 4:
        sleep(0.1)
    print("Ready For Command")

def homeToPerch(posID = 0):
    global rCyclePos, whenCompleteID, whenCompleteCyclePos, whenCompleteMess
    rCyclePos = 999
    print("Goto Perch ->")               
    whenCompleteID = playSeq("HomeToPerch", posID)
    whenCompleteCyclePos = 2
    whenCompleteMess = "At Perch -> Last Cmd ID:" + str(whenCompleteID) + " CyclePos:" + str(whenCompleteCyclePos)    

def perchToPounce(posID = 0):
    global rCyclePos, whenCompleteID, whenCompleteCyclePos, whenCompleteMess
    rCyclePos = 999
    print("Goto Pounce ->")               
    whenCompleteID = playSeq("PerchToPounce", posID)
    whenCompleteCyclePos = 3
    whenCompleteMess = "At Pounce -> Last Cmd ID:" + str(whenCompleteID) + " CyclePos:" + str(whenCompleteCyclePos)    
        
def pounceToGlueS1(posID = 0):
    global rCyclePos, whenCompleteID, whenCompleteCyclePos, whenCompleteMess
    rCyclePos = 999
    print("Goto GlueS1 ->")               
    whenCompleteID = playSeq("PounceToGlueS1", posID)
    whenCompleteCyclePos = 4
    whenCompleteMess = "At GlueS1 -> Last Cmd ID:" + str(whenCompleteID) + " CyclePos:"+ str(whenCompleteCyclePos) 
    
def glueToPerch(posID = 0):
    global rCyclePos, whenCompleteID, whenCompleteCyclePos, whenCompleteMess
    rCyclePos = 999
    print("Goto Perch ->")               
    whenCompleteID = playSeq("GlueToPerch", posID)
    whenCompleteCyclePos = 2
    whenCompleteMess = "At Perch -> Last Cmd ID:" + str(whenCompleteID) + " CyclePos:" + str(whenCompleteCyclePos)
#///////////////////////////////////////////////////////////////////////////////////////////////////


#----------------------------------------Main-------------------------------------------------------
def main(config_path):
    global homingComplete, rCyclePos
    try:
        print("Init")
        init()        
        #Connect to Robot
        print("Init Connect")
        with open(config_path) as json_file:
            arg = json.load(json_file)
            print("JSON Loaded ", arg["ip"], " ", arg["port"])
        if sim == 0:
            print("Connecting...")
            robot.connect(arg["ip"], arg["port"])
            print("Connected ->")
        else:
            print("Sim Mode")
    except:
        print("Exception")
        robot.close()
        sys.exit()
    #Start Monitoring Thread
    thMonitor = threading.Thread(target=thread_Monitor, args=(1,), daemon=True)
    thMonitor.start()
    #Start Simulate IO Thread for Testing Without PLC
    if usePLC == 0:
        thSimIO = threading.Thread(target=thread_simIO, args=(2,), daemon=True)
        thSimIO.start()
    sleep(1)
    haltStatus = 0      
    print("Entering Main Loop")
    while True: 
    # IO Command (0-Halt) (4-Wait When Done) (7-Homing Request) (1-Recipe 1 Sequence) (2-Recipe 2 Sequence) (3-Recipe 3 Sequence)
    # Cycle Position (999-@Unknown) (1-@Home) (2-@Perch) (3-@Pounce) (4-@GlueS1)
        if ioCmd == 0:
            if haltStatus == 0:
                haltStatus = 1
                print("------------ MAIN Halted- Last Known Position ID:", rLastposID, "  ------------") 
        elif ioCmd == 5555:
            if sim == 0:
                robot.close()
            sys.exit()
        elif homingComplete == False:            
            if ioCmd == 7:#Homing Requested           
                homingComplete = home()
                if True:
                    stepperMotor(1)
        elif stepperStat:
            if haltStatus == 1:
                haltStatus = 0
                if rCyclePos == 999:#Currently @Fucked
                    resumeFromPosID(rLastposID)
            elif rCyclePos != 999:
                if rCyclePos == 1:#Currently @Home
                    homeToPerch()
                elif (rCyclePos == 2) and (ioCmd == 1):#Currently @Perch
                    perchToPounce()
                elif rCyclePos == 3:#Currently @Pounce
                    pounceToGlueS1()
                elif rCyclePos == 4:#Currently @GlueS1
                    glueToPerch()
                    if ioCmd != 4:
                        waitForFreshCommand()
            else:
                rCyclePos = checkMotionComplete(whenCompleteID, whenCompleteCyclePos, whenCompleteMess)




#Maybe ask dorna about TCP offset or check the command sent out by lab.
#for some reason continous motion is not longer working, may need to look into hoe the commands are issued. My need to pack them back in a list
#add another dimension to the postion array that indicates glue recipe 0off1,2,3 and add glue command to txt file




if __name__ == '__main__':
    main("config.json")


