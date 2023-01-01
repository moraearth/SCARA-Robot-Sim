import b0RemoteApi
import time, math
import pyads

class Conveyor:
    def __init__(self, client, plc, nameConveyor):
        self.Client = client
        self.Plc = plc
        self.NameConveyor = nameConveyor
        self.PatihHandle = self.Client.simxGetObjectHandle(self.NameConveyor+'_Path', 
                self.Client.simxServiceCall())
        self.Forwarder = self.Client.simxGetObjectHandle(self.NameConveyor+'_forwarder',
                self.Client.simxServiceCall())
        self.Sensor1 = self.Client.simxGetObjectHandle(self.NameConveyor+'_S1',
                self.Client.simxServiceCall())
        self.Sensor2 = self.Client.simxGetObjectHandle(self.NameConveyor+'_S2',
                self.Client.simxServiceCall())
        
    def vrep_conveyor_visualSimulation(self):
        if self.NameConveyor == "LB1":
            self.Client.simxCallScriptFunction("sysCall_actuation@LB1", "sim.scripttype_childscript",
                    [ ],self.Client.simxDefaultPublisher())
        else:
            self.Client.simxCallScriptFunction("sysCall_actuation@LB2", "sim.scripttype_childscript",
                    [ ],self.Client.simxDefaultPublisher())
        
    def read_conveyorActualVelocity_tcToVrep(self):
        if self.NameConveyor == "LB1":
            beltVelocity = self.Plc.read_by_name("Main.fbLaufband1.LB1_actualVelocity", pyads.PLCTYPE_LREAL)
        else:
            beltVelocity = self.Plc.read_by_name("Main.fbLaufband2.LB2_actualVelocity", pyads.PLCTYPE_LREAL)
        return beltVelocity/1000 # mm/s to m/s

    def read_sensor_vrepToTc(self):
        if self.NameConveyor == "LB1":
            sensor1_val = self.Client.simxGetFloatSignal("LB1_S1_signal", self.Client.simxServiceCall())[1]
            sensor2_val = self.Client.simxGetFloatSignal("LB1_S2_signal", self.Client.simxServiceCall())[1]
            print(sensor1_val, sensor2_val)
            if sensor1_val > 0.0:
                self.Plc.write_by_name("Main.fbLaufband1.LB1_bSens_S1", True, pyads.PLCTYPE_BOOL)
            else:           
                self.Plc.write_by_name("Main.fbLaufband1.LB1_bSens_S1", False, pyads.PLCTYPE_BOOL)
            if sensor2_val > 0.0:
                self.Plc.write_by_name("Main.fbLaufband1.LB1_bSens_S2", True, pyads.PLCTYPE_BOOL)
            else:
                self.Plc.write_by_name("Main.fbLaufband1.LB1_bSens_S2", False, pyads.PLCTYPE_BOOL)
        else:
            sensor1_val = self.Client.simxGetFloatSignal("LB2_S1_signal", self.Client.simxServiceCall())[1]
            sensor2_val = self.Client.simxGetFloatSignal("LB2_S2_signal", self.Client.simxServiceCall())[1]
            print(sensor1_val, sensor2_val)
            if sensor1_val > 0.0:
                self.Plc.write_by_name("Main.fbLaufband2.LB2_bSens_S1", True, pyads.PLCTYPE_BOOL)
            else:           
                self.Plc.write_by_name("Main.fbLaufband2.LB2_bSens_S1", False, pyads.PLCTYPE_BOOL)
            if sensor2_val > 0.0:
                self.Plc.write_by_name("Main.fbLaufband2.LB2_bSens_S2", True, pyads.PLCTYPE_BOOL)
            else:
                self.Plc.write_by_name("Main.fbLaufband2.LB2_bSens_S2", False, pyads.PLCTYPE_BOOL)


    def set_conveyorActualVelocity_tcToVrep(self, velocity):
        if self.NameConveyor == "LB1":
            done=self.Client.simxSetFloatSignal("LB1_setVelocitySignal", velocity, 
                    self.Client.simxServiceCall())
        else:
            done=self.Client.simxSetFloatSignal("LB2_setVelocitySignal", velocity,
                    self.Client.simxServiceCall())
        return done 