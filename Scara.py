import b0RemoteApi
import time, math
import pyads
import os

class Scara:
    def __init__(self):
        self.Plc = self.connect_ads_on_windows()
        self.Client = self.b0RemoteApi_create_client()
        self.m1 = self.Client.simxGetObjectHandle('Joint1', self.Client.simxServiceCall())
        self.m2 = self.Client.simxGetObjectHandle('Joint2', self.Client.simxServiceCall())
        self.m3 = self.Client.simxGetObjectHandle('Joint3', self.Client.simxServiceCall())
        self.m4 = self.Client.simxGetObjectHandle('Joint4', self.Client.simxServiceCall())

    def connect_ads_on_windows(self):
        return pyads.Connection('192.168.0.157.1.1', 851)

    def b0RemoteApi_create_client(self):
        return b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApi')

    def plc_open(self):
        self.Plc.open()

    def plc_close(self):
        self.Plc.close()
    
    def calc_vrepTopy_tcp_pose(self, objectHandle):
        pos = self.Client.simxGetObjectPosition(objectHandle, -1, self.Client.simxServiceCall())[1]
        ori = self.Client.simxGetObjectOrientation(objectHandle, -1, self.Client.simxServiceCall())[1]
        half_height = self.Client.simxGetObjectFloatParameter(objectHandle, 20, self.Client.simxServiceCall())[1]
        return [pos[0], pos[1], pos[2] + half_height, ori[2]]

    def set_pyTotc_tcp_poses_and_grippArr(self, prog_file_name):
        # Fisierul se afla in acelasi director cu exec py
        line_list = []
        list_of_points = []
        list_gripp = [] # 1 prinde 2 elibereaza 0 neutru- ramane comanda precedenta
        f = open(prog_file_name, "r")
        for line in f:
            line_list = line.split()
            if len(line_list) > 2:
                # linia de cod contine un 3Dpunct
                list_gripp.append(0) # gripp ramane pe com neutru
                for coord in line_list:
                    # se adauga coordonata in lista de puncte
                    if self.is_number(coord):
                        list_of_points.append(float(coord))
            else:
                # linia de cod contine o comanda de gripper
                if line_list[1] == "GRIPP_ON":
                    list_gripp.append(3)
                elif line_list[1] == "GRIPP_OFF":
                    list_gripp.append(4)
                elif line_list[1] == "WAIT_SENSOR":
                    list_gripp.append(1)
        for i in range(150 - len(list_of_points)):
            # se umple lista cu valori 0.0 pana la 150 de elemente coresp a max 50 de linii de program
            list_of_points.append(0.0)
        
        for i in range(50 - len(list_gripp)):
            list_gripp.append(5)

        #print(list_of_points, list_gripp)
                        
        self.Plc.write_by_name("Main.fbNciSequence.aPositionList", list_of_points, pyads.PLCTYPE_REAL * 150)
        self.Plc.write_by_name("Main.fbNciSequence.aGrippList", list_gripp, pyads.PLCTYPE_REAL * 50)
        # sterge fisierul temp dupa ce a citit datele din el
        f.close()
        os.remove(prog_file_name)
    
    def is_number(self,string):
        try:
            complex(string)
            return True
        except ValueError:
            return False

    def get_tcTopy_joints_positions(self):
        m1 = self.Plc.read_by_name("Main.m1_pos", pyads.PLCTYPE_LREAL)
        m2 = self.Plc.read_by_name("Main.m2_pos", pyads.PLCTYPE_LREAL)
        m3 = self.Plc.read_by_name("Main.m3_pos", pyads.PLCTYPE_LREAL)
        m4 = self.Plc.read_by_name("Main.m4_pos", pyads.PLCTYPE_LREAL)
        return [m1, m2, m3, m4]

    def get_tcTopy_joints_velocities(self):
        v1 = self.Plc.read_by_name("Main.m1_vel", pyads.PLCTYPE_LREAL)
        v2 = self.Plc.read_by_name("Main.m2_vel", pyads.PLCTYPE_LREAL)
        v3 = self.Plc.read_by_name("Main.m3_vel", pyads.PLCTYPE_LREAL)
        v4 = self.Plc.read_by_name("Main.m4_vel", pyads.PLCTYPE_LREAL)
        return [v1, v2, v3, v4]

    def set_pyTovrep_joints_positions(self):
        joint_positions = self.get_tcTopy_joints_positions()
        teta1 = joint_positions[0] if (joint_positions[0]>0 and joint_positions[0]<180) else joint_positions[0]-360
        teta2 = joint_positions[1] if (joint_positions[1]>0 and joint_positions[1]<180) else joint_positions[1]-360
        d3 = joint_positions[2]
        teta4 = joint_positions[3] if (joint_positions[3]>0 and joint_positions[3]<180) else joint_positions[3]-360
        teta1 = teta1 * math.pi/180
        teta2 = teta2 * math.pi/180
        d3 = self.mm_to_m(d3)
        teta4 = teta4 * math.pi/180
        self.Client.simxSetJointPosition(self.m1[1], teta1, self.Client.simxDefaultPublisher())
        self.Client.simxSetJointPosition(self.m2[1], teta2, self.Client.simxDefaultPublisher())
        self.Client.simxSetJointPosition(self.m3[1], d3, self.Client.simxDefaultPublisher())
        self.Client.simxSetJointPosition(self.m4[1], teta4, self.Client.simxDefaultPublisher())
        #print(teta1, teta2, self.m_to_mm(d3), teta4)

    #def plot_joints_pos(self):
        #joint_positions = self.get_tcTopy_joints_positions()
        #teta1 = joint_positions[0]
        #teta2 = joint_positions[1]
        #d3 = joint_positions[2]
        #teta4 = joint_positions[3]
        #return [teta1, teta2, d3, teta4]
    
    def m_to_mm(self, m):
        return m * 1000

    def mm_to_m(self, mm):
        return mm / 1000

    def vrep_create_cube(self, size, list_color, list_coord):
        return self.Client.simxAddDrawingObject_cubes(size, list_color, list_coord, self.Client.simxServiceCall())
    
    def vrep_remove_cube(self, handleObject):
        return self.Client.simxRemoveDrawingObject(handleObject, self.Client.simxServiceCall())
    
    def vrep_vacuum_on_off(self):
        readSensor_tcToPy = self.Plc.read_by_name("Main.fbNciSequence.bGripper", pyads.PLCTYPE_BOOL)
        if readSensor_tcToPy:
             self.Client.simxSetIntSignal('BaxterVacuumCup_active',1,self.Client.simxServiceCall())
        else:
             self.Client.simxSetIntSignal('BaxterVacuumCup_active',0,self.Client.simxServiceCall())
