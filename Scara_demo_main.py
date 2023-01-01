import b0RemoteApi
import time, math
import pyads 
from PySide2.QtWidgets import (QApplication, QWidget, QTextEdit,
            QPushButton, QLabel, QMessageBox, QFileDialog, QVBoxLayout,
            QHBoxLayout, QMainWindow)
from PySide2.QtGui import QPixmap
from PySide2.QtCore import QObject
import sys, traceback
import matplotlib
from matplotlib.backends.backend_qt5agg import(FigureCanvas, 
    NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure

from Scara import  Scara 
from Conveyor import Conveyor
import numpy as np
import concurrent.futures, threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections
import os

class MainApplication(QWidget, Scara, Conveyor):
    def __init__(self):
        QWidget.__init__(self)

        self.Run_program = False
        self.joint1_positions = np.zeros(0)
        self.joint2_positions = np.zeros(0)
        self.joint3_positions = np.zeros(0)
        self.joint3_positions = np.zeros(0)
        self.step_positions = 0
        self.prog_file_name = "prog_file_hidden_temp.txt"

        # Widgets
        self.textEdit = QTextEdit()
        self.newButton = QPushButton("New")
        self.openButton = QPushButton("Open")
        self.saveButton = QPushButton("Save")
        self.startButton = QPushButton("Start")
        self.stopButton = QPushButton("Stop")
        self.fotoLabel = QLabel()
        self.foto = QPixmap("ScaraFoto.png")
        self.fotoLabel.setPixmap(self.foto)
        self.programLabel = QLabel()
        self.manualButton = QPushButton("Manual")
        self.positionsButton = QPushButton("Positions")
        self.velocitiesButton = QPushButton("Velocities")
        self.accelerationsButton = QPushButton("Accelerations")
        # Initial Values and Properties Assignments
        self.programLabel.setText("Program:")
        #self.textEdit.setFontPointSize(12)
        self.textEdit.setText(" ")
        self.setWindowTitle("SCARA - DEMO - MORA")
        
        # Style
        self.setStyleSheet(open("Scara_demo_main.css").read())
        self.textEdit.setFixedWidth(400)
        self.textEdit.setFixedHeight(600)

        # Layout
        hBox1, hBox2, hBox3 = QHBoxLayout(), QHBoxLayout(), QHBoxLayout()
        vBox1, vBox2, vBox3 = QVBoxLayout(), QVBoxLayout(), QVBoxLayout()
        for widget in (self.newButton, self.openButton, self.saveButton): hBox1.addWidget(widget)
        for widget in (self.fotoLabel, self.startButton, self.stopButton, self.positionsButton, 
                self.velocitiesButton): vBox1.addWidget(widget)
        vBox2.addWidget(self.textEdit)
        vBox2.addLayout(hBox1)
        for layout in (vBox2, vBox1): hBox3.addLayout(layout)
        for layout in (hBox3, hBox2): vBox3.addLayout(layout)
        self.setLayout(vBox3)

        # Connections
        self.openButton.clicked.connect(self.open_file)
        self.saveButton.clicked.connect(self.save_file)
        self.newButton.clicked.connect(self.new_file)
        self.startButton.clicked.connect(self.start_program)
        self.stopButton.clicked.connect(self.stop_program)
        self.positionsButton.clicked.connect(self.view_joints_positions)
        self.velocitiesButton.clicked.connect(self.view_joints_velocities)
        
        self.show()

    def closeEvent(self, event):
        if self.textEdit.toPlainText() != " ":
            reply = QMessageBox.question(self, 'Message',
                    "Are you sure to quit without Saving ?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()
            self.save_file()
            event.accept()
    
    def new_file(self):
        if self.textEdit.toPlainText() != " ":
            reply = QMessageBox.question(self, 'Message',
                    "Are you sure to open new file without Saving ?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.textEdit.setText(" ")
            else: 
                self.save_file()
    
    def open_file(self):
        fname = QFileDialog.getOpenFileName(self, 'Open file', 'c://')
        if fname[0]:
            f = open (fname[0], 'r')
            with f:
                data = f.read()
                self.textEdit.setText(data)

    def save_file(self):
        if self.textEdit.toPlainText() != " ":
            fname = QFileDialog.getSaveFileName(self, 'Save File')
            data = self.textEdit.toPlainText()
            f = open(fname[0], 'w')
            f.write(data)
            f.close()

    def start_program(self):
        if not self.Run_program: 
            if self.textEdit.toPlainText() == " ":
                msgBox = QMessageBox()
                msgBox.setText("Open a previously saved program or write a new program ")
                msgBox.exec()
            else:    
                self.Run_program = True
                self.plotLength = 100
                self.q_j1_pos = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j2_pos = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j3_pos = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j4_pos = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j1_vel = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j2_vel = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j3_vel = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.q_j4_vel = collections.deque([0]*self.plotLength, maxlen=self.plotLength)
                self.Mainloop_thread = threading.Thread(target=self.mainloop)
                self.Mainloop_thread.start()

    def stop_program(self):
        self.Run_program = False
    
    def start_sym_vrep(self):
        self.scara.Client.simxStartSimulation(self.scara.Client.simxDefaultPublisher())

    def stop_sym_vrep(self):
        self.scara.Client.simxStopSimulation(self.scara.Client.simxDefaultPublisher())

    def mainloop(self):
        self.scara = Scara()
        self.conveyor1 = Conveyor(self.scara.Client, self.scara.Plc, "LB1")
        self.conveyor1.set_conveyorActualVelocity_tcToVrep(0.0)
        self.conveyor2 = Conveyor(self.scara.Client, self.scara.Plc, "LB2")
        self.conveyor2.set_conveyorActualVelocity_tcToVrep(0.0)
        self.start_sym_vrep()
        # create temp file and send to func
        f = open(self.prog_file_name, 'w')
        data = self.textEdit.toPlainText()
        print(data)
        f.write(data)
        f.close()
        with self.scara.Client:
            time.sleep(1)
            self.scara.plc_open()
            self.scara.Plc.write_by_name("Main.bStart", True, pyads.PLCTYPE_BOOL)
            self.scara.set_pyTotc_tcp_poses_and_grippArr(self.prog_file_name)
            while self.Run_program:
                self.conveyor1.vrep_conveyor_visualSimulation()
                self.conveyor1.read_sensor_vrepToTc()
                conveyor1_velocity = self.conveyor1.read_conveyorActualVelocity_tcToVrep()
                self.conveyor1.set_conveyorActualVelocity_tcToVrep(conveyor1_velocity)
                self.conveyor2.read_sensor_vrepToTc()
                self.conveyor2.vrep_conveyor_visualSimulation()
                conveyor2_velocity = self.conveyor2.read_conveyorActualVelocity_tcToVrep()
                self.conveyor2.set_conveyorActualVelocity_tcToVrep(conveyor2_velocity)
                #self.scara.set_pyTotc_tcp_poses_and_grippArr("SCARA_WorkPoints.txt")
                self.scara.set_pyTovrep_joints_positions()
                self.scara.vrep_vacuum_on_off()
                #joints_pos = self.scara.plot_joints_pos()
                joints_pos = self.scara.get_tcTopy_joints_positions()
                self.q_j1_pos.append(joints_pos[0])
                self.q_j2_pos.append(joints_pos[1])
                self.q_j3_pos.append(joints_pos[2])
                self.q_j4_pos.append(joints_pos[3])
                joints_vel = self.scara.get_tcTopy_joints_velocities()
                self.q_j1_vel.append(joints_vel[0])
                self.q_j2_vel.append(joints_vel[1])
                self.q_j3_vel.append(joints_vel[2])
                self.q_j4_vel.append(joints_vel[3])
                #positions_read = self.scara.format_joints_positions_for_plot()
                #positions = self.scara.format_joints_positions_for_plot()
                #process_callback_positions.emit(positions)
          
            self.scara.plc_close()
            self.stop_sym_vrep()

    def view_joints_positions(self):
        view_joints_pos_obj = viewJointsPositions(
            self.plotLength, self.q_j1_pos, self.q_j2_pos, self.q_j3_pos, self.q_j4_pos)

    def view_joints_velocities(self):
        view_joints_vel_obj = viewJointsVelocities(
                self.plotLength, self.q_j1_vel, self.q_j2_vel, self.q_j3_vel, self.q_j4_vel)
        
class viewJointsPositions(QWidget):
    def __init__(self, plotLength, q_j1_pos, q_j2_pos, q_j3_pos, q_j4_pos):
        super().__init__()
        self.q_j_pos = [q_j1_pos, q_j2_pos, q_j3_pos, q_j4_pos]
        self.plotLength = plotLength
        self.plotMaxLength = 100
        self.numPlots = 4
        self.data = []
        for i in range(self.numPlots):
            self.data.append(self.q_j_pos[i])
        self.plot_thread_pos = threading.Thread(target=self.plot_data(), daemon=True)
        self.plot_thread_pos.start()

    def load_plot_data(self, frame, lines, lineValueText, lineLabel, timeText):
        for i in range(self.numPlots):
            lines[i].set_data(range(self.plotMaxLength), self.data[i])

    def plot_data(self):
        pltInterval = 50 # Period at which the plot animation updates[ms]
        xmin, xmax = 0, self.plotMaxLength
        ymin, ymax = -200, 200
        fig = plt.figure()
        ax = plt.axes(xlim=(xmin,xmax), ylim=(ymin,ymax))
        ax.set_title('Scara Joints Position')
        ax.set_xlabel('Time')
        ax.set_ylabel('Joints Position')
        lineLabel = ['J1', 'J2', 'J3', 'J4']
        #style = ['-', '-.','--', ':']
        style = ['r-','c-','b-','g-']
        #color = ['red', 'green', 'blue', 'black']
        timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
        lines = []
        lineValueText = []
        for i in range(self.numPlots):
            lines.append(ax.plot([],[],style[i], label=lineLabel[i])[0])
            lineValueText.append(ax.text(0.70,0.90-i*0.05,'', transform=ax.transAxes))
        plt.legend(loc='upper left')
        anim = animation.FuncAnimation(
                fig, 
                self.load_plot_data,
                fargs=(lines, lineValueText, lineLabel, timeText),
                interval=pltInterval)
        plt.show()


class viewJointsVelocities(QWidget):
    def __init__(self, plotLength, q_j1_vel, q_j2_vel, q_j3_vel, q_j4_vel):
        super().__init__()
        self.q_j_vel = [q_j1_vel, q_j2_vel, q_j3_vel, q_j4_vel]
        self.plotLength = plotLength
        self.plotMaxLength = 100
        self.numPlots = 4
        self.data = []
        for i in range(self.numPlots):
            self.data.append(self.q_j_vel[i])
        self.plot_thread_vel = threading.Thread(target=self.plot_data(), daemon=True)
        self.plot_thread_vel.start()

    def load_plot_data(self, frame, lines, lineValueText, lineLabel, timeText):
        for i in range(self.numPlots):
            lines[i].set_data(range(self.plotMaxLength), self.data[i])

    def plot_data(self):
        pltInterval = 50 # Period at which the plot animation updates[ms]
        xmin, xmax = 0, self.plotMaxLength
        ymin, ymax = -200, 200
        fig = plt.figure()
        ax = plt.axes(xlim=(xmin,xmax), ylim=(ymin,ymax))
        ax.set_title('Scara Joints Velocity')
        ax.set_xlabel('Time')
        ax.set_ylabel('Joints Velocity')
        lineLabel = ['V1', 'V2', 'V3', 'V4']
        #style = ['-', '-.','--', ':']
        style = ['r-','c-','b-','g-']
        #color = ['red', 'green', 'blue', 'black']
        timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
        lines = []
        lineValueText = []
        for i in range(self.numPlots):
            lines.append(ax.plot([],[],style[i], label=lineLabel[i])[0])
            lineValueText.append(ax.text(0.70,0.90-i*0.05,'', transform=ax.transAxes))
        plt.legend(loc='upper left')
        anim = animation.FuncAnimation(
                fig, 
                self.load_plot_data,
                fargs=(lines, lineValueText, lineLabel, timeText),
                interval=pltInterval)
        plt.show()

if __name__=='__main__':
    
    app = QApplication(sys.argv)
    mainApplication = MainApplication()
    sys.exit(app.exec_())
