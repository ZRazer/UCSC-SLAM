#!/usr/bin/env python
# Author : Joseph Grant
# Welcome to the SLAM
import rospy
from sensor_msgs.msg import PointCloud2
import os
import io
import sys
import csv
import string
import numpy as np
from numpy.linalg import inv
from collections import OrderedDict
import multiprocessing
import matplotlib
matplotlib.use('TkAgg')
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
# implement the default mpl key bindings
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.patches import Ellipse

if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk

if sys.version_info[0] < 3:
    import Queue as queue
else:
    import queue

class StdOutListener():
    def __init__(self, num):
        self.fig = Figure(figsize=(5, 5), dpi=100)
        # black like my soul
        #self.fig.patch.set_facecolor('white')
        self.fig.patch.set_edgecolor('white')
        self.num = num
        self.start_time = None
        self.x = []
        self.y = []
        self.lx = []
        self.ly = []
        self.my_average = []
        self.sub = self.fig.add_subplot(111)
        self.sub.set_xlabel('x (mm)')
        # List probably needs to be changed if any graphs are modified/added
        self.sub.set_ylabel('y (mm)')
        self.sub.spines['bottom'].set_color('black')
        self.sub.xaxis.label.set_color('black')
        self.sub.yaxis.label.set_color('black')
        self.sub.tick_params(axis = 'x', colors = 'black')
        self.sub.tick_params(axis = 'y', colors = 'black')
        self.sub.plot(self.x, self.y, color='blue')       # line stores a Line2D we have just updated with X/Y data
        self.sub.scatter(self.lx, self.ly, color = 'red')
 
    # On_data adds new y val to a set of values and calculates x value based off time
    # method also plots avg X val over time. For now, plots xmin/ymin to show all data
    def on_data(self, x):
        ax = canvas[self.num].figure.axes[0]
        ax.cla()
        self.sub.set_xlabel('x (m)')
        # List probably needs to be changed if any graphs are modified/added
        self.sub.set_ylabel('y (m)')
        self.x.append(x[0,0])
        self.y.append(x[1,0])
        print(self.x)
        self.sub.plot(self.x, self.y, color='blue') 
        self.lx = []
        self.ly = []
        for j in range(int((len(x)-3)/2)):
            self.lx.append(x[3+2*(j),0])
            self.ly.append(x[4+2*(j),0])
        self.lm_actual = self.sub.scatter(self.lx, self.ly, color = 'red')
        # ax.set_ylim([min(self.y)-1, max(self.y)+1])        # update axes to fit the data
        # ax.set_xlim([min(self.x)-1, max(self.x)+1])
        canvas[self.num].draw()

    # This method is used to clear X/Y data and redraw all plots
    def clear_data(self):
        self.start_time = None
        self.x = []
        self.y = []
        self.lx = []
        self.ly = []
        ax = canvas[self.num].figure.axes[0]
        ax.cla()
        ax.set_ylim(0, 1)
        ax.set_xlim(0, 1)
        canvas[self.num].draw()

# Modified readline function from serialutil.py
def _readline(self):
    eol = b'\r'
    leneol = len(eol)
    line = bytearray()
    while True:
        c = self.read(1)
        if c:
            line += c
        if line[-leneol:] == eol:
            break
        else:
            break
    return bytes(line)

#def on_key_event(event):
#    print('you pressed %s' % event.key)
#    key_press_handler(event, canvas, toolbar)

# Basic test method for adding random data to the plots
def updateGraph(out_listener, q, textBoxes):
    global start
    # start is used as a flag to control program operation
    # 0 = Stop all graphing
    # 1 = Run main program for Xbee data
    # 2 = Run random numbers

    if start == 1:
        try:
            data = q.get(block=False)
        except queue.Empty:
            root.after(1000, updateGraph, out_listener, q, textBoxes)
            return
        # This list must be expanded if graphs are added/modified
        out_listener[0].on_data(data)
        textBoxes[0].configure(state = 'normal')
        textBoxes[0].delete('1.0', Tk.END)
        textBoxes[0].insert(Tk.INSERT, "Pose:\nx: " + str(int(data[0,0])) + "\ny: " + str(int(data[1,0])) + "\ntheta: " + str(int(data[2,0]*180/np.pi)))
        textBoxes[0].configure(state = 'disabled')
        textBoxes[1].configure(state = 'normal')
        textBoxes[1].delete('1.0', Tk.END)
        textBoxes[1].insert(Tk.INSERT, "Num landmarks:\n" + str(int((len(data)-3)/2)))
        textBoxes[1].configure(state = 'disabled')
        root.after(1000, updateGraph, out_listener, q, textBoxes)
        return # Return to prevent extra after() call
    elif start == 2:
        for listener in out_listener:
            listener.on_data(np.random.randint(-5,50))
    after_id = root.after(250, updateGraph, out_listener, 0, 0)
    if not start:
        if after_id is not None:
            root.after_cancel(after_id)
        return

# Called when quit button pressed
def _quit():
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate

# Called when clear button pressed
def _clearData(out_listener):
    for listener in out_listener:
        listener.clear_data()

# Called when run button pressed
def _startRun(out_listener, q, textBoxes):
    global start
    start = 1
    updateGraph(out_listener, q, textBoxes)
    

class TkGUI(multiprocessing.Process):
    def __init__(self, q):
        multiprocessing.Process.__init__(self)
        self.q = q

    def run(self):
        global canvas
        global root
        # global startTest
        # startTest = False
        root = Tk.Tk()
        root.wm_title("SLAM Visualization")
        root.configure(background = 'white') # black like my soul

        plt.ion()                            # ion() allows matplotlib to update animations.
        canvas = []
        out_listener = []

        # This and figure size need modification if number of figures greater than 6
        for i in range(1):
            out_listener.append(StdOutListener(i))
            # a tk.DrawingArea
            canvas.append(FigureCanvasTkAgg(out_listener[i].fig, master=root))
            if i < 3:
                vRow = 0
                vCol = i
            else:
                vRow = 1
                vCol = i - 3
            canvas[i].get_tk_widget().grid(row=20*vRow, column=2*vCol, ipadx = 41, columnspan=2, rowspan=20)
            canvas[i].get_tk_widget().configure(background='white',  highlightcolor='black', highlightbackground='black')
            #toolbar = NavigationToolbar2TkAgg(canvas, root)      # Nobody likes toolbars anyway.
            #toolbar.update()
            # I am not completely sure how tkcanvas.grid vs get_tk_widget().grid are different
            canvas[i]._tkcanvas.grid(row=20*vRow, column=2*vCol, ipadx = 41, columnspan=2, rowspan=20)

        # Creates delete, stop, start, and clear button objects
        clearDataButton = Tk.Button(master=root, width=19, bd=1, bg='white', fg='black', text='Clear', command=lambda: _clearData(out_listener))
        startRunButton = Tk.Button(master=root, width=19, bd=1, bg='white', fg='black', text='Run', command=lambda: _startRun(out_listener, self.q, textBoxes))
        quitButton = Tk.Button(master=root, width=19, bd=1, fg='black', bg='white', text='Quit', command=lambda: _quit())

        # Text boxes in 'normal' mode by default. Set to disabled to make uneditable.
        poseText = Tk.Text(root, height=4, fg='black', bg='white', bd=0, highlightthickness=0, width=19)
        poseText.insert(Tk.INSERT, "Pose:\nx:0\ny:0\ntheta:0")
        poseText.configure(state = 'disabled')
        lmTxt = Tk.Text(root, height=2, fg='black', bg='white', bd=0, highlightthickness=0, width=19)
        lmTxt.insert(Tk.INSERT, "Num landmarks:\n0")
        lmTxt.configure(state = 'disabled')
        textBoxes = []
        textBoxes.append(poseText)
        textBoxes.append(lmTxt)

        # Calling grid() to place objects
        poseText.grid(column = 6, row = 0)
        lmTxt.grid(column = 6, row = 1)
        startRunButton.grid(column = 6, row = 17)
        clearDataButton.grid(column = 6, row = 18)
        quitButton.grid(column = 6, row = 19)
        #canvas[0].mpl_connect('key_press_event', on_key_event)

        root.mainloop()
        sys.exit()

def wrap_to_pi(angle):
    if angle > np.pi:
        return (angle-2*np.pi)
    elif angle < -np.pi:
        return (angle+2*np.pi)
    return angle

class SLAM():
    def __init__(self, q):
        # Initialized state vector, covariance, etc 
        self.r_t = 20                                  # Threshold for assosciation
        self.v_r = 0.1                                  # Measurement error ratio
        self.v_b = 0.0005
        self.x = np.array([[0],                 # state vector
            [0],
            [0.029194467]])
        self.P = np.array([[0,0,0],[0,np.pi/4,0],[0,0,np.pi/4]]) # Covariance matrix

        # Process noise intensity matrix 
        self.Gamma = np.array([[15, 1],
            [15, 1],
            [  0, 0.1]])
        self.q = q
        
    def update(self, landmarks, fwd, dT):
        print('Running update with landmarks: ' + str(landmarks))
        dX = -18*np.cos(self.x[2,0]) + 18*np.cos(self.x[2,0] + dT) + fwd*np.cos(self.x[2,0])
        dY = -18*np.sin(self.x[2,0]) + 18*np.sin(self.x[2,0] + dT) + fwd*np.sin(self.x[2,0])
        # Calculate priors for pose and covariance
        xP = np.array([[self.x[0,0] + dX],
            [self.x[1,0] + dY],
            [self.x[2,0] + dT]])
        xP[2,0] = wrap_to_pi(xP[2,0])
        self.x[0:3] = np.array([[self.x[0,0] + dX],
            [self.x[1,0] + dY],
            [self.x[2,0] + dT]])
        self.x[2,0] = wrap_to_pi(self.x[2,0])
        print('Prior: ' + str(xP))

        Phi = np.array([[1, 0, -dY],
            [0, 1, dX],
            [0, 0, 1]])

        r1 = np.matmul(np.matmul(self.Gamma,np.identity(2)), np.transpose(self.Gamma)) # Gamma*Q*Gamma^T
        r2 = np.matmul(np.matmul(Phi,self.P[0:3,0:3]), np.transpose(Phi))         # Phi*P*Phi^T
        self.P[0:3,0:3] = r1 + r2

        if len(self.P) > 3:
            temp = np.matmul(Phi,self.P[0:3,3:len(self.P)])
            self.P[0:3,3:len(self.P)] = temp
            self.P[3:len(self.P),0:3] = np.transpose(temp)
            
        # For every landmark observed, run update or add it to state vector
        for j in range(len(landmarks)):
            r = self.r_t
            # Find landmark using observation model
            num_landmarks = int((len(self.x)-3)/2)
            A = np.matrix([[np.cos(self.x[2,0]), -np.sin(self.x[2,0]), self.x[0,0]],
                [np.sin(self.x[2,0]),  np.cos(self.x[2,0]), self.x[1,0]],
                [0                ,  0                , 1]])
            m_x = landmarks[j,0]
            m_y = landmarks[j,1]

            lm = np.array([m_x,m_y,1])
            meas_landmark = np.matmul(A,lm)
            meas_landmark = np.delete(meas_landmark,[2,2])
            print('Observed landmark: ' + str(meas_landmark) + ' (mx,my): (' + str(m_x) + ',' + str(m_y) + ')')

            # Calculate depth and noise for observed landmark
            meas_range = np.sqrt(np.square(meas_landmark[0,0]-self.x[0,0])+np.square(meas_landmark[0,1]-self.x[1,0]))
            meas_bearing = np.arctan((meas_landmark[0,1]-self.x[1,0])/(meas_landmark[0,0]-self.x[0,0])) - self.x[2,0]
            meas_bearing = wrap_to_pi(meas_bearing)
            R = np.array([[self.v_r*meas_range , 0],
                [0,  self.v_b*meas_range]])
            
            # Use ML estimator for data assosciation
            residuals = np.array([])
            for i in range(num_landmarks):
                # construct transformation matrix (with rotation and translation)
                pred_landmark = np.array([self.x[3+2*(i),0],self.x[4+2*(i),0]])
                print('Recorded landmark '+ str(i) + ' ' + str(pred_landmark))
                # Calculate depth and noise for observed landmark
                pred_range = np.sqrt(np.square(pred_landmark[0]-self.x[0,0])+np.square(pred_landmark[1]-self.x[1,0]))
                pred_bearing = np.arctan((pred_landmark[1]-self.x[1,0])/(pred_landmark[0]-self.x[0,0])) - self.x[2,0]

                # Construct H matrix rows, append 0s and 1s depending on which landmark is considered, then stack rows
                hr1 = np.array([(self.x[0,0]-pred_landmark[0])/pred_range, (self.x[1,0]-pred_landmark[1])/pred_range, 0])
                hr2 = np.array([(pred_landmark[1]-self.x[1,0])/np.square(pred_range), (pred_landmark[0]-self.x[0,0])/np.square(pred_range), -1])
                for val in range(num_landmarks):
                    if val == i:
                        hr1 = np.append(hr1,[-(self.x[0,0]-pred_landmark[0])/pred_range, -(self.x[1,0]-pred_landmark[1])/pred_range])
                        hr2 = np.append(hr2,[-(pred_landmark[1]-self.x[1,0])/np.square(pred_range), -(pred_landmark[0]-self.x[0,0])/np.square(pred_range)])
                    else:
                        hr1 = np.append(hr1,[0, 0])
                        hr2 = np.append(hr2,[0, 0])
                H = np.vstack((hr1,hr2))
                Kappa = np.matmul(H,np.matmul(self.P,np.transpose(H))) + R
                #residuals = np.append(residuals,np.matmul((meas_landmark-pred_landmark),np.matmul(inv(Kappa),np.transpose(meas_landmark-pred_landmark))))
                rsD = 0.5*np.sqrt(np.matmul((meas_landmark-pred_landmark),np.transpose(meas_landmark-pred_landmark)))
                residuals = np.append(residuals, rsD)
                print('Does this look like zero? ' + str(rsD))

                # Find ML estimate of which landmark is being observed

            if len(residuals) > 0:
                residuals = np.asarray(residuals)
                residuals = np.absolute(residuals)
                ind = np.unravel_index(np.argmin(residuals, axis=None), residuals.shape)
                r = residuals[ind]
                print('Residuals: ' + str(residuals))
                    
            # If no known correspondance, add landmark to state vector
            if r >= self.r_t:
                self.x = np.concatenate((self.x,[[meas_landmark[0,0]],[meas_landmark[0,1]]]),axis=0)
                print('Landmark appended to state vector, new state vector: ' + str(self.x))
                Jz = np.array([[np.cos(self.x[2,0]+dT), -dY],[np.sin(self.x[2,0]+dT), dX]])
                C = np.matmul(Phi[0:2,0:3],np.matmul(self.P[0:3,0:3],np.transpose(Phi[0:2,0:3]))) + np.matmul(Jz,np.matmul(R,np.transpose(Jz))) # Jxr*P*Jxr^T + R (iden)
                G = np.matmul(self.P[0:3,0:3],np.transpose(Phi[0:2,0:3]))                                 # P*Jxr^T
                if num_landmarks > 0:
                    G = np.concatenate((G,np.zeros((int(2*num_landmarks),2))),axis=0)
                M1 = np.concatenate((np.transpose(G),C),axis=1)
                M2 = np.concatenate((self.P,G),axis=1)
                self.P = np.concatenate((M2,M1),axis=0)                                             # New Cov Matrix
                # If known correspondance, we run an update from that landmark    
            else: 
                # predicted landmark found from ML estimator
                # construct transformation matrix (with rotation and translation)
                pred_landmark = np.array([self.x[3+2*(ind[0]),0],self.x[4+2*(ind[0]),0]])
                print('Recorded landmark '+ str(ind[0]) + ' ' + str(pred_landmark))
                # Calculate depth and noise for observed landmark
                pred_range = np.sqrt(np.square(pred_landmark[0]-self.x[0,0])+np.square(pred_landmark[1]-self.x[1,0]))
                pred_bearing = np.arctan((pred_landmark[1]-self.x[1,0])/(pred_landmark[0]-self.x[0,0])) - self.x[2,0]
                pred_bearing = wrap_to_pi(pred_bearing)
                pred = np.array([[pred_range], [pred_bearing]])
                meas = np.array([[meas_range], [meas_bearing]])

                # Construct H matrix rows, append 0s and 1s depending on which landmark is considered, then stack rows
                hr1 = np.array([(self.x[0,0]-pred_landmark[0])/pred_range, (self.x[1,0]-pred_landmark[1])/pred_range, 0])
                hr2 = np.array([(pred_landmark[1]-self.x[1,0])/np.square(pred_range), (pred_landmark[0]-self.x[0,0])/np.square(pred_range), -1])
                for val in range(num_landmarks):
                    if val == i:
                        hr1 = np.append(hr1,[-(self.x[0,0]-pred_landmark[0])/pred_range, -(self.x[1,0]-pred_landmark[1])/pred_range])
                        hr2 = np.append(hr2,[-(pred_landmark[1]-self.x[1,0])/np.square(pred_range), -(pred_landmark[0]-self.x[0,0])/np.square(pred_range)])
                    else:
                        hr1 = np.append(hr1,[0, 0])
                        hr2 = np.append(hr2,[0, 0])
                H = np.vstack((hr1,hr2))
                # compute kalman gain
                K1 = np.matmul(H,np.matmul(self.P,np.transpose(H)))  # H*P*H^T
                K2 = np.matmul(self.P,np.transpose(H))               # P*H^T
                K = np.matmul(K2,inv(np.add(K1,R)))                     # P*H^T(H*P*H^T + R)^-1
                # Use kalman gain to generate estimate and update covariance
                err = pred-meas
                err[1] = wrap_to_pi(err[1])
                print('Error: ' + str(err))
                print('Gain: ' + str(K)) 
                self.x = np.add(self.x, np.matmul(K,err)) # x = x + K*(y-h(x))
                self.x[2,0] = wrap_to_pi(self.x[2,0])
                print('Update: ' + str(self.x))
                self.P = np.matmul(np.subtract(np.eye(len(self.x)),np.matmul(K,H)),self.P) # (I-K*H)*P 
            self.q.put(self.x)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('slam_node')

    rospy.Subscriber("/cloud_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # Create SLAM object
    q = multiprocessing.Queue()
    slam_obj = SLAM(q)
    tk_proc = TkGUI(q)
    tk_proc.start()
    # This is the code block for importing the data recorded by Cozmo 
    # Read in our data and do basic parsing

    pose = []       # robot pose data (x,y,theta)
    control = []    # control input/odom (dist,angle)
    landmarks = []  # landmark data (num,mx,my)
    imgfile = []    # true if next step is new image, false if same image

    with open('/home/labuser/UCSC-SLAM/project_ws/src/slam_node/data/data.csv','r') as cfile:
        reader = csv.reader(cfile)
        # Next will skip the first row of values (the column names)
        next(reader)
        for row in reader:
            # Add texts from csv to corpus
            if row[1] != '0':
                landmarks.append([int(row[1]),float(row[7]),float(row[8])])
                pose.append([float(row[4]),float(row[5]),float(row[6])]) 
                control.append([float(row[2]),float(row[3])])
            else:
                landmarks.append([0,0,0])
                pose.append([float(row[4]),float(row[5]),float(row[6])]) 
                control.append([float(row[2]),float(row[3])])

            if row[0] == '1':
                imgfile.append(True)
            else: 
                imgfile.append(False)
                    
    print("Read in control parameter: " + str(control) + '\n*********************************************************************************')
    # Transform to np array for usability
    pose = np.asarray(pose)
    control = np.asarray(control)
    landmarks = np.asarray(landmarks)
    imgfile = np.asarray(imgfile)

    # Loop through data and run update for each timestep 
    obsLandmarks = []
    for j in range(len(landmarks)-1):
        j = j + 1
        if landmarks[j][0] != 0:
            obsLandmarks.append(landmarks[j][1:3])
        if landmarks[j][0] == 1 or landmarks[j][0] == 0:
            obsLandmarks = np.asarray(obsLandmarks)
            print('Running ' + str(j) + '-th update on ' + str(len(obsLandmarks)) + ' observed landmarks: ' + str(obsLandmarks))
            print('Control update with Forward: ' + str(control[j][0]) + ' ,Angle: ' + str((control[j][1]*np.pi)/180))
            slam_obj.update(obsLandmarks, control[j][0], (control[j][1]*np.pi)/180)
            obsLandmarks = []

    # finally let the node spin to its wee hearts content
    rospy.spin()

if __name__ == '__main__':
    listener()

