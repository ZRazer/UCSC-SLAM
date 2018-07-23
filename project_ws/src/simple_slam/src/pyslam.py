#!/usr/bin/env python
# Author : Joseph Grant
# Welcome to the SLAM
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
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

am_debugging = True

def debug_print(inp_str):
    if am_debugging:
        print(inp_str)
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
        self.sub.set_xlabel('x (m)')
        # List probably needs to be changed if any graphs are modified/added
        self.sub.set_ylabel('y (m)')
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
        self.sub.plot(self.x, self.y, color='blue') 
        # keep track of which landmark we are dealing with
        i = 0
        numLandmarks = int((len(x)-3)/2)
        self.lx = []
        self.ly = []
        for j in range(int((len(x)-3)/2)):
            self.lx.append(x[3+2*(j),0])
            self.ly.append(x[4+2*(j),0])

        self.sub.scatter(self.lx, self.ly, color='red')

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
#    debug_print('you pressed %s' % event.key)
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
            dict_data = q.get(block=False)
        except queue.Empty:
            root.after(10, updateGraph, out_listener, q, textBoxes)
            return
        # This list must be expanded if graphs are added/modified
        data = dict_data['state']
        out_listener[0].on_data(data)
        textBoxes[0].configure(state = 'normal')
        textBoxes[0].delete('1.0', Tk.END)
        textBoxes[0].insert(Tk.INSERT, "Pose:\nx: " + str(int(data[0,0])) + "\ny: " + str(int(data[1,0])) + "\ntheta: " + str(int(data[2,0]*180/np.pi)))
        textBoxes[0].configure(state = 'disabled')
        textBoxes[1].configure(state = 'normal')
        textBoxes[1].delete('1.0', Tk.END)
        textBoxes[1].insert(Tk.INSERT, "Num landmarks:\n" + str(int((len(data)-3)/2)))
        textBoxes[1].configure(state = 'disabled')
        root.after(10, updateGraph, out_listener, q, textBoxes)
        return # Return to prevent extra after() call
    elif start == 2:
        for listener in out_listener:
            listener.on_data(np.random.randint(-5,50))
    after_id = root.after(10, updateGraph, out_listener, 0, 0)
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
        self.poseInit = False 
        self.r_t = 0.1                                    # Threshold for assosciation
        self.v_r = 0.05                                  # Measurement error ratio
        self.v_b = 0.005
        self.x = None
        self.P = np.array([[0.1,0,0],[0,0.1,0],[0,0,np.pi/4]]) # Covariance matrix

        # Process noise intensity matrix 
        self.Gamma = np.array([[0.05, 0.005],
            [0.05, 0.005],
            [  0, 0.001]])
        self.dX = None
        self.dY = None
        self.dT = None
        self.q = q
        self.data = {}

    def odom_update(self,dx,dy,dt):
        debug_print('Running odometry update (x,y,t): (' + str(self.x[0,0]) + ',' + str(self.x[1,0]) + ',' + str(self.x[2,0]) + ')')
        self.dT = dt
        self.dX = dx
        self.dY = dy
        # Calculate priors for pose and covariance
        xP = np.array([[self.x[0,0] + self.dX],
            [self.x[1,0] + self.dY],
            [self.x[2,0] + self.dT]])
        xP[2,0] = wrap_to_pi(xP[2,0])
        self.x[0:3] = np.array([[self.x[0,0] + self.dX],
            [self.x[1,0] + self.dY],
            [self.x[2,0] + self.dT]])
        self.x[2,0] = wrap_to_pi(self.x[2,0])

        Phi = np.array([[1, 0, -self.dY],
            [0, 1, self.dX],
            [0, 0, 1]])

        r1 = np.matmul(np.matmul(self.Gamma,np.identity(2)), np.transpose(self.Gamma)) # Gamma*Q*Gamma^T
        r2 = np.matmul(np.matmul(Phi,self.P[0:3,0:3]), np.transpose(Phi))         # Phi*P*Phi^T
        self.P[0:3,0:3] = r1 + r2

        if len(self.P) > 3:
            temp = np.matmul(Phi,self.P[0:3,3:len(self.P)])
            self.P[0:3,3:len(self.P)] = temp
            self.P[3:len(self.P),0:3] = np.transpose(temp)
        self.data['state'] = self.x
        self.q.put(self.data)

    def landmark_update(self, data):
        debug_print('Running landmark update')
        debug_print('Prior: ' + str(self.x[0:3]))
        Phi = np.array([[1, 0, -self.dY],
            [0, 1, self.dX],
            [0, 0, 1]])
        # For every landmark observed, run update or add it to state vector
        # In simple case we only observe one landmark at a time

        r = self.r_t
        # Find landmark using observation model
        num_landmarks = int((len(self.x)-3)/2)
        # A = np.matrix([[np.cos(self.x[2,0]), -np.sin(self.x[2,0]), self.x[0,0]],
        #     [np.sin(self.x[2,0]),  np.cos(self.x[2,0]), self.x[1,0]],
        #     [0                ,  0                , 1]])
        # m_x = landmark.x
        # m_y = landmark.y

        # lm = np.array([m_x,m_y,1])
        # meas_landmark = np.matmul(A,lm)
        # meas_landmark = np.delete(meas_landmark,[2,2])
        meas_landmark = np.array([[self.x[0,0]+np.cos(self.x[2,0])*data.x-np.sin(self.x[2,0])*data.y, self.x[1,0]+np.cos(self.x[2,0])*data.y+np.sin(self.x[2,0])*data.x]])
        debug_print('Observed landmark: ' + str(meas_landmark))

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
            debug_print('Recorded landmark '+ str(i) + ' ' + str(pred_landmark))
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

            # Find ML estimate of which landmark is being observed

        if len(residuals) > 0:
            residuals = np.asarray(residuals)
            residuals = np.absolute(residuals)
            ind = np.unravel_index(np.argmin(residuals, axis=None), residuals.shape)
            r = residuals[ind]
            debug_print('Residuals: ' + str(residuals))
                
        # If no known correspondance, add landmark to state vector
        if r >= self.r_t:
            self.x = np.concatenate((self.x,[[meas_landmark[0,0]],[meas_landmark[0,1]]]),axis=0)
            debug_print('Landmark appended to state vector, new state vector: ' + str(self.x))
            Jz = np.array([[np.cos(self.x[2,0]+self.dT), -self.dY],[np.sin(self.x[2,0]+self.dT), self.dX]])
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
            debug_print('Recorded landmark '+ str(ind[0]) + ' ' + str(pred_landmark))
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
            debug_print('Error: ' + str(err))
            debug_print('Gain: ' + str(K)) 
            self.x = np.add(self.x, np.matmul(K,err)) # x = x + K*(y-h(x))
            self.x[2,0] = wrap_to_pi(self.x[2,0])
            debug_print('Update: ' + str(self.x))
            self.P = np.matmul(np.subtract(np.eye(len(self.x)),np.matmul(K,H)),self.P) # (I-K*H)*P 
        self.data['state'] = self.x
        #self.q.put(self.data)

class slam_node():
    def __init__(self):
        rospy.Subscriber("/tf", TFMessage, self.lm_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.q = multiprocessing.Queue()
        self.slam_obj = SLAM(self.q)
        tk_proc = TkGUI(self.q)
        tk_proc.start()
        # Lock for callback threads
        self.lock = False
        # For keeping track of time delta 
        self.t1 = None
        self.t2 = None

    # Callback upon reciving new landmarks, updating state estimate
    def lm_callback(self, data):
        # if pose is unitialized, wait for initialization and let these landmarks be consumed
        if self.slam_obj.poseInit:
            for tfm in data.transforms:
                do_nother = True
                if tfm.child_frame_id == "landmark":
                    self.slam_obj.landmark_update(tfm.transform.translation)

    # Use odometry and prediction model to update state  
    def odom_callback(self, data):
        # if pose is unitialized, initialize it with first data
        if self.slam_obj.poseInit:
            self.t2 = float(data.header.stamp.secs) + (float(data.header.stamp.nsecs)*(10^-9))
            time_delta = 0.05
            dx = time_delta*data.twist.twist.linear.x
            dy = time_delta*data.twist.twist.linear.y
            dt = time_delta*data.twist.twist.angular.z
            self.slam_obj.odom_update(dx,dy,dt)
            self.t1 = self.t2
        else:
            self.t1 = float(data.header.stamp.secs) + (float(data.header.stamp.nsecs)*(10^-9))
            x_angle = 2 * np.arccos(data.pose.pose.orientation.w)
            self.slam_obj.x = np.array([[data.pose.pose.position.x],
                                        [data.pose.pose.position.y],
                                        [x_angle]])
            self.slam_obj.dX = 0
            self.slam_obj.dY = 0
            self.slam_obj.dT = 0
            self.slam_obj.poseInit = True



def listener():
    rospy.init_node('simple_slame')
    # init slam node, non anonymous mode
    sm_node = slam_node()
    # let the node spin to its wee hearts content
    rospy.spin()

if __name__ == '__main__':
    listener()
