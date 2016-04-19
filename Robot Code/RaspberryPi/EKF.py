#EKF.py
import time
import math
import numpy as np
import serial
import time

def runEKF(mu, Sigma, u, z, c, m, dt):
    a = np.matrix([0.1, 0.1, 0.1, 0.1]) #Noise
    #Extract variables
    theta = mu[2,0]
    v = u[0,0]
    w = u[1,0]
    #---Prediction Step
        #Jacobian derivatives
    G = np.matrix([[1, 0, ((v/w) * (-math.cos(theta) + math.cos(theta + (w*dt))))],
         [0, 1, ((v/w)) * (-math.sin(theta) + math.sin(theta + (w*dt)))],
         [0, 0, 1]])
    V = np.matrix([[(-math.sin(theta) + math.sin(theta + (w*dt)))/w, (v*(math.sin(theta) - math.sin(theta + (w*dt))))/math.pow(w,2) + (v*(math.cos(theta + (w*dt))*dt))/w],
         [(math.cos(theta) - math.cos(theta + (w*dt)))/w, -(v*(math.cos(theta) - math.cos(theta + (w*dt))))/math.pow(w,2) + (v*(math.sin(theta + (w*dt))*dt))/w],
         [0, dt]])
        #Noise in control space
    M = np.matrix([[(a[0,0] * math.pow(v,2)) + (a[0,1] * math.pow(w,2)), 0],
         [0, (a[0,2] * math.pow(v,2)) + (a[0,3] * math.pow(w,2))]])
        #Motion Update
    muBar = mu + np.matrix([[-(v/w)*math.sin(theta) + (v/w)*math.sin(theta + (w*dt))],
                            [((v/w)*math.cos(theta)) - ((v/w)*math.cos(theta + (w*dt)))],
                            [(w*dt)]])
    SigmaBar = np.dot(np.dot(G,Sigma),G.T) + np.dot(np.dot(V,M),V.T)
    #---Measurement Update Step
    Q = np.matrix([[0.1, 0, 0],
                   [0, 0.1, 0],
                   [0, 0, 0.1]])
    #For each feature
    for n in range(0,1):
        j = c
        q = pow(m[1,j]-muBar[0,0],2) + pow(m[2,j] - muBar[1,0],2)
        zHat = np.matrix([[math.sqrt(q)],
                [math.fmod(np.arctan2(m[2,j]-muBar[1,0], m[1,j]-muBar[0,0]) - muBar[2,0] + math.pi, 2*math.pi) - math.pi],
                [m[0,j]]])
        H = np.matrix([[-((m[1,j]-muBar[0,0])/math.sqrt(q)), -((m[2,j]-muBar[1,0])/math.sqrt(q)), 0],
             [(m[2,j]-muBar[1,0])/q, -((m[1,j]-muBar[0,0])/q), -1],
             [0, 0, 0]])
        S = np.dot(np.dot(H, SigmaBar), (H.T)) + Q
        K = np.dot(np.dot(SigmaBar, (H.T)), np.linalg.inv(S))
        zDiff = z[:,n] - zHat
        zDiff[1,n] = math.fmod(zDiff[1,n] + math.pi, 2*math.pi) - math.pi
        muBar = muBar + np.dot(K, zDiff)
        SigmaBar = np.dot((np.matrix([[1, 0, 0],
                                      [0, 1, 0],
                                      [0, 0, 1]]) - np.dot(K,H)),SigmaBar)
    mu = muBar
    Sigma = SigmaBar
    return mu, Sigma






print "EKF Light Sensor v1.0"
print "written by Francis Poole"
#---Initialise World
print "Initialisting world..."
    #Map of Features
m = np.matrix([[1],
               [0],
               [0]])
    #Correspondences
c = 0
#---Agent Setup
r = 3.1 #Radius of wheels
D = 14 #Distance between wheels
    #--Brain
    #Pose
mu = np.matrix([[0],
                [0],
                [0]])
Sigma = np.matrix([[0.1,0,0],
                   [0,0.1,0],
                   [0,0,0.1]])
print "------"
#---Connect to Arduino
print "Initialising Serial port"
ser = serial.Serial('/dev/usbmodemfa131',115200)#('/dev/ttyACM0',115200)
print "Connecting to Arduino..."
#Wait for reply
while ser.inWaiting() == 0 :
        ser.write(1) #Ping
        time.sleep(1)  
#Reply recieved! Connected
print ser.readline()
print "------"
#---Run
#Start robot motion...
for x in range(1,50):
    ser.write(1) #Ping data request
    while ser.inWaiting() == 0 :
        pass
    data = ser.readline()
    dataSplit = data.split(';')
    #Motion Control
    uTemp = dataSplit[0].split(',')
    u = np.matrix([[uTemp[0]],
                   [uTemp[1]]])
    print u
    #Sensor Measurement
    zTemp = dataSplit[1].split(',')
    z = np.matrix([[zTemp[0]],
                   [zTemp[1]],
                   [0]])
    print z
    dt = dataSplit[2]
    print dt
    dt = 0.1; #Need to get from Arduino!
    mu, Sigma = runEKF(mu, Sigma, u, z, c, m, dt)
    time.sleep(0.1)
