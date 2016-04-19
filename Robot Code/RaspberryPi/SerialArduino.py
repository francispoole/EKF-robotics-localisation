import serial
import time
ser = serial.Serial('/dev/ttyACM0',9600)

while True :
    varL = input("Enter left motor speed: ")
    if varL > 255 or varL < -255 :
        continue
    
    varR = input("Enter right motor speed: ")
    if varR > 255 or varR < -255 :
        continue

    
    ser.write(str(varL) + "," + str(varR) + "\n")
    
    print "Out: ", str(varL), ", ", str(varR)
    while ser.inWaiting() == 0 :
        pass
    print ser.readline()
    
    print "------"
    #print "in: ", int(num) * 2
