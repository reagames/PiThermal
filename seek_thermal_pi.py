###########################################################################
#
#Based on beautiful code from Cynfab, under permission of author.
#License allows unrestricted use with attribution
#Attributions (including original note):
# Many thanks to the folks at eevblog, especially (in no particular order)
#   miguelvp, marshallh, mikeselectricstuff, sgstair, Fry-kun and many others
#     for the inspiration to figure this out
# This is not a finished product and you can use it if you like.

# You will probably have to run this as root unless you get your udev/mdev rules
# set up to allow the Seek device to be used by other than root. Usually python requires at least sudo as well

#ABOUT THIS VERSION
#This code allows you to connect and receive images from Seek Thermal device to Raspberry Pi computer (at least Zero, v.2 B, B+, ver. 3, supported). This version was tested on Raspbian Jessie kernel ver. 4.9
#List of changes made to original code:

#- lightweight app, starts from command line (no Tk GUI/ X server required), uses framebuffer 0 (/dev/fb0)
#- can be run on SPI display connected to Pi GPIO (see notro/fbtft on how to connect and initialize display, this may require certain changes in code) using SPI bus communication (should be enabled on Pi in configs
#- has basic GUI which can be navigated using Pi GPIO buttons (three on/off switches)
#- (!)requires Raspberry Pi hardware camera module connected - allows blending of Pi camera visible and thermal image. Pi Camera stream runs in different thread to provide faster camera image aquisition. Overlay can be adjusted by manipulating Pi camera image crop area. Level of blending between thermal and visual image can be adjusted using GUI and GPIO buttons.
#- Uses adjustable calibration mode (meaning device performs calibration only at certain periods of time, set by user) - can be adjusted using GPIO buttons. 
#- Different color modes - can be switched by GPIO buttons

# You will need to have python 2.7 (used for this version)
# and PyUSB 1.0
# and PIL 1.1.6 or better
# and numpy
# and scipy
# and ImageMagick
# and openCV 2.0
# and pygame
# and Imutils
# and fbi installed at raspberry to use external display
# and other stuff to get this work
# There is a lot of space for improvement in the code, so everyone is welcome to...

#Read more out there:
#https://github.com/lod/seek-thermal-documentation/wiki/Seek-Software
#https://www.eevblog.com/forum/thermal-imaging/yet-another-cheap-thermal-imager-incoming/

#
###########################################################################

#Core imports
import usb.core
import usb.util
import sys, os
import cv2
from PIL import Image
import numpy
from numpy import array
from scipy.misc import toimage
from scipy import ndimage
import matplotlib
import math
import io
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS

#external module imports
import RPi.GPIO as GPIO
import time

#import fbtft driver
import pygame
from pygame.locals import *

########Reload external display driver - uncomment following lines to use external display
#command = 'sudo modprobe -r fbtft_device'
#p = os.system('sudo %s' % (command))
#print "display reset"
#time.sleep(0.1) # delay for 0.1 seconds

########Initialize external display - uncomment following lines to use external display, this code is for ST7735 driver display
#command = 'sudo modprobe fbtft_device custom name=adafruit18 gpios=reset:25,dc:24,wr:0,cs:8 width=128 height=160 rotate=90'
#p = os.system('sudo %s' % (command))
#time.sleep(0.1)
#print "display started"

#Use logo to indicate system startup - put the picture you like in the same folder, name it splah.png and see the program starts with it.
#img=pygame.image.load("splash.png")

#adjustable parameters
height=128
width=160
rotate=1
cameramode=0

#Yeah, that is magic number
magicnum=100

#init pygame output - change to /fb1 to use external display
os.environ["SDL_FBDEV"] = "/dev/fb0"

#Pygame display surface initiation
pygame.init()
MAINSURF=pygame.display.set_mode((width, height),0,32)
DISPLAYSURF=pygame.Surface((width, height))
pygame.mouse.set_visible(0) #hide mouse
#MAINSURF.blit(img,(0,0))  #uncomment to use loading splashscreen
pygame.display.flip() # update the display

# set up the colors BGR
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE   = (0,   0,   255)
GREEN = (  0, 255,   0)
RED  = (  255,   0, 0)
UIGREEN = (8, 150, 8 )
UIRED = (220, 77 , 48)
UIBLUE = (20, 80, 160)
UIWHITE = (128, 128 , 128)
UIGRAY = (50, 50 , 50)
#DISPLAYSURF.fill(RED)  # Debug


#init pygame timer
clock = pygame.time.Clock()

#global calibration array variables
imcalib = 0
imgain = 0

#frame variables
counter=0
mode=0
frame=0

#Pi Camera interface from imutils
vs = PiVideoStream(resolution=(width*2, height*2),framerate=30).start()
vsimage=vs.read()

try:
    import pygame.surfarray as surfarray
except ImportError:
    raise ImportError, "pygame failed to initialize"

#pygame font init
pygame.font.init()
myfont = pygame.font.SysFont('Nimbus Sans', 15)

#GUI variables
uiMenuItem = 0
uiSelector = 0
uiTimer=0
uiVisible = 0
uiCrosshair=0
uiColormode=0
uiColormodesMax=5
uiMaxItems=7
scl=1
scl1=26
killtimer=0 #UI shutdown command
actnum=0
counterMax=360
counterAct=90 #default calibration delay
modename=""

xmed=80
ymed=60

dbgX=292
dbgY=0

dc = 50 # duty cycle (0-100) for PWM backlight pin

#GPIO Pin definitons for use with click buttons: make sure they are not mixed with external display connections
fwPin = 16 # Broadcom pin 16 - p36 R3  left button
midPin = 20 # Broadcom pin 20 - p38 R3 middle button
revPin = 21 # Broadcom pin 17 - p40 R3 right button
ledPin = 26 # Broadcom pin 26 - p37 R3 external display brightness LED - pwm brightness

GPIO.cleanup() # cleanup all GPIO
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(ledPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(ledPin, 50)  # Initialize PWM on pwmPin 50Hz frequency
pwm.start(dc)

#Setup GPIO buttons
GPIO.setup(fwPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Button pin set as input w/ pull-up
GPIO.setup(midPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Button pin set as input w/ pull-up
GPIO.setup(revPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Button pin set as input w/ pull-up

#Thermal camera class
class Thermal:
    

    def close(event):
        print("closing...")
        GPIO.cleanup() # cleanup all GPIO
        sys.exit() # if you want to exit the entire thing

# defs
    #This one is for color maps - different visualisation. User-adjustable.
    def applyCustomColorMap(self,im_gray) :
    
        lut = numpy.zeros((256, 1, 3), dtype=numpy.uint8)
    
        #Red channel
        lut[:, 0, 0] = [54,55,56,57,57,58,59,60,61,62,62,63,64,65,66,67,67,68,69,70,71,72,72,73,74,75,76,77,78,78,79,80,81,82,83,83,84,85,86,87,88,88,89,90,91,92,93,93,94,95,96,97,98,98,99,100,101,102,103,103,104,105,106,107,108,110,112,114,116,118,119,121,123,125,127,129,131,133,135,137,139,141,143,145,146,148,150,152,154,156,158,160,162,164,166,168,170,171,173,175,177,179,181,183,185,187,189,191,193,195,197,198,200,202,204,206,208,210,212,214,216,218,220,222,224,225,227,229,230,231,231,232,232,232,233,233,234,234,234,235,235,236,236,236,237,237,238,238,238,239,239,240,240,240,241,241,242,242,242,243,243,244,244,244,245,245,246,246,246,247,247,248,248,248,249,249,250,250,250,251,251,252,252,252,253,253,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255]
        
        #Green channel
        lut[:, 0, 1] = [55,56,56,57,58,59,59,60,61,62,63,63,64,65,66,66,67,68,69,69,70,71,72,72,73,74,75,75,76,77,78,79,79,80,81,82,82,83,84,85,85,86,87,88,88,89,90,91,92,92,93,94,95,95,96,97,98,98,99,100,101,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,164,162,161,160,158,157,156,154,153,152,150,149,148,146,145,144,142,141,140,138,137,136,134,133,132,130,129,128,126,125,124,122,121,120,118,117,116,114,113,112,110,109,108,106,105,104,102,101,100,98,97,96,94,93,92,90,89,88,86,85,84,82,81,80]
    
        #Blue channel
        lut[:, 0, 2] = [54,55,56,56,57,58,59,60,60,61,62,63,64,64,65,66,67,67,68,69,70,71,71,72,73,74,75,75,76,77,78,79,79,80,81,82,82,83,84,85,86,86,87,88,89,90,90,91,92,93,93,94,95,96,97,97,98,99,100,101,101,102,103,104,104,105,105,105,105,105,105,106,106,106,106,106,106,107,107,107,107,107,107,108,108,108,108,108,108,109,109,109,109,109,109,110,110,110,110,110,110,111,111,111,111,111,111,112,112,112,112,112,113,113,113,113,113,113,114,114,114,114,114,114,115,115,115,115,115,115,115,116,116,116,116,116,116,116,117,117,117,117,117,117,117,117,118,118,118,118,118,118,118,119,119,119,119,119,119,119,119,120,120,120,120,120,120,120,121,121,121,121,121,121,121,121,122,122,122,122,122,122,122,123,123,123,123,123,123,123,123,124,122,120,118,116,114,112,111,109,107,105,103,101,99,97,95,93,91,89,87,85,83,81,80,78,76,74,72,70,68,66,64,62,60,58,56,54,52,50,48,47,45,43,41,39,37,35,33,31,29,27,25,23,21,19,17,16,14,12,10,8,6,4,2,0]
    
    
        im_color = cv2.LUT(im_gray, lut)
    
        return im_color;



    def usbinit(self):
    # find our Seek Thermal device  289d:0010
        dev = usb.core.find(idVendor=0x289d, idProduct=0x0010)

    # was it found?
        if dev is None:
    	    raise ValueError('Device not found')

    # set the active configuration. With no arguments, the first
    # configuration will be the active one
        dev.set_configuration()

    # get an endpoint instance
        cfg = dev.get_active_configuration()
        intf = cfg[(0,0)]

        ep = usb.util.find_descriptor(
	    intf,
    # match the first OUT endpoint
    	    custom_match = \
    	    lambda e: \
		usb.util.endpoint_direction(e.bEndpointAddress) == \
    		usb.util.ENDPOINT_OUT)

        assert ep is not None

	return dev

    # send_msg sends a message that does not need or get an answer
    def send_msg(self,dev,bmRequestType, bRequest, wValue=0, wIndex=0, data_or_wLength=None, timeout=None):
        assert (dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, data_or_wLength, timeout) == len(data_or_wLength))

    # alias method to make code easier to read
    # receive msg actually sends a message as well.
    def receive_msg(self,dev,bmRequestType, bRequest, wValue=0, wIndex=0, data_or_wLength=None, timeout=None):
        zz = dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, data_or_wLength, timeout) # == len(data_or_wLength))
        return zz

    # De-init the device
    def deinit(self,dev):
        msg = '\x00\x00'
        for i in range(3):
            self.send_msg(dev,0x41, 0x3C, 0, 0, msg)           # 0x3c = 60  Set Operation Mode 0x0000 (Sleep)

    #Thermal Camera initilization
    def camerainit(self,dev):

        try:
            msg = '\x01'
            self.send_msg(dev,0x41, 0x54, 0, 0, msg)              # 0x54 = 84 Target Platform 0x01 = Android
        except Exception as e:
            self.deinit(dev)
            msg = '\x01'
            self.send_msg(dev,0x41, 0x54, 0, 0, msg)              # 0x54 = 84 Target Platform 0x01 = Android

        self.send_msg(dev,0x41, 0x3C, 0, 0, '\x00\x00')              # 0x3c = 60 Set operation mode    0x0000  (Sleep)
        ret1 = self.receive_msg(dev,0xC1, 0x4E, 0, 0, 4)             # 0x4E = 78 Get Firmware Info
        
        self.send_msg(dev,0x41, 0x56, 0, 0, '\x20\x00\x30\x00\x00\x00')                  # 0x56 = 86 Set Factory Settings Features
        self.send_msg(dev,0x41, 0x56, 0, 0, '\x20\x00\x50\x00\x00\x00')                  # 0x56 = 86 Set Factory Settings Features
        self.send_msg(dev,0x41, 0x56, 0, 0, '\x0C\x00\x70\x00\x00\x00')                  # 0x56 = 86 Set Factory Settings Features
        self.send_msg(dev,0x41, 0x56, 0, 0, '\x06\x00\x08\x00\x00\x00')                  # 0x56 = 86 Set Factory Settings Features
        self.send_msg(dev,0x41, 0x3E, 0, 0, '\x08\x00')                                  # 0x3E = 62 Set Image Processing Mode 0x0008 Normal
        self.send_msg(dev,0x41, 0x3C, 0, 0, '\x01\x00')                                  # 0x3c = 60 Set Operation Mode        0x0001  (Run)

    def read_frame(self,dev): # Send a read frame request

        self.send_msg(dev,0x41, 0x53, 0, 0, '\xC0\x7E\x00\x00')                 # 0x53 = 83 Set Start Get Image Transfer

        try:
            data  = dev.read(0x81, 0x3F60, 1000)
            data += dev.read(0x81, 0x3F60, 1000)
            data += dev.read(0x81, 0x3F60, 1000)
            data += dev.read(0x81, 0x3F60, 1000)
        
        except usb.USBError as e:
            print "device error"
            GPIO.cleanup() # cleanup all GPIO
            sys.exit()

        return data
    
    #No calibration frame - streaming mode
    def set_stream(self,dev): 
        self.send_msg(dev,0x41, 0x3C, 0, 0, '\x00\x00')  # 0x3c = 60 Set operation mode    0x0000  (Sleep)
        time.sleep(0.05)
        
        #disable calibration
        self.send_msg(dev,0x41, 0x3E, 0, 0, '\x00\x00')  

        time.sleep(0.05)
        #print "mode changed to stream"
        self.send_msg(dev,0x41, 0x3C, 0, 0, '\x01\x00')              # 0x3c = 60 Set operation mode    0x0001  (Run)
        return

    
    def set_normal(self,dev): #set normal mode with calibration
        self.send_msg(dev,0x41, 0x3C, 0, 0, '\x00\x00')  # 0x3c = 60 Set operation mode    0x0000  (Sleep)
        time.sleep(0.01)
        self.send_msg(dev,0x41, 0x3E, 0, 0, '\x08\x00')  #send image processing mode 
        self.send_msg(dev,0x41, 0x37, 0, 0, '\x00\x00')
        time.sleep(0.01)
        #print "mode changed to normal"
        self.send_msg(dev,0x41, 0x3C, 0, 0, '\x01\x00')              # 0x3c = 60 Set operation mode    0x0001  (Run)
        return
    

###################################
#--------------------*************-------------------#
# Start of main routine
#--------------------*************-------------------#

    # Main program starts here (you can tell I'm new to Python ;)
    #Initialization
    def initialize(self):

        global dev, scl, scl1, scl2
        global calImage, calImagex, killtimer
        global camimage, stream, vs


    # Set up device
        dev = self.usbinit()
        self.camerainit(dev)

    # get a cal image so the data isn't null if/when we miss the first one
        self.get_cal_image(dev)

    # var to store adj scl1 value
    #    self.topadj=0
       
       
    #init streaming frame mode
        self.set_stream(dev)
    
        time.sleep(2.0)

    # Start the update image routine with defined delay
        self.UpdateImage(0.0125)
    
    
    counter=counterAct-1

# End of the initilization routine


#--------------------*************-------------------#
# This is actually the main loop
#--------------------*************-------------------#

    def UpdateImage(self, delay, event=None):
        global scl, scl1, dev, status, calImage, label, dc, counter, mode, actnum, counterAct, counterMax, modename, frame
        global uiMenuItem, uiSelector, uiTimer, uiVisible, uiMaxItems, uiCrosshair, dbgX, uiColormode, uiColormodesMax, killtimer, cameramode
        global MAINSURF, camera, rawCapture, camimage, dispimg, stream,vs, vsimage, outimage, magicnum
        

        
        while 1:
            
            frame+=1
            
            if frame==1: #read from Seek Thermal
            
                self.image = self.get_image(dev)
                
#------->>>>GOTO CAMERA ADDON
                #######Uncomment if you do not use pi camera
                frame=0
            
            #if cameramode==0:
            #        frame=0
            
            elif frame==2: #read from Pi Camera - comment following if camera is not used
            
                vsimage=vs.read()
                vsimage = vsimage[102:178,122:219] # Crop from y:y+h, x:x+w for 160,128 w h
                #NOTE: it is img[y: y + h, x: x + w]
               
                #Process camera image
                vsimage = cv2.resize(vsimage,(width, height), interpolation = cv2.INTER_LINEAR)
                vsimage = numpy.rot90(vsimage,3)
                vsimage = numpy.flipud(vsimage)
                vsimage = cv2.cvtColor(vsimage, cv2.COLOR_RGB2BGR)
                dispimg = cv2.cvtColor(dispimg, cv2.COLOR_RGB2BGR)
                
                #Default image blending mode 50/50
                alpha = 0.5
                beta = ( 1.0 - alpha )
                
               #cv2.AddWeighted(src1, alpha, src2, beta, gamma, dst)
                cv2.addWeighted( dispimg, alpha, vsimage, beta, 0.0, dispimg)
                dispimg = cv2.cvtColor(dispimg, cv2.COLOR_BGR2RGB)
                
                frame=0
        
  
            #Write to pygame surface
            dst_ary = pygame.surfarray.pixels3d(DISPLAYSURF)
            dst_ary[...] = dispimg
            del dst_ary
            
            
            #DISPLAYSURF.unlock()
            MAINSURF.blit(DISPLAYSURF,(0,0))

            
            #increase frame counter to enable mode change
            counter+=1
            
                
            if mode==0: #ff mode
                if counter>counterAct:
                    self.set_normal(dev)
                    mode=1
                    counter=0
                
                
            elif mode==1:
                #if counter>0:
                self.get_cal_image(dev)
                self.set_stream(dev)
                mode=0
                counter=0
            
            #go on with GPIO
            if GPIO.input(midPin): # button is released
                uiSelector=0
                killtimer=0
                modename=""
                
            else:
                #Pi shutdown function - hold button to shut down
                killtimer+=1
                actnum = killtimer
                if killtimer>=70:
                    killtimer=0
                    print "shutdown"
                    command = 'sudo shutdown -h now'
                    p = os.system('sudo %s' % (command))

                if uiSelector==0:
                    uiSelector = 1
                    
                    #cycle through menu items
                    uiMenuItem += 1
                    
                    if uiMenuItem > uiMaxItems:
                        uiMenuItem=1
                
                    uiTimer = 32
                    print(uiMenuItem)

            #Toggle menu visibility
            if uiTimer>0:
                uiTimer-=1
                uiVisible=1
                
            if uiTimer<=0:
                uiVisible=0
                uiMenuItem=0
        
            #________GUI_________#
        
            if uiVisible==1:
                #button controls within menu items
                # place text on framebuffer screen
                text2 = myfont.render('%d' %(actnum), False, (0, 255, 0))
                newtextsurface=pygame.transform.rotate(text2,180)
                MAINSURF.blit(newtextsurface,(20,50))
                
                if uiMenuItem==1111: #DEBUG - this is a test menu item to test something
                    actnum = magicnum
                    modename="testnumber"
                    #Control lines
                    pygame.draw.line(MAINSURF, UIGREEN, (10, 10), (100, 10),2)#
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 20), (100, 20),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 30), (100, 30),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 40), (100, 40),1)
                    pygame.draw.circle(MAINSURF, UIBLUE, (100-magicnum, 10), 3, 0)#
                    pygame.draw.circle(MAINSURF, UIWHITE, (40-int(0.15*scl), 20), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-scl1, 30), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100- int(0.08 * dbgX), 40), 2, 0)
                    
                    #Handle +/- button clicks
                    if GPIO.input(fwPin)==0:
                        uiTimer = 20
                        magicnum+=1
                        if magicnum>=200:
                            magicnum=200
                        actnum = magicnum
            
                    elif GPIO.input(revPin)==0:
                        uiTimer = 20
                        magicnum-=1
                        if magicnum<=20:
                            magicnum=20
                        actnum = magicnum
                #now let's take it more seriously
                if uiMenuItem==1: #LCD brightness
                    actnum = dc
                    modename="Brightness"
                    #GUI control lines
                    pygame.draw.line(MAINSURF, UIGREEN, (10, 10), (100, 10),2)#
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 20), (100, 20),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 30), (100, 30),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 40), (100, 40),1)
                    pygame.draw.circle(MAINSURF, UIBLUE, (100-dc, 10), 3, 0)#
                    pygame.draw.circle(MAINSURF, UIWHITE, (40-int(0.15*scl), 20), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-scl1, 30), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100- int(0.08 * dbgX), 40), 2, 0)
                    
                    #Handle +/- button clicks
                    if GPIO.input(fwPin)==0:
                            uiTimer = 20
                            dc+=1
                            if dc>=95:
                                dc=95
                            pwm.ChangeDutyCycle(dc)
                            actnum = dc
                
                    elif GPIO.input(revPin)==0:
                            uiTimer = 20
                            dc-=1
                            if dc<=2:
                                dc=2
                            pwm.ChangeDutyCycle(dc)
                            actnum = dc
                                
                if uiMenuItem==2:#Low contrast threshold
                    actnum = scl
                    modename="Low limit"
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 10), (100, 10),1)
                    pygame.draw.line(MAINSURF, UIGREEN, (10, 20), (100, 20),2)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 30), (100, 30),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 40), (100, 40),1)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-dc, 10), 2, 0)
                    pygame.draw.circle(MAINSURF, UIBLUE, (40-int(0.15*scl), 20), 3, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-scl1,30), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100- int(0.08 * dbgX), 40), 2, 0)
                    
                    #Handle +/- button clicks
                    if GPIO.input(fwPin)==0:
                        uiTimer = 32
                        scl+=2
                        if scl>=200:
                            scl=200
                        actnum = scl

                    elif GPIO.input(revPin)==0:
                        uiTimer = 32
                        scl-=2
                        if scl<=-380:
                            scl=-380
                        actnum = scl

                if uiMenuItem==3:#High Contrast threshold
                    actnum = scl1
                    modename="High limit"
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 10), (100, 10),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 20), (100, 20),1)#
                    pygame.draw.line(MAINSURF, UIGREEN, (10, 30), (100, 30),2)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 40), (100, 40),1)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-dc, 10), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (40-int(0.15*scl), 20), 2, 0)#
                    pygame.draw.circle(MAINSURF, UIBLUE, (100-scl1, 30), 3, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100- int(0.08 * dbgX), 40), 2, 0)
                     
                     #Handle +/- button clicks
                    if GPIO.input(fwPin)==0:
                        uiTimer = 32
                        scl1+=1
                        if scl1>=100:
                            scl1=100
                        actnum = scl1
                                         
                    elif GPIO.input(revPin)==0:
                        uiTimer = 32
                        scl1-=1
                        if scl1<=1:
                            scl1=1
                        actnum = scl1
            
                if uiMenuItem==4:#calibration threshold
                    modename="Calibration impact"
                    actnum = dbgX
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 10), (100, 10),1)
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 20), (100, 20),1)#
                    pygame.draw.line(MAINSURF, UIWHITE, (10, 30), (100, 30),1)
                    pygame.draw.line(MAINSURF, UIGREEN, (10, 40), (100, 40),2)
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-dc, 10), 2, 0)
                    pygame.draw.circle(MAINSURF, UIWHITE, (40-int(0.15*scl), 20), 2, 0)#
                    pygame.draw.circle(MAINSURF, UIWHITE, (100-scl1, 30), 2, 0)
                    pygame.draw.circle(MAINSURF, UIBLUE, (100- int(0.08 * dbgX), 40), 3, 0)
                                                
                    #Handle +/- button clicks
                    if GPIO.input(fwPin)==0:
                        uiTimer = 32
                        dbgX+=4
                        if dbgX>=800:
                            dbgX=800
                        actnum = dbgX

                    elif GPIO.input(revPin)==0:
                            uiTimer = 32
                            dbgX-=4
                            
                            if dbgX<=0:
                                dbgX=0
                            actnum = dbgX

                 
                if uiMenuItem==5:#calibration frame counter
                    actnum = counterAct
                    modename="Calibration delay"
                    #Handle +/- button clicks
                    if GPIO.input(fwPin)==0:
                        uiTimer = 32
                        counterAct+=1
                        if counterAct>=counterMax:
                            counterAct=counterMax
                        actnum = counterAct

                    elif GPIO.input(revPin)==0:
                            uiTimer = 32
                            counterAct-=1
                            
                            if counterAct<=16:
                                counterAct=16
                            actnum = counterAct
            
                    pygame.draw.circle(MAINSURF, UIWHITE, (xmed,ymed), 10, 1)
                    pygame.draw.circle(MAINSURF, UIBLUE, (xmed, ymed-10), 3, 0)

                    step=numpy.deg2rad(counterAct)
                    cX = int(xmed+10*math.sin(step))
                    cY = int(ymed-10*math.cos(step))
                    pygame.draw.circle(MAINSURF, UIRED, (cX, cY), 3, 0)



                if uiMenuItem==6:#Color mode
                    modename="Color mode"
                    pygame.draw.circle(MAINSURF, UIWHITE, (64, 64), 10, 1)
                    actnum = uiColormode
                                        
                    #for a while, crosshair
                    if GPIO.input(fwPin)==0:
                        uiTimer = 32
                        uiColormode+=1
                        if(uiColormode)>uiColormodesMax:
                            uiColormode=0
                                                            
                    elif GPIO.input(revPin)==0:
                        uiTimer = 32
                        uiColormode-=1
                        if uiColormode<0:
                            uiColormode=uiColormodesMax
        
                    step=math.pi*2/uiColormodesMax
                    cX = int(64+10*math.sin(uiColormode*step))
                    cY= int(64-10*math.cos(uiColormode*step))
                                            
                    pygame.draw.circle(MAINSURF, UIWHITE, (cX, cY), 3, 0)
                        
        
            #Draw crosshair to mark screen center point
            #pygame.draw.line(MAINSURF, UIRED, (58, 55), (62, 55),1)
            #pygame.draw.line(MAINSURF, UIRED, (60, 53), (60, 57),1)


            if killtimer>16:
                modename="Shutdown"
                pygame.draw.rect(MAINSURF, UIGREEN, (110,40,10-killtimer,30), 0)
                
                if killtimer>32:
                    modename="Turning off now.."
                    pygame.draw.rect(MAINSURF, UIRED, (110,40,10-killtimer,30), 0)
        
            servicetext=""

            if cameramode==1:
                servicetext="blend"


            #Debug - count framerate
            #clock.tick(100)
            #fps = clock.get_fps()
            #print fps #OR
            #place fps text on framebuffer screen
            #text = myfont.render('FPS: %d' %(fps), False, (255, 0, 0))
            #newtextsurface=pygame.transform.rotate(text,180)
            #MAINSURF.blit(newtextsurface,(118,110))
            
            # place text on framebuffer screen
            text2 = myfont.render('%s' %(modename), False, (255, 255, 255))
            newtextsurface2=pygame.transform.rotate(text2,180)
            MAINSURF.blit(newtextsurface2,(50,90))
            
            
            # place text on framebuffer screen
            text3 = myfont.render('%s' %(servicetext), False, (255, 255, 255))
            newtextsurface3=pygame.transform.rotate(text3,180)
            MAINSURF.blit(newtextsurface3,(50,90))

#------->>>>FLIP SCREEN
            ####uncomment to flip
            #flipsurf=pygame.transform.rotate(MAINSURF,180)
            #MAINSURF.blit(flipsurf,(0,0))

            pygame.display.update()


#--------------------*************-------------------#
# End of main loop
#--------------------*************-------------------#

    def get_cal_image(self,dev):
    # Get the first cal image so calImage isn't null
        global status, calImage, calImagex, calimgI, imcalib
        status = 0
        while status != 1:
            # Read a raw frame
            ret9 = self.read_frame(dev)
            status = ret9[20]
            status1 = ret9[80]
        #print (status , status1)
    
        # Convert the raw 16 bit calibration data to a PIL Image
        calimgI = Image.frombytes("F", (208,156), ret9, "raw", "F;16")
        #Convert the PIL Image to an unsigned numpy float array
        im2arr = numpy.asarray(calimgI)
        
        #clamp values < 2000 to 2000
        im2arr = numpy.where(im2arr < 2000, 2000, im2arr)
        im2arrF = im2arr.astype('float')
        calImage = im2arrF
        imcalib=calImage
        
        return


    def get_image(self,dev):
        
        global calImage,calimgI, calImagex, status, scl, scl1, imcalib, imgain, actnum, dispimg
        global uiMenuItem, uiSelector, uiTimer, uiVisible, uiCrosshair, dbgX, uiColormode, cameramode
        global MAINSURF
        
        status = 0

        #  Wait for the next image frame, ID = 3 is a Normal frame
        while status != 3:
            # Read a raw frame
            ret9 = self.read_frame(dev)
            status = ret9[20]
            #print status
    
    
            # check for a new cal frame, if so update the cal image
            if status == 1:

                #Convert the raw 16 bit calibration data to a PIL Image
                calimgI = Image.frombytes("F", (208,156), ret9, "raw", "F;16")


                #Convert the PIL Image to an unsigned numpy float array
                im2arr = numpy.asarray(calimgI)

                # clamp values < 2000 to 2000
                im2arr = numpy.where(im2arr < 2000, 2000, im2arr)
                im2arrF = im2arr.astype('float')

                # Clamp pixel 40 to 2000 so it doesn't cause havoc as it rises to 65535
                im2arrF[0,40] = 2000

                #Add the row 207 correction (maybe) >>Looks like it needs to be applied to just the cal frame<<
                #self.add_207(im2arrF)

                #Zero out column 207
                im2arrF[:,206] = numpy.zeros(156)


                #Save the calibration image
                calImage = im2arrF
                imcalib = calImage
                print "calibrated"

        #If this is normal image data
        #Convert the raw 16 bit thermal data to a PIL Image
    
        #imgx = Image.fromstring("F", (208,156), ret9, "raw", "F;16")
        imgx = Image.fromstring("F", (208,156), ret9, "raw", "F;16N")

        #Convert the PIL Image to an unsigned numpy float array
        im1arr = numpy.asarray(imgx)

        #clamp values < 2000 to 2000
        im1arr = numpy.where(im1arr < 2000, 2000, im1arr)
        im1arrF = im1arr.astype('float')

        #Clamp pixel 40 to 2000 so it doesn't cause havoc as it rises to 65535
        im1arrF[0,40]  = 2000
        
        #Subtract the most recent calibration image from the offset image data
        #With both the cal and image as floats, the offset doesn't matter and
        #the following image conversion scales the result to display properly
        
        additionF = (im1arrF) + 200 + dbgX - calImage
        
        
        additionF[81,115] = 0
        additionF[123,99] = 60
        additionF[122,100] = 60
        additionF[122,99] = 60
        additionF[30,99]  = 60
        additionF[81,115] = 60
        additionF[81,116] = 60
        additionF[81,117] = 60
        additionF[81,118] = 60
        additionF[59,115] = 60
        additionF[60,115] = 60
        additionF[61,115] = 60
        additionF[38,69] = 0
        additionF[39,69] = 0
        additionF[40,69] = 0
        additionF[40,68] = 0

        #Crop or resize image to match display (pygame surface) area
        #noiselessF = additionF[14:142, 24:184]
        noiselessF = cv2.resize(additionF,(width, height), interpolation = cv2.INTER_LINEAR)
        
        bottom = scl
        top = scl1

        display_min = bottom * 4
        display_max = top * 16
    
        noiselessF.clip(display_min, display_max, out=noiselessF)
        noiselessF -= display_min
        noiselessF //= (display_max - display_min + 1) / 256.
        
        #convert and denoise image
        noiselessF = noiselessF.astype(numpy.uint8)
        noiselessF = ndimage.median_filter(noiselessF, 3)

        #convert color and rotate
        noiselessF = cv2.cvtColor(noiselessF,cv2.COLOR_GRAY2RGB)
        
        #cameramode=0
        
        if uiColormode==0:
            
            #noiselessF=self.applyCustomColorMap(noiselessF);
            noiselessF=noiselessF
        
        if uiColormode==1:
            
            noiselessF = cv2.applyColorMap(noiselessF, cv2.COLORMAP_HSV)
        
        if uiColormode==2:
            
            noiselessF = cv2.applyColorMap(noiselessF, cv2.COLORMAP_HOT)
        
        if uiColormode==3:
            
            noiselessF = cv2.applyColorMap(noiselessF, cv2.COLORMAP_COOL)
        
        if uiColormode==4:
            
            noiselessF = cv2.applyColorMap(noiselessF, cv2.COLORMAP_AUTUMN)

        noiselessF = numpy.rot90(noiselessF,1)
        noiselessF = numpy.flipud(noiselessF)

        dispimg=noiselessF


App=Thermal()
App.initialize()





