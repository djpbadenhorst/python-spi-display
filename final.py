import spidev
import time
import gpiod
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from gpiod.line import Direction, Value

#BL=/dev/gpiochip0 - 74
#RST=/dev/gpiochip0 - 76
#DC=/dev/gpiochip0 - 75
bl_freq=1000

spi = spidev.SpiDev()
spi.open(1, 0)

req = gpiod.request_lines(
    "/dev/gpiochip0",
    config={
        74: gpiod.LineSettings( direction=Direction.OUTPUT, output_value=Value.ACTIVE ), #BL
        75: gpiod.LineSettings( direction=Direction.OUTPUT, output_value=Value.INACTIVE ), #DC
        76: gpiod.LineSettings( direction=Direction.OUTPUT, output_value=Value.INACTIVE ), #RS
    },
)

def reset():
    req.set_value(76, Value.ACTIVE)
    time.sleep(0.01)
    req.set_value(76, Value.INACTIVE)
    time.sleep(0.01)
    req.set_value(76, Value.ACTIVE)
    time.sleep(0.01)
reset()

#cmd
def command(val):
    req.set_value(75, Value.INACTIVE)
    spi.writebytes([val])
def data(val):
    req.set_value(75, Value.ACTIVE)
    spi.writebytes([val])

command(0x36)
data(0xC8)
command(0xB0) 		
data(0xC0)  
command(0xB2) 			
data(0x2F)  
command(0xB3) 		
data(0x03) 
command(0xB6) 		
data(0x19)  
command(0xB7) 		
data(0x01)   
command(0xAC) 
data(0xCB) 
command(0xAB)  
data(0x0e) 
command(0xB4) 	
data(0x04) 
command(0xA8) 
data(0x19) 
command(0x3A) 		
data(0x05)  
command(0xb8) 
data(0x08) 
command(0xE8) 
data(0x24) 
command(0xE9) 
data(0x48) 
command(0xea) 	
data(0x22) 
command(0xC6) 
data(0x30) 
command(0xC7) 
data(0x18) 
command(0xF0) 
data(0x1F) 
data(0x28) 
data(0x04) 
data(0x3E) 
data(0x2A) 
data(0x2E) 
data(0x20) 
data(0x00) 
data(0x0C) 
data(0x06) 
data(0x00) 
data(0x1C) 
data(0x1F) 
data(0x0f) 
command(0xF1)  
data(0X00) 
data(0X2D) 
data(0X2F) 
data(0X3C) 
data(0X6F) 
data(0X1C) 
data(0X0B) 
data(0X00) 
data(0X00) 
data(0X00) 
data(0X07) 
data(0X0D) 
data(0X11) 
data(0X0f) 
command(0x21)  
command(0x11) 
command(0x29) 

# clear
_buffer = [0xff]*(128 * 128 * 2)
Xstart=2
Xend=128+2
Ystart=1
Yend=128+1
command(0x2A)
data((Xstart)>>8& 0xff)
data((Xstart)   & 0xff)
data((Xend-1)>>8& 0xff)
data((Xend-1)   & 0xff)
command(0x2B)
data((Ystart)>>8& 0xff)
data((Ystart)   & 0xff)
data((Yend-1)>>8& 0xff)
data((Yend-1)   & 0xff)
command(0x2C)

req.set_value(75, Value.ACTIVE)
for i in range(0,len(_buffer),4096):
    spi.writebytes(_buffer[i:i+4096])	        
# cleared

#image1 = Image.new("RGB", (128,128 ), "WHITE")
#Font1 = ImageFont.truetype("./Font/Font01.ttf",20)
#Font2 = ImageFont.truetype("./Font/Font01.ttf",25)
#Font3 = ImageFont.truetype("./Font/Font02.ttf",30)
#draw.rectangle([(0,0),(128,25)],fill = "BLUE")
#draw.text((13, 0), 'Hello world', fill = "RED",font=Font1)
#draw.rectangle([(0,25),(128,55)],fill = "RED")
#draw.text((3, 25), 'WaveShare', fill = "WHITE",font=Font2)
#draw.text((0, 55), '12345678', fill = "GREEN",font=Font3)
#text= u"微雪电子"
#draw.text((0, 85),text, fill = "BLUE",font=Font3)
#image1=image1.rotate(0)

#ffmpeg -f x11grab -i :0 -frames:v 1 -q:v 2 output.jpg
image1 = Image.open('/home/djpb/output.jpg')

def ShowImage(image1):
    imwidth, imheight = image1.size
    img = np.asarray(image1)
    pix = np.zeros((128,128,2), dtype = np.uint8)
    
    pix[...,[0]] = np.add(np.bitwise_and(img[...,[0]],0xF8),np.right_shift(img[...,[1]],5))
    pix[...,[1]] = np.add(np.bitwise_and(np.left_shift(img[...,[1]],3),0xE0),np.right_shift(img[...,[2]],3))
    
    pix = pix.flatten().tolist()
    Xstart=2
    Xend=128+2
    Ystart=1
    Yend=128+1
    command(0x2A)
    data((Xstart)>>8& 0xff)
    data((Xstart)   & 0xff)
    data((Xend-1)>>8& 0xff)
    data((Xend-1)   & 0xff)
    command(0x2B)
    data((Ystart)>>8& 0xff)
    data((Ystart)   & 0xff)
    data((Yend-1)>>8& 0xff)
    data((Yend-1)   & 0xff)
    command(0x2C)
    
    req.set_value(75, Value.ACTIVE)
    for i in range(0,len(pix),4096):
        spi.writebytes(pix[i:i+4096])		

ShowImage(image1)
ShowImage(image1.resize((128,128)))
ShowImage(image1.crop((0, 0, 250, 250)).resize((128,128)))


############## PWM
from threading import Thread

def cycle():
    while True:
        print('a')
        req.set_value(74, Value.ACTIVE)
        time.sleep(1)
        req.set_value(74, Value.INACTIVE)
        time.sleep(1)

t = Thread(target = cycle)
t.setDaemon(True)
t.start()
###########3
class MultiThreading:

    def __init__(self):
        self.thread = None
        self.started = True
    def threaded_program(self):
        while self.started:
            print("running")
            # time.sleep(10)
    def run(self):
        self.thread = threading.Thread(target=self.threaded_program, args=())
        self.thread.start()
    def stop(self):
        self.started = False
        self.thread.join()
##########3
from threading import Thread

class myClassA(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while True:
            print 'A'

class myClassB(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while True:
            print 'B'


myClassA()
myClassB()
while True:
    pass
###
    def clear(self):
        """Clear contents of image buffer"""
        _buffer = [0xff]*(self.width * self.height * 2)
        self.SetWindows ( 0, 0, self.width, self.height)
        self.digital_write(self.DC_PIN,self.GPIO.HIGH)
        for i in range(0,len(_buffer),4096):
            self.spi_writebyte(_buffer[i:i+4096])	        
###
    def ShowImage(self,Image):
        """Set buffer to value of Python Imaging Library image."""
        """Write display buffer to physical display"""
                
        imwidth, imheight = Image.size
        if imwidth != self.width or imheight != self.height:
            raise ValueError('Image must be same dimensions as display \
                ({0}x{1}).' .format(self.width, self.height))
        img = self.np.asarray(Image)
        pix = self.np.zeros((self.height,self.width,2), dtype = self.np.uint8)
        
        pix[...,[0]] = self.np.add(self.np.bitwise_and(img[...,[0]],0xF8),self.np.right_shift(img[...,[1]],5))
        pix[...,[1]] = self.np.add(self.np.bitwise_and(self.np.left_shift(img[...,[1]],3),0xE0),self.np.right_shift(img[...,[2]],3))
        
        pix = pix.flatten().tolist()
        self.SetWindows ( 0, 0, self.width, self.height)
        self.digital_write(self.DC_PIN,self.GPIO.HIGH)
        for i in range(0,len(pix),4096):
            self.spi_writebyte(pix[i:i+4096])		
###
    def SetWindows(self, Xstart, Ystart, Xend, Yend):
        #set the X coordinates
        Xstart=Xstart+2
        Xend=Xend+2
        Ystart=Ystart+1
        Yend=Yend+1

        self.command(0x2A)
        self.data((Xstart)>>8& 0xff)               #Set the horizontal starting point to the high octet
        self.data((Xstart)   & 0xff)      #Set the horizontal starting point to the low octet
        self.data((Xend-1)>>8& 0xff)        #Set the horizontal end to the high octet
        self.data((Xend-1)   & 0xff) #Set the horizontal end to the low octet 
        
        #set the Y coordinates
        self.command(0x2B)
        self.data((Ystart)>>8& 0xff)
        self.data((Ystart)   & 0xff)
        self.data((Yend-1)>>8& 0xff)
        self.data((Yend-1)   & 0xff)

        self.command(0x2C) 
###
####
from PIL import ImageGrab
screenshot = ImageGrab.grab()
import pyscreenshot as ImageGrab
im = ImageGrab.grab()
ffmpeg -f x11grab -i :0 -frames:v 1 -q:v 2 output.jpg
ffmpeg -video_size 800x480 -framerate 10 -f x11grab -i :0 output.mp4

###
#pip install ffmpeg-python


ffmpeg -re -video_size 800x480 -f x11grab -i :0 -r 10 -f mpegts udp://djpb-yoga.local:8554?pkt_size=1316
ffmpeg -re -video_size 800x480 -f x11grab -i :0 -r 10 -f mpegts udp://localhost:8554?pkt_size=1316
ffplay udp://localhost:8554
import cv2

cap = cv2.VideoCapture()
cap.open("udp://localhost:8554")
tmp = cap.read()
from PIL import Image
im = Image.fromarray(tmp[1[])
im.save("your_file.jpeg")
