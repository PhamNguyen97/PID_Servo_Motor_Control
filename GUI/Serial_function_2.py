import serial
from threading import Timer
import threading
import time


class reader (threading.Thread):
    def __init__(self, threadID, name, exitFlag = 0,inBuffer = [],ser = None):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.exitFlag = exitFlag
        self.inBuffer = inBuffer
        self.ser = ser

        
    def run(self):
        if not self.ser == None:
            print ("Starting " + self.name)
            self.read_serial(self.ser,0.2,self.inBuffer)
            print ("Exiting " + self.name)
        
    def read_serial(self,ser,sampleTime,buffer):
        if ser.is_open:
            while (self.exitFlag==0):
                size = ser.inWaiting()
                if size:
                    data = ser.read(size).decode('ascii')
                    
                    self.inBuffer.append(data)
                    print(time.asctime(),data,'-',len(self.inBuffer))
                time.sleep(sampleTime)

        
class writer (threading.Thread):
    def __init__(self, threadID, name, exitFlag = 0,outBuffer = 'start',ser = None):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.exitFlag = exitFlag
        self.outBuffer = 0
        self.outcount = 0
        self.ser = ser
        
    def run(self):
        if not self.ser == None:
            print ("Starting " + self.name)
            self.write_serial(self.ser,1)
            print ("Exiting " + self.name)
        
    def write_serial(self,ser,sampleTime):
        if ser.is_open:
            while (self.exitFlag==0):
                string_out = '^{}$'.format('0'+str(self.outBuffer) if self.outBuffer<10 else str(self.outBuffer))
                self.outBuffer +=1
                ser.write(str.encode(string_out))
                self.outcount = self.outcount+1
                print(time.asctime(),'-write','-',self.outcount)
                time.sleep(sampleTime)

class serial_buffer:
    def __init__(self,com = 'COM1',rate = 9600,timeout = 1,sampletime = 0.5):
        self.com = com
        self.baudrate = rate
        self.timeout = timeout
        self.sampletime = sampletime
        self.ser = serial.Serial(port = self.com,
                                 baudrate = self.baudrate,
                                 timeout = self.timeout)
        self.inbuffer = []
        self.outbuffer = '!!!!'
        self.start_read = 0
        self.start_write = 0

        self.reader = reader(1,'thread-1',0,ser = self.ser,inBuffer = self.inbuffer)
        self.writer = writer(2,'thread-2',0,ser = self.ser)
        
        if not self.ser.is_open:
            self.ser.open()
            
    def read(self):
        self.reader.start()
        self.writer.start()

    def read_stop(self):
        self.reader.exitFlag = 1
        self.writer.exitFlag = 1

        self.reader.join()
        self.writer.join()
        self.ser.close()
            
            

# ser_buff = serial_buffer(com = 'COM2')
# ser_buff.read()
# while True:#(len(ser_buff.inbuffer)<50 or ser_buff.writer.outcount <50):
#     #print(ser_buff.writer.outcount)
#     #print(len(ser_buff.inbuffer))
    # None

# ser_buff.read_stop()



            
        

        
