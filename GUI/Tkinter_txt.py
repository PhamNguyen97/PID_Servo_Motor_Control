import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2TkAgg
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np 

import serial
from threading import Timer
import threading
import time

# from Serial_function import serial_buffer,reader,writer


import tkinter as tk 
from tkinter import ttk 




plot_stype = {1:'ggplot',2:'dark_background'}



LARGE_FONT = ('Verdana',14)
style.use(plot_stype[1])


class DataHouse:
	def __init__(self):
		self.num_sample = 1000
		self.posdata = list(np.zeros(self.num_sample,np.int))

		self.old_pos = np.zeros(self.num_sample-1,np.int)

		self.xpos = np.arange(0,50,0.05)
		print(len(self.xpos))
		self.xvec = np.arange(0,50-0.05,0.05)
		self.Kp = '+00.0000000'
		self.Ki = '+00.0000000'
		self.Kd = '+00.0000000'
		self.Sp = '+0000000.00'

		self.graph = Figure(figsize = (9.7,7.5),dpi = 100)
		self.gs = gridspec.GridSpec(3, 1)
		 
		self.pos_graph = self.graph.add_subplot(self.gs[0:2,0])
		self.pos_graph.set_title('Position')
		self.pos_graph.set_xlabel('Time (s)')
		self.pos_graph.set_ylabel('Degree')

		self.vec_graph = self.graph.add_subplot(self.gs[2,0])
		self.vec_graph.set_title('Veclocity')
		self.vec_graph.set_xlabel('Time (s)')
		self.vec_graph.set_ylabel('Degree/s')

		self.graph.tight_layout()


	def animate(self,i):

		self.pos_graph.clear()
		self.pos_graph.plot(list(self.xpos),list(self.posdata),'r')
		self.pos_graph.set_title('Position')
		self.pos_graph.set_xlabel('Time (s)')
		self.pos_graph.set_ylabel('Degree')
		self.pos_graph.autoscale(True)

		old_pos = self.posdata[:-1]
		new_pos = self.posdata[1:] 
		vec_data = list((np.array(new_pos)-np.array(old_pos))*50)

		self.vec_graph.clear()
		self.vec_graph.plot(list(self.xvec),list(vec_data),'b')
		self.vec_graph.set_title('Veclocity')
		self.vec_graph.set_xlabel('Time (s)')
		self.vec_graph.set_ylabel('Degree/s')
		self.vec_graph.autoscale(True)
		# self.vec_graph.set_ylim(top = 500000,bottom = -500000)
		



PID_datahouse = DataHouse()



##############SERIAL#################################
class reader (threading.Thread):
	def __init__(self, threadID, name, exitFlag = 0,ser = None,sampletime = 0.5):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.exitFlag = exitFlag
		self.inBuffer = ''
		self.ser = ser
		self.sampletime = sampletime
		

        
	def run(self):
		if not self.ser == None:
			print ("Starting " + self.name)
			self.read_serial(self.ser,self.sampletime,self.inBuffer)
			print ("Exiting " + self.name)
	    
	def read_serial(self,ser,sampleTime,buffer):
		if ser.is_open:
			while (self.exitFlag==0):
				self.size = ser.inWaiting()
				if self.size:
					try:
						data = (ser.read(self.size)).decode("utf-8")
						# print(data)
						self.inBuffer +=data
						self.saveData()
					except:
						# self.inBuffer += '^{}$'.format(PID_datahouse.posdata[-1])
						pass
					# print(time.asctime(),data,'-',len(self.inBuffer))
					# print(PID_datahouse.posdata)
				time.sleep(sampleTime)
	def saveData(self):
		receive_frames = self.inBuffer.split('$')

		for i in range(len(receive_frames)-1):
			PID_datahouse.posdata[0:-1] = PID_datahouse.posdata[1:]
			PID_datahouse.posdata[-1]= (float(receive_frames[i][1:])/9000*360)
			PID_datahouse.xpos +=0.05;
			PID_datahouse.xvec +=0.05;
		self.inBuffer = receive_frames[-1]




	    
class writer (threading.Thread):
	def __init__(self, threadID, name, exitFlag = 0,outBuffer = 'start',ser = None,sampletime = 0.5):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.exitFlag = exitFlag
		self.outBuffer = 0
		self.outcount = 0
		self.ser = ser
		self.sampletime = sampletime

	def run(self):
		if not self.ser == None:
			print ("Starting " + self.name)
			self.write_serial(self.ser,self.sampletime)
			print ("Exiting " + self.name)

	def write_serial(self,ser,sampleTime):
		if ser.is_open:
			while (self.exitFlag==0):
				try:
					# Kp_str = self.str_format(str(float(PID_datahouse.Kp)))
					# Ki_str = self.str_format(str(float(PID_datahouse.Ki)))
					# Kd_str = self.str_format(str(float(PID_datahouse.Kd)))
					# Sp_str = self.str_format(str(float(PID_datahouse.Sp)),7,2)

					string_out = '^#{}#{}#{}#{}$'.format(PID_datahouse.Kp,PID_datahouse.Ki,PID_datahouse.Kd,PID_datahouse.Sp)
					self.outBuffer +=1
					ser.write(str.encode(string_out))
					# self.outcount = self.outcount+1
					# print('-write','-',string_out)
				except Exception as e:
					print(e)
				time.sleep(sampleTime)

	# def str_format(self,string,nmax=2,tpmax=7):
	# 	pos = string[0]
	# 	if pos!='-':
	# 		n,tp = string.split('.')
	# 		pos = '+'
	# 	else:
	# 		n,tp = string[1::].split('.')
	# 		pos = '-'

	# 	if len(n)<nmax:
	# 		while len(n)<nmax:
	# 			n = '0'+n
	# 	else:
	# 		n = n[-nmax:]
	# 	n = pos+n

	# 	if len(tp)>tpmax:
	# 		tp = tp[:tpmax]
	# 	else:
	# 		while len(tp)<tpmax:
	# 			tp = tp+'0' 
	# 	return '{}.{}'.format(n,tp) 

class serial_buffer:
    def __init__(self,com = 'COM1',rate = 9600,timeout = 1,read_sampletime = 0.1,write_sampletime = 0.5):
        self.com = com
        self.baudrate = rate
        self.timeout = timeout
        self.read_sampletime = read_sampletime
        self.write_sampletime = write_sampletime
        self.ser = serial.Serial(port = self.com,
                                 baudrate = self.baudrate,
                                 timeout = self.timeout)
        self.inbuffer = []
        self.outbuffer = '!!!!'
        self.start_read = 0
        self.start_write = 0

        self.reader = reader(1,'thread-1',0,ser = self.ser,sampletime = self.read_sampletime)
        self.writer = writer(2,'thread-2',0,ser = self.ser,sampletime = self.write_sampletime)
        
        if not self.ser.is_open:
            self.ser.open()
            
    def serial_start(self):
        self.reader.start()
        self.writer.start()

    def serial_stop(self):
        self.reader.exitFlag = 1
        self.writer.exitFlag = 1

        self.reader.join()
        self.writer.join()
        self.ser.close()
################################################################################






class MyPIDMonitering_app(tk.Tk):
	def __init__(self,*args,**kwargs):
		tk.Tk.__init__(self,*args,**kwargs)

		tk.Tk.iconbitmap(self,default = "IOTicon2.ico")
		tk.Tk.wm_title(self,"PID_G5")


		container = tk.Frame(self)
		container.pack(side='top',fill = 'both',expand = True)
		container.grid_rowconfigure(0,weight = 1)
		container.grid_columnconfigure(0,weight = 1)

		

		self.frames = {}


		frame = StartPage(container,self)
		self.frames[StartPage] = frame 
		frame.grid(row=0, column=0,sticky = 'nsew')

		self.show_frame(StartPage)

	def show_frame(self,cont):
		frame = self.frames[cont]
		frame.tkraise()


def qf(stingToPrint):
	print(stingToPrint)


class StartPage(tk.Frame):
	def __init__(self,parent,controller):
		tk.Frame.__init__(self,parent)
		self.configure(background='white')
		w, h = self.winfo_screenwidth(), self.winfo_screenheight()
		xunit =	w/1000
		yunit = h/1000

		label = tk.Label(self,text="PID Controller\nand\nMonitering\n\nGROUP 5",font = LARGE_FONT,background = 'white')
		label.place(x = xunit*20,y = yunit*20)

		self.KpLabel = tk.Label(self,text="Kp:",background = 'white')
		self.KpLabel.place(x = xunit*10,y = yunit*325)
		self.KiLabel = tk.Label(self,text="Ki:",background = 'white')
		self.KiLabel.place(x = xunit*10,y = yunit*375)
		self.KdLabel = tk.Label(self,text="Kd:",background = 'white')
		self.KdLabel.place(x = xunit*10,y = yunit*425)
		self.Setpoint = tk.Label(self,text="Sp:",background = 'white')
		self.Setpoint.place(x = xunit*10,y = yunit*475)

		CurrentKp = tk.Label(self,text=      "Current Kp:",background = 'white')
		CurrentKp.place(x = xunit*10,y = yunit*625)
		self.CurrentKp = tk.Label(self,text=str(PID_datahouse.Kp),background = 'white')
		self.CurrentKp.place(x = xunit*85,y = yunit*625)

		CurrentKi = tk.Label(self,text=      "Current Ki:",background = 'white')
		CurrentKi.place(x = xunit*10,y = yunit*675)
		self.CurrentKi = tk.Label(self,text=str(PID_datahouse.Ki),background = 'white')
		self.CurrentKi.place(x = xunit*85,y = yunit*675)

		CurrentKd = tk.Label(self,text=      "Current Kd:",background = 'white')
		CurrentKd.place(x = xunit*10,y = yunit*725)
		self.CurrentKd = tk.Label(self,text=str(PID_datahouse.Kd),background = 'white')
		self.CurrentKd.place(x = xunit*85,y = yunit*725)

		CurrentSetpoint = tk.Label(self,text="Current Setpoint: ",background = 'white')
		CurrentSetpoint.place(x = xunit*10,y = yunit*775)
		self.CurrentSetpoint = tk.Label(self,text=str(PID_datahouse.Sp),background = 'white')
		self.CurrentSetpoint.place(x = xunit*85,y = yunit*775)

		self.KpEntry = ttk.Entry(self)
		self.KpEntry.place(x = xunit*30,y = yunit*325)
		self.KiEntry = ttk.Entry(self)
		self.KiEntry.place(x = xunit*30,y = yunit*375)
		self.KdEntry = ttk.Entry(self)
		self.KdEntry.place(x = xunit*30,y = yunit*425)
		self.SpEntry = ttk.Entry(self)
		self.SpEntry.place(x = xunit*30,y = yunit*475)



		button1 = ttk.Button(self,text = 'Connect',
			command = lambda: self.connectingFunc())
		button1.place(x = xunit*30,y = yunit*200)

		button2 = ttk.Button(self,text = 'Disconnect',
			command = lambda: self.disconnectingFunc())
		button2.place(x = xunit*30,y = yunit*250)

		button3 = ttk.Button(self,text = 'Update',
			command =lambda: self.updateFromEntry())
		button3.place(x = xunit*30,y = yunit*525)

		button3 = ttk.Button(self,text = 'Quit',
			command = lambda: self.quitFunc(controller))
		button3.place(x = xunit*30,y = yunit*875)


		self.ErrorValue = tk.Label(self,text=  "No Error",background = 'Green',width = 15,anchor = 'w')
		self.ErrorValue.place(x = xunit*100,y = yunit*526)

		self.ErrorConnect = tk.Label(self,text="Not Connected",background = 'red',width = 15,anchor = 'w')
		self.ErrorConnect.place(x = xunit*100,y = yunit*200)



		canvas = FigureCanvasTkAgg(PID_datahouse.graph,self)
		canvas.show()

		toolbar = NavigationToolbar2TkAgg(canvas,self)
		toolbar.update()
		canvas._tkcanvas.place(x = xunit*250,y = 00)


	def updateFromEntry(self):
		try:
			Kp = float(self.KpEntry.get())
			Ki = float(self.KiEntry.get())
			Kd = float(self.KdEntry.get())
			Sp = float(self.SpEntry.get())

			PID_datahouse.Kp = self.str_format(str(float(Kp)))
			PID_datahouse.Ki = self.str_format(str(float(Ki)))
			PID_datahouse.Kd = self.str_format(str(float(Kd)))
			PID_datahouse.Sp = self.str_format(str(float(Sp)),7,2)

			# PID_datahouse.Kp = Kp
			# PID_datahouse.Ki = Ki 
			# PID_datahouse.Kd = Kd 
			# PID_datahouse.Sp = Sp

			self.CurrentKp['text'] =PID_datahouse.Kp
			self.CurrentKi['text'] =PID_datahouse.Ki
			self.CurrentKd['text'] =PID_datahouse.Kd
			self.CurrentSetpoint['text'] =PID_datahouse.Sp

			self.ErrorValue['text'] = 'No Error'
			self.ErrorValue['background'] = 'Green'

		except Exception:				
			self.ErrorValue['text'] = 'Invalid Value'
			self.ErrorValue['background'] = 'red'
	def connectingFunc(self):
		try:
			self.ser = serial_buffer(com = 'COM6',read_sampletime = 0.5,write_sampletime = 1)
			self.ser.serial_start()
			self.ErrorConnect['text'] = 'Connected'
			self.ErrorConnect['background'] = 'Green'
		except Exception:
			if self.ErrorConnect['text']!="Connected":
				self.ErrorConnect['text'] = "Can't Connected"
				self.ErrorConnect['background'] = 'Red'
	def disconnectingFunc(self):
		try:
			self.ser.serial_stop()
		except:
			pass
		self.ErrorConnect['text'] = "No Connection"
		self.ErrorConnect['background'] = 'Red'

	def quitFunc(self,controller):
		try:
			self.ser.serial_stop()
		except: 
			pass
		controller.destroy()

	def str_format(self,string,nmax=2,tpmax=7):
		pos = string[0]
		if pos!='-':
			n,tp = string.split('.')
			pos = '+'
		else:
			n,tp = string[1::].split('.')
			pos = '-'

		if len(n)<nmax:
			while len(n)<nmax:
				n = '0'+n
		else:
			n = n[-nmax:]
		n = pos+n

		if len(tp)>tpmax:
			tp = tp[:tpmax]
		else:
			while len(tp)<tpmax:
				tp = tp+'0' 
		return '{}.{}'.format(n,tp) 







		

if __name__ == '__main__':

	app = MyPIDMonitering_app()
	w, h = app.winfo_screenwidth(), app.winfo_screenheight()
	# app.overrideredirect(1)
	app.geometry("%dx%d+0+0" % (w,h))
	app.configure(background='white')


	ani = animation.FuncAnimation(PID_datahouse.graph,PID_datahouse.animate,interval = 500)
	app.mainloop()


