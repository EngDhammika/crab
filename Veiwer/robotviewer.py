import Tkinter as tk
import ttk
import matplotlib
import time

matplotlib.use("TkAgg")

from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2TkAgg
from matplotlib.figure import Figure
#......................................
LARGE_FONT = ("Verdana",12)# font type
style.use("ggplot")        #matplotlib plot type
#...............................................#...................................Tkinter face
class Display (tk.Tk):
#............................................................
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.wm_title(self,"Sea of BTC client") # set title

        container = tk.Frame(self)   # creating container for Frame

        container.pack(side="top",fill="both", expand = True)
        container.grid_rowconfigure(0,weight=1)
        container.grid_columnconfigure(0,weight=1)


        self.frames = {}

        for F in (StartPage,Graps): # create the frames

            frame = F(container,self)
            self.frames[F] = frame

            frame.grid(row=0,column= 0, sticky="nsew")

        self.show_frame(StartPage) # begin the main page

#...........................................................genarate frame

    def show_frame(self,frame_name ):
        frame = self.frames[frame_name]
        frame.tkraise()
#.................................................................
class StartPage(tk.Frame):

    def __init__(self,parent,controler):
        tk.Frame.__init__(self,parent)
        label = ttk.Label(self,text="System", font=LARGE_FONT)
        label.pack(pady = 10 , padx=10)
        button2 = ttk.Button(self, text="Agree",
                             command=lambda: controler.show_frame(Graps))

        button2.pack()
#..................................................................
class Graps(tk.Frame):

    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)

        label = ttk.Label(self,text="Graph_Page", font=LARGE_FONT)
        label.pack(pady = 10 , padx=10)

        f = Figure(figsize=(10,10), dpi=100)
        a = f.add_subplot(111)
        a.plot([1,1,4,5,4],[5,4,3,6,7])

        canvas = FigureCanvasTkAgg(f,self)
        canvas.show()
        canvas.get_tk_widget().pack(side=tk.TOP,fill=tk.BOTH,expand=True)
#......................................................
if __name__ =='__main__':

    app = Display()
    #ani2 = animation.FuncAnimation(frame1, draw, interval=200)
    app.mainloop()
#.........................................................

