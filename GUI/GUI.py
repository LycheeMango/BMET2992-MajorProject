from serialcomm import SerialReader
from visualAlarm import create_led, set_led_color
import PySimpleGUI as sg
import threading
import numpy as np
from tkinter import *
from random import randint
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, FigureCanvasAgg
from matplotlib.figure import Figure
import tkinter as Tk
from datetime import datetime
import csv
import time
import pandas as pd



def addPlot(canvasElement):
    # create a figure
    fig = Figure(figsize=(5,4))
    # create a set of axes on the figure
    ax = fig.add_subplot(1,1,1)
    random_data = np.random.uniform(0, 10, 20)
    ax.plot(random_data, marker='o', linestyle='-', color='b', label='Random Data')
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_title("Random Data Plot")
    ax.legend()
    canvas = canvasElement.TKCanvas
    # place the figure on the canvas
    figAgg = FigureCanvasTkAgg(fig, canvas)
    figAgg.draw()
    figAgg.get_tk_widget().pack(side='top', fill='both', expand=1)
    
    
    return ax, figAgg


def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

def update_graph(shared_data):
    shared_data['task_2_running'] = True
    
    print('update graph')

def printWithTimeTag(msg):
    current_time = datetime.now()
    formatted_time = current_time.strftime("%a %b %d %H:%M:%S %Y")
    log_string = f"{formatted_time}: {msg}"
    print(log_string)



class SerialGUI:
    def __init__(self):
        self.reader = None
        self.thread = None
        self.window = self.setup_window()
        self.shared_data = {
            'slider_value': 2,
            'task_2_running': False
        }
        self.task_2_thread = None
        self.led_ids = {
            'low_pulse': None,
            'high_pulse': None,
            'poor_recording': None
        }


    def long_running_task_2(self, wave, x, y, canvas, fig, fig_agg, ax):
        # draw graph
        self.shared_data['task_2_running'] = True
        iterations = self.shared_data['slider_value']
        x = np.linspace(0, 5, 250)
        y = wave


        for i in range(iterations):

            if not self.shared_data['task_2_running']:
                break
            print(f"Task 2 running... {i+1}/{iterations}")
            # draw the graph
            y = y[50:]

            if len(y) < 250:
                y = wave

            ax.clear()
            ax.cla()
            ax.plot(x, y[0:250])

            fig_agg.draw()
            self.window.refresh()
            sg.time.sleep(0.1)

        print('Task 2 completed')
        self.shared_data['task_2_running'] = False

    def setup_window(self):
        sg.theme('BlueMono')

        com=('COM3','COM4','COM5','COM6','COM7','COM8','COM9','COM10','COM11','COM12','COM13')
        
        baud_rate=('9600','38400','115200') #at least there are two baud rate option

        parameter_serial_layout=[
                [sg.Text('Step 1. Select a comport for serial connection:', text_color='Red', font=20)],
                [sg.Text('COM port:'), sg.InputCombo(com,default_value='COM4',size=(15, 1), key ='-COMPORT-')], 
                [sg.Text('Or Input PortName: '), sg.InputText(key='-MACCOMPORT-', size=(9, 1), default_text='/dev/tty.ESP32XX0H32'), sg.Button('Submit')],
                [sg.Text('Step 2. Enter Baudrate:', text_color='Red', font=20)],
                [sg.Text('baud rate:'), sg.InputCombo(baud_rate,default_value='115200',size=(15, 1),key='-BAUDRATE-')],
                [sg.Text('Step 3. Connect Bluetooth: ', text_color='red', font=20)], 
                [sg.Button('Connect to Serial Port', size=(20,1))],
                [sg.Text('Stop reading from Bluetooth: ', text_color='Purple', font=20)], 
                [sg.Button('Stop Read Data', size=(20,1))]
            ]
        parameter_save_file=[
            [sg.InputText(key='-PathFileLog-',size=(115,5), default_text='./tmp/log.txt'),
            sg.FileSaveAs(initial_folder='./tmp')]]
        

        parameter_patient_info=[
            [sg.Text('Step 1. Input Patient ID (optional)', text_color='Purple', font=20)],
            [sg.InputText(key='-PatientID-', size=(9, 1), default_text='000')],
            [sg.Text('Step 2. Input Patient CSV file path (optional)', text_color='Purple', font=20)],
            [sg.Text('CSV file path: ', font=20)],
            [sg.InputText(key='-CSVFilePath-', size=(9, 1), default_text='./tmp/sensor_data_tmp.csv'), sg.FileSaveAs(initial_folder='./tmp')]
        ]

        # Layout with LED indicators
        visual_alarm_holder = [
            [sg.Text('Low Pulse', size=(10, 1)), sg.Canvas(size=(20, 20), key='-low_pulse-')],
            [sg.Text('High Pulse', size=(10, 1)), sg.Canvas(size=(20, 20), key='-high_pulse-')],
            [sg.Text('Poor Recording', size=(10, 1)), sg.Canvas(size=(20, 20), key='-poor_recording-')]
        ]

        clear_output = [
            [sg.Button('Clear Output', size=(30,1)), sg.Button('Save Output', size=(30,1))]]

        NUM_DATAPOINTS = 10000

        tab1 = [[sg.Text('Animated pulse wave', size=(40, 1),
                justification='center', font='Helvetica 20')],
              [sg.Canvas(size=(40, 40), key='-CANVAS-')],
              [sg.Text('Progress through the data')],
              [sg.Slider(range=(0, NUM_DATAPOINTS), size=(60, 10),
                orientation='h', key='-SLIDER-',enable_events=True)],
              [sg.Text('Number of data points to display on screen')],
               [sg.Slider(range=(0, 100), default_value=40, size=(40, 10),
                    orientation='h', key='-SLIDER-DATAPOINTS-', enable_events=True)],
              [sg.Button('Start', size=(10, 1), font='Helvetica 14'),
              sg.Button('Stop', size=(10, 1),font='Helvetica 14')]]
              
        tab2 = [[sg.Text('Animated BPM', size=(40, 1),
                justification='center', font='Helvetica 20')],
              [sg.Canvas(size=(40, 40), key='-CANVAS-')],
              [sg.Button('Start', size=(10, 1), font='Helvetica 14'),
              sg.Button('Stop', size=(10, 1),font='Helvetica 14')]]
        
        tabgrp = [[sg.TabGroup([[
            sg.Tab('Graph',tab1),
            sg.Tab('Graph again', tab2)
        ]], selected_background_color='LightGreen', title_color='Black')]]

        combine_frame = [[sg.Frame('Patient Info', parameter_patient_info)],[sg.Frame('Visual Alarms', visual_alarm_holder)], [sg.Frame('Parameter Serial Port',parameter_serial_layout)]]
        # combine_frame = [[sg.Frame('Visual Alarms', visual_alarm_holder)], [sg.Frame('Parameter Serial Port',parameter_serial_layout)]]

        layout1 = [
            [sg.Frame('Graphs', tabgrp), sg.Frame('',combine_frame)],
            [sg.Output(size=(200, 6),key = '_output_')],
            [sg.Frame('Log Path File Name',parameter_save_file)],
            [sg.Frame('',clear_output)]
                ]
        return sg.Window('Read Serial Data', layout1, size=(1000,800),finalize=True)

    def run(self):
        # canvasElement = self.window['plot']
        canvas_elem = self.window['-CANVAS-']
        slider_elem = self.window['-SLIDER-']
        canvas = canvas_elem.TKCanvas
        # get the pulse data
        SAMPLE_TIME = 0.02
        data_df = pd.read_csv('./test_data/pulse.csv', header=None, names=['package_sequence', 'PPG'])
        wave = data_df['PPG']

        with open('./test_data/pulse.csv') as csvDataFile:
            # read file as csv file
            csvReader = csv.reader(csvDataFile)
            wave = []
            pulse = []
            rate = []
            # append each row to data
            for row in csvReader:
                wave.append(int(row[0]))
                pulse.append(float(row[1]))
                rate.append(60.0/float(row[1]))

        x = np.linspace(0, 5, 250)
        y = wave
        fig, ax = Figure(figsize=(5, 4)), None
        if not ax:
            ax = fig.add_subplot(111)
        ax.plot(x, y[0:250])
        fig_agg = draw_figure(canvas, fig)

        # Create LEDs:

        for led_name, led_id in self.led_ids.items():
            led_canvas = self.window[f'-{led_name}-']
            self.led_ids[led_name] = create_led(led_canvas, 0, 0, 19, 19, 'gray')

        while True:
            event, values = self.window.read()

            if event in (sg.WIN_CLOSED, 'Exit'):
                printWithTimeTag("clicked exit.")
                break
            elif event == 'Submit':
                self.window['-COMPORT-'].Update(values['-MACCOMPORT-'])
                printWithTimeTag('Set SerialPortName as ' + values['-MACCOMPORT-'])
                
            elif event.startswith('Connect') and not self.thread:
                self.reader = SerialReader(values['-COMPORT-'], values['-BAUDRATE-'], values['-PathFileLog-'], values['-CSVFilePath-'])
                self.reader.looprun = 1
                self.thread = threading.Thread(target=self.reader.read_serial, daemon=True)
                self.thread.start()
            
            elif event == 'Stop':
                # self.shared_data['slider_value'] = int(values['-SLIDER-DATAPOINTS-'])
                if self.shared_data['task_2_running']:
                    self.shared_data['task_2_running'] = False
                    self.task_2_thread.join()

            elif event == 'Start' :
                if not self.shared_data['task_2_running']:
                    print('start')
                    self.shared_data['slider_value'] = int(values['-SLIDER-DATAPOINTS-'])
                    self.task_2_thread = threading.Thread(target = self.long_running_task_2,args=[wave, x, y, canvas, fig, fig_agg, ax], daemon=True)
                    self.task_2_thread.start()
            
            elif event == 'Stop Read Data':
                print('Stop reading from bluetooth')
            
            elif event == 'Clear Output':
                self.window['_output_'].update('')
            
            elif event == 'Save Output':
                output = self.window['_output_'].get()
                file_log = open(values['-PathFileLog-'], "a")
                file_log.writelines(output+"\n") # write/append new data from serial port to file
                file_log.close()
            

            # detect led status and set leds:

            
                
        self.window.close()

if __name__ == '__main__':
    gui = SerialGUI()
    gui.run()
    print('Exiting Program')

