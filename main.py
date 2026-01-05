from machine import ADC, Pin, I2C
from time import sleep_ms, sleep, ticks_ms, ticks_diff
#from ssd1306 import SSD1306_I2C
import sh1107  # Use the SH1107 driver instead of SSD1306
from umqtt_simple import MQTTClient
import network
import urequests as requests 
import ujson
import mip
import math
import utime
import random
import framebuf
import time


oled_width = 128
oled_height = 128
pixel = 8
line_spacing = 20
border_width = 1

# Network Credentials 
SSID = "EWIS"
PASSWORD = "onion321"
BROKER_IP = "192.168.1.250"
CLIENT_ID = ""  # Unique ID for your device

start_y_welcome = (oled_height - line_spacing * 2) // 2
start_y_pulse = start_y_welcome + line_spacing - 2 
start_y_group = start_y_pulse + line_spacing
start_y_ecg = start_y_pulse - 2  


sensor = ADC(26)
sw0 = Pin(7, Pin.IN, Pin.PULL_UP)

# Initialize I2C and OLED display
i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
#oled = SSD1306_I2C(oled_width, oled_height, i2c)
oled = sh1107.SH1107_I2C(oled_width, oled_height, i2c, addr=0x3C)  # SH1107 128x64
oled.fill(0)
buffer = bytearray((oled_width * oled_height) // pixel)
fb = framebuf.FrameBuffer(buffer, oled_width, oled_height, framebuf.MONO_HLSB)

#class Encoder to handle rotation and button interrupts
class Encoder:   
    
    def __init__(self, pin_a, pin_b, pin_sw):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP) 
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP) 
        self.pin_sw = Pin(pin_sw, Pin.IN, Pin.PULL_UP) 
        self.option = 1
        self.ScreenManager = 0
        self.pin_a.irq(trigger = Pin.IRQ_FALLING, handler = self.rotation)
        self.pin_sw.irq(trigger = Pin.IRQ_FALLING, handler = self.button_press)
        
    
    # handler for rotary encoder rotation 
    def rotation(self, pin):    
        if self.ScreenManager == 0:
            if self.pin_b.value():   
                self.option = (self.option % 4) + 1                
                ScreenManager.Menu(self.option)
            else:            
                self.option = (self.option - 2) % 4 + 1
                ScreenManager.Menu(self.option)

                
    # handler for button press
    def button_press(self, pin):
        global DataCollector
        global sw0
        global ReturnController
        sleep_ms(20)
        if not self.pin_sw():
            if self.ScreenManager == 0:
                self.ScreenManager = self.option
                ScreenManager.Option(self.option)
                sw0.irq(trigger = Pin.IRQ_FALLING, handler = Switch_0)
            else:                    
                self.ScreenManager = 0
                sw0.irq(trigger = Pin.IRQ_FALLING, handler = None)
                DataCollector = False
                ReturnController = True

# Function to connect to WLAN
def connect_wlan():
  # Connecting to the group WLAN
  wlan = network.WLAN(network.STA_IF)
  wlan.active(True)
  wlan.connect(SSID, PASSWORD)

  # Attempt to connect once per second
  while not wlan.isconnected():
    print("Connecting... ")
    sleep(1)

  # Print the IP address of the Pico
  print("Connection successful. Pico IP:", wlan.ifconfig()[0])


def connect_mqtt():
  mqtt_client = MQTTClient(CLIENT_ID, BROKER_IP)
  mqtt_client.connect(clean_session=True)
  return mqtt_client

#Main programe
if __name__ == "__main__":
    #connect to WLAN
    connect_wlan()

    
  # Connect to MQTT
    try:
        mqtt_client = connect_mqtt()
    except Exception as e:
        print(f"Failed to connect to MQTT: {e}")
           

# Function to animate text movement on OLED display
def move_text_animation(welcome_text, group_text, pulse_text, start_x, end_x, y, direction, color=1):
    if direction == "left_to_right":
        for i in range(start_x, end_x, pixel):
            oled.fill(1)
            oled.text(welcome_text, i, y, 0)
            oled.text(group_text, i, start_y_pulse, 0)
            oled.text(pulse_text, i, start_y_group, 0)
            oled.show()
            utime.sleep_ms(300)

# Function to display loading animation on OLED
def loading():
    oled.fill(0)
    oled.text("LOADING...", 0, 30)
    oled.show()
    utime.sleep_ms(2000)

# Class for managing screens on OLED display
class ScreenManager:
    @staticmethod
    def Menu(selected_option):        
        oled.fill(0)
        
        menu_options = [
            ('1.MEASURE HR', 10),
            ('2.HRV ANALYSIS', 25),
            ('3.KUBIOS', 40),
            ('4.HISTORY', 55)
        ]
        
        # Display menu options with checkboxes for selection
        for i, (option_text, y_position) in enumerate(menu_options):
            # Draw checkbox icon for selected option
            if i == selected_option - 1:
                oled.fill_rect(0, y_position +1, 5, 5, 1)
            # Display menu option text
            oled.text(option_text, 10, y_position, 1)  # Adjusted position for option text
        oled.show()

    @staticmethod
    def Option(selected_option):
        oled.fill(0)
        oled.text('PRESS SW_2 TO...', 0, 5)
        oled.text('START-->', 0, 20)
        oled.show()

#class MeasurementProcessor to execute MQTT and KUBIOS
class MeasurementProcessor:
    
    def DataAnalysis():
        oled.fill(0)
        oled.text('ANALYZING...', 0, 30)
        oled.show()
        
        temp = 0
        oled.text('ANALYZING...', 0, 30)
        oled.show()
        
        temp = 0
        meanRR = sum(intervals) // 20
        meanHR = 60000 // meanRR
        for i in intervals:
            temp += (i - meanRR)**2
        sdnn = int(math.sqrt(temp / 19)) #int(math.sqrt(temp / (len(intervals) - 1)))
        temp = 0
        for i in range(19): # 19 = len(intervals) - 1
            temp += (intervals[i+1] - intervals[i])**2
        rmssd = int(math.sqrt(temp / 19)) #int(math.sqrt(temp / (len(intervals) - 1)))
        
        oled.fill(0)
        oled.text('HRV Analysis: ',10,0)
        oled.text('Mean HR: '+str(meanHR)+' bpm',0, 15)
        oled.text('Mean PPI: '+str(meanRR),0, 27)
        oled.text('RMSSD: '+str(rmssd),0, 51)
        oled.text('SDNN: '+str(sdnn),0, 39)
        oled.show()
        
        # Construct HRV data dictionary
        measurement = {
            "mean_hr": meanHR,
            "mean_ppi": meanRR,
            "rmssd": rmssd,
            "sdnn": sdnn
        }

        # Convert data to JSON string
        json_message = ujson.dumps(measurement)

        # Send message to MQTT broker
        topic = "HRV Analysis"
        mqtt_client.publish(topic, json_message)
    
    def CloudAnalysis():  
        APIKEY = "pbZRUi49X48I56oL1Lq8y8NDjq6rPfzX3AQeNo3a" 
        CLIENT_ID = "3pjgjdmamlj759te85icf0lucv" 
        CLIENT_SECRET = "111fqsli1eo7mejcrlffbklvftcnfl4keoadrdv1o45vt9pndlef"
        
        LOGIN_URL = "https://kubioscloud.auth.eu-west-1.amazoncognito.com/login"
        TOKEN_URL = "https://kubioscloud.auth.eu-west-1.amazoncognito.com/oauth2/token"
        REDIRECT_URI = "https://analysis.kubioscloud.com/v1/portal/login"

        response = requests.post( 
            url = TOKEN_URL, data = 'grant_type=client_credentials&client_id={}'.format(CLIENT_ID), 
            headers = {'Content-Type':'application/x-www-form-urlencoded'}, auth = (CLIENT_ID, CLIENT_SECRET)) 
        response = response.json() #Parse JSON response into a python dictionary
        access_token = response["access_token"] #Parse access token out of the response dictionary 
                
        oled.fill(0)
        oled.text('ANALYZING...', 0, 30)
        oled.show()

        data_set = {
            "type": "RRI",
            "data": intervals,
            "analysis": {
            "type": "readiness"}
            }

        # Make the readiness analysis with the given data 
        response = requests.post( url = "https://analysis.kubioscloud.com/v2/analytics/analyze", 
            headers = { "Authorization": "Bearer {}".format(access_token), 
            #use access token to access your KubiosCloud analysis session 
            "X-Api-Key": APIKEY }, 
            json = data_set) #dataset will be automatically converted to JSON by the urequests library 
        response = response.json() 
        # Print out the SNS and PNS values on the OLED screen
        #print(response)
        meanRR = int(response['analysis']['mean_rr_ms'])
        meanHR = int(response['analysis']['mean_hr_bpm'])
        sdnn = int(response['analysis']['sdnn_ms'])
        rmssd = int(response['analysis']['rmssd_ms'])
        sns_index = response['analysis']['sns_index']
        pns_index = response['analysis']['pns_index']
        oled.fill(0)
        oled.text('Kubios Results: ',15,0)
        oled.text('MEAN HR: '+str(meanHR)+' bpm',0, 9)
        oled.text('MEAN PPI: '+str(meanRR),0, 18)
        oled.text('RMSSD: '+str(rmssd),0, 27)
        oled.text('SDNN: '+str(sdnn),0, 36)
        oled.text('SNS: ' +str(sns_index),0,45)
        oled.text('PNS: ' +str(pns_index),0,54)
        oled.show()
        
#sw_2 interrpt handler to start a new measurement
def Switch_0(pin):
    global InitiateMeasurement
    sleep_ms(20)
    if not sw0():
        InitiateMeasurement = True


#function to draw hr pulse graphic and bpm
LastPositionY = 0
def DisplayUpdater(data_value, minimum_val, maximum_val, bpm):
    global LastPositionY
    oled.scroll(-1,0) 
    if data_value > maximum_val:
        data_value = maximum_val
    elif data_value < minimum_val:
        data_value = minimum_val
    NewPositionY = 64 - 32 * (data_value - minimum_val) // Range_of_values
    oled.line(125, LastPositionY, 126, NewPositionY, 1)
    LastPositionY = NewPositionY
    oled.fill_rect(0,0,128,32,0)
    oled.text('%d BPM' % bpm, 40, 5)
    oled.show()


def generate_ecg_data():
    ecg_data = []
    for _ in range(oled_width):
        ecg_data.append(random.randint(10, 20))  
    return ecg_data
 
def draw_ecg_wave(ecg_data, speed=0.2):
    scaled_ecg_data = [int(y * oled_height / max(ecg_data)) for y in ecg_data]
    fb.fill(0)
    for i in range(len(scaled_ecg_data) - 1):
        fb.line(i, scaled_ecg_data[i] - 12, i + 1, scaled_ecg_data[i + 1] - 12, 1)  
        oled.blit(fb, 0, 0)
        oled.show()
        time.sleep(speed)  
    oled.fill(0)
    oled.show()
    time.sleep(speed)
    utime.sleep_ms(500)
            
welcome_text = "WELCOME"
group_text = "GROUP 11"
pulse_text = "PULSE WAVE"

move_text_animation(welcome_text, pulse_text, group_text, 0, oled_width+2, start_y_welcome, "left_to_right")

ecg_data = generate_ecg_data()
draw_ecg_wave(ecg_data, speed=0.005)

loading()

route_val = Encoder(10,11,12)
InitiateMeasurement = False
DataCollector = False
ReturnController = True
DataList = []

while True:
    if ReturnController: #check if go back from executing MeasurementProcessor to selecting mode screen
        ReturnController = False
        ScreenManager.Menu(route_val.option)
        
    if InitiateMeasurement: #check if sw_2 has been pressed to start new measurement
        InitiateMeasurement = False
        DataCollector = True
        pulse = False
        pulseTime = [0 , 0] #store timestamps of 2 consecutive pulses
        bpm = 0 #store calculated bpm
        minimum_val = 65535 #minimum_val to calculating threshold
        maximum_val = 0 #maximum_val to calculating threshold
        sampleCount = 0 #count samples to recalculate threshold
        sumppi = 0 #sum of amount of ppi to calculate mean value of ppi
        pulseCount = 0 #count number of pulses to calculate a average ppi
        intervals = []
        intervalsCount = 0
        
        
        for a in range(200):
            data_value = sensor.read_u16()
            sleep_ms(4)
            DataList.append(data_value)
            DataList = DataList[-5:]
            #find max and min value to display pulse line on oled
            if data_value > maximum_val:
                maximum_val = data_value
            if data_value < minimum_val:
                minimum_val = data_value
        #calculate threshold
        High_thrsld = (minimum_val + maximum_val * 3) // 4   
        Low_thrsld = (minimum_val + maximum_val) // 2      
        Range_of_values = maximum_val - minimum_val
        
    if DataCollector: #check if MeasurementProcessor is executing a measurement
        if intervalsCount < 20:
            DataList.append(sensor.read_u16())
            sleep_ms(4)                
            DataList = DataList[-5:]
            data_value = sum(DataList) // 5
            sampleCount += 1 #count number of data_value
            #find max and min value to display pulse line on oled
            if data_value > maximum_val:
                maximum_val = data_value
            if data_value < minimum_val:
                minimum_val = data_value
       
            if data_value > High_thrsld and not pulse:
                pulse = True
                dt = ticks_ms() 
                pulseTime.append(dt) #add new timestamp to a queue
                pulseTime = pulseTime[-2:] #limit the timestamp list to 2 items
                #calculate inter-pulse-interval ppi: calculate a timespan between 2 consecutive pulses
                ppi = ticks_diff(pulseTime[-1], pulseTime[-2])
                #calculate mean ppi then calculate bpm
                if 250 < ppi < 1500: #limit range of bpm from 40-240 bpm
                    sumppi += ppi
                    pulseCount += 1
                    #calculate mean ppi after pulseCount
                    if pulseCount == 1: #modify this value to get better result
                        pulseCount = 0
                        avrppi = sumppi #modify this value to get better result
                        bpm = 60000 // avrppi
                        intervals.append(avrppi)
                        intervalsCount += 1
                        sumppi = 0
            if data_value < Low_thrsld and pulse: #ignore all data_value < threshold
                pulse = False
            #update data and draw pulse line to oled after every calculated mean data_value
            DisplayUpdater(data_value, minimum_val, maximum_val, bpm)
            if sampleCount > 200:
                sampleCount = 0                 
                High_thrsld = (minimum_val + maximum_val * 3) // 4  
                Low_thrsld = (minimum_val + maximum_val) // 2      
                Range_of_values = maximum_val - minimum_val
                minimum_val = 65535
                maximum_val = 0
                
            if route_val.option == 1:
                intervalsCount = 0 #reset intervalsCount to get infinite measurement in option 1
            
        else:
            DataCollector = False
            if route_val.option == 2: #MQTT
                MeasurementProcessor.DataAnalysis()
            if route_val.option == 3: #KUBIOS
                MeasurementProcessor.CloudAnalysis()

