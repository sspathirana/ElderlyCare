from machine import ADC, Pin, I2C, UART
from time import sleep_ms, sleep, ticks_ms, ticks_diff
import sh1107
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
BROKER_IP = "192.168.11.253"
CLIENT_ID = ""  # Unique ID for your device

start_y_welcome = (oled_height - line_spacing * 2) // 2
start_y_pulse = start_y_welcome + line_spacing - 2 
start_y_group = start_y_pulse + line_spacing
start_y_ecg = start_y_pulse - 2  

sensor = ADC(26)

# Initialize I2C for OLED display
i2c_oled = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
oled = sh1107.SH1107_I2C(oled_width, oled_height, i2c_oled, addr=0x3C)

# Initialize I2C for sensors
i2c_sensors = I2C(0, scl=Pin(17), sda=Pin(16), freq=100000)

buffer = bytearray((oled_width * oled_height) // pixel)
fb = framebuf.FrameBuffer(buffer, oled_width, oled_height, framebuf.MONO_HLSB)

# ADKeyboard setup
adkey_pin = ADC(Pin(27))  # GPIO 27 for ADKeyboard
ADKEY_VALUES = {
    0: 'RIGHT',      # 0-1000
    9500: 'DOWN',    # 9400-9600
    21400: 'UP',     # 21300-21500
    32700: 'LEFT',   # 32600-32800
    47900: 'SELECT', # 47700-47900
    65000: 'NONE'    # No press (high value)
}

# GPS UART setup
gps_uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
gps_parser = None

# BMP280 Pressure Sensor Class
class BMP280:
    def __init__(self, i2c, addr=0x76):
        self.i2c = i2c
        self.addr = addr
        self.dig_T1 = 0
        self.dig_T2 = 0
        self.dig_T3 = 0
        self.dig_P1 = 0
        self.dig_P2 = 0
        self.dig_P3 = 0
        self.dig_P4 = 0
        self.dig_P5 = 0
        self.dig_P6 = 0
        self.dig_P7 = 0
        self.dig_P8 = 0
        self.dig_P9 = 0
        self.t_fine = 0
        self._read_calibration()
        self._configure()
        
    def _read_calibration(self):
        try:
            # Read temperature calibration
            cal1 = self.i2c.readfrom_mem(self.addr, 0x88, 24)
            self.dig_T1 = (cal1[1] << 8) | cal1[0]
            self.dig_T2 = (cal1[3] << 8) | cal1[2]
            if self.dig_T2 > 32767:
                self.dig_T2 -= 65536
            self.dig_T3 = (cal1[5] << 8) | cal1[4]
            if self.dig_T3 > 32767:
                self.dig_T3 -= 65536
                
            # Read pressure calibration
            self.dig_P1 = (cal1[7] << 8) | cal1[6]
            self.dig_P2 = (cal1[9] << 8) | cal1[8]
            if self.dig_P2 > 32767:
                self.dig_P2 -= 65536
            self.dig_P3 = (cal1[11] << 8) | cal1[10]
            if self.dig_P3 > 32767:
                self.dig_P3 -= 65536
            self.dig_P4 = (cal1[13] << 8) | cal1[12]
            if self.dig_P4 > 32767:
                self.dig_P4 -= 65536
            self.dig_P5 = (cal1[15] << 8) | cal1[14]
            if self.dig_P5 > 32767:
                self.dig_P5 -= 65536
            self.dig_P6 = (cal1[17] << 8) | cal1[16]
            if self.dig_P6 > 32767:
                self.dig_P6 -= 65536
            self.dig_P7 = (cal1[19] << 8) | cal1[18]
            if self.dig_P7 > 32767:
                self.dig_P7 -= 65536
            self.dig_P8 = (cal1[21] << 8) | cal1[20]
            if self.dig_P8 > 32767:
                self.dig_P8 -= 65536
            self.dig_P9 = (cal1[23] << 8) | cal1[22]
            if self.dig_P9 > 32767:
                self.dig_P9 -= 65536
            return True
        except:
            return False
    
    def _configure(self):
        # Configure BMP280
        # osrs_t x1, osrs_p x4, normal mode
        config = (0x01 << 5) | (0x03 << 2) | 0x03
        self.i2c.writeto_mem(self.addr, 0xF4, bytes([config]))
        # Set standby time to 1000ms
        self.i2c.writeto_mem(self.addr, 0xF5, bytes([0xA0]))
        
    def _read_raw(self):
        try:
            data = self.i2c.readfrom_mem(self.addr, 0xF7, 6)
            pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            return temp_raw, pres_raw
        except:
            return 0, 0
    
    def _compensate_temperature(self, raw_temp):
        var1 = ((raw_temp / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
        var2 = ((raw_temp / 131072.0) - (self.dig_T1 / 8192.0)) * ((raw_temp / 131072.0) - (self.dig_T1 / 8192.0)) * self.dig_T3
        self.t_fine = var1 + var2
        temperature = self.t_fine / 5120.0
        return temperature
    
    def _compensate_pressure(self, raw_pressure):
        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        pressure = 1048576.0 - raw_pressure
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self.dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self.dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + self.dig_P7) / 16.0
        return pressure / 100.0  # Convert to hPa
    
    def read_temperature(self):
        temp_raw, pres_raw = self._read_raw()
        if temp_raw == 0:
            return None
        return self._compensate_temperature(temp_raw)
    
    def read_pressure(self):
        temp_raw, pres_raw = self._read_raw()
        if temp_raw == 0 or pres_raw == 0:
            return None
        self._compensate_temperature(temp_raw)  # Update t_fine
        return self._compensate_pressure(pres_raw)
    
    def read_altitude(self, sea_level_pressure=1013.25):
        pressure = self.read_pressure()
        if pressure is None:
            return None
        altitude = 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903))
        return altitude

# MLX90614 Temperature Sensor Class
class MLX90614:
    def __init__(self, i2c, addr=0x5A):
        self.i2c = i2c
        self.addr = addr
        
    def read_temp(self, reg):
        try:
            data = self.i2c.readfrom_mem(self.addr, reg, 3)
            temp = (data[1] << 8) | data[0]
            temp_kelvin = temp * 0.02  # 0.02 degrees per LSB
            temp_celsius = temp_kelvin - 273.15
            return temp_celsius
        except:
            return None
    
    def read_object_temp(self):
        return self.read_temp(0x07)
    
    def read_ambient_temp(self):
        return self.read_temp(0x06)

# MPU6050 Motion Sensor Class
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        # Wake up MPU6050
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        utime.sleep(0.1)
        # Set accelerometer range to ±2g
        self.i2c.writeto_mem(self.addr, 0x1C, b'\x00')
        # Set gyro range to ±250°/s
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x00')
        
    def read_raw_data(self, addr):
        try:
            high = self.i2c.readfrom_mem(self.addr, addr, 1)[0]
            low = self.i2c.readfrom_mem(self.addr, addr + 1, 1)[0]
            value = (high << 8) | low
            if value > 32768:
                value = value - 65536
            return value
        except:
            return 0
    
    def get_accel_data(self):
        x = self.read_raw_data(0x3B)
        y = self.read_raw_data(0x3D)
        z = self.read_raw_data(0x3F)
        accel_x = x / 16384.0  # ±2g range
        accel_y = y / 16384.0
        accel_z = z / 16384.0
        return accel_x, accel_y, accel_z
    
    def get_gyro_data(self):
        x = self.read_raw_data(0x43)
        y = self.read_raw_data(0x45)
        z = self.read_raw_data(0x47)
        gyro_x = x / 131.0  # ±250°/s range
        gyro_y = y / 131.0
        gyro_z = z / 131.0
        return gyro_x, gyro_y, gyro_z
    
    def get_temp(self):
        temp_raw = self.read_raw_data(0x41)
        temp_c = (temp_raw / 340.0) + 36.53
        return temp_c
    
    def get_angle(self):
        accel_x, accel_y, accel_z = self.get_accel_data()
        angle_x = math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z)) * 180/math.pi
        angle_y = math.atan2(-accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z)) * 180/math.pi
        return angle_x, angle_y

# GPS Parser Class
class GPSParser:
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.speed = 0.0
        self.satellites = 0
        self.fix_quality = 0
        self.timestamp = ""
        self.date = ""
        self.has_fix = False
        
    def parse_gpgga(self, sentence):
        """Parse GGA sentence"""
        if not sentence.startswith('$GPGGA'):
            return False
            
        try:
            parts = sentence.split(',')
            # Time
            self.timestamp = parts[1][:6] if parts[1] else ""
            
            # Latitude
            lat_str = parts[2]
            lat_dir = parts[3]
            if lat_str and lat_dir:
                lat_deg = float(lat_str[:2])
                lat_min = float(lat_str[2:])
                self.latitude = lat_deg + (lat_min / 60.0)
                if lat_dir == 'S':
                    self.latitude = -self.latitude
            
            # Longitude
            lon_str = parts[4]
            lon_dir = parts[5]
            if lon_str and lon_dir:
                lon_deg = float(lon_str[:3])
                lon_min = float(lon_str[3:])
                self.longitude = lon_deg + (lon_min / 60.0)
                if lon_dir == 'W':
                    self.longitude = -self.longitude
            
            # Fix quality
            self.fix_quality = int(parts[6]) if parts[6] else 0
            self.satellites = int(parts[7]) if parts[7] else 0
            
            # Altitude
            self.altitude = float(parts[9]) if parts[9] else 0.0
            
            # Has fix?
            self.has_fix = (self.fix_quality > 0)
            return True
            
        except Exception as e:
            return False

    def parse_gprmc(self, sentence):
        """Parse RMC sentence"""
        if not sentence.startswith('$GPRMC'):
            return False
            
        try:
            parts = sentence.split(',')
            if len(parts) < 12:
                return False
                
            # Time and date
            self.timestamp = parts[1][:6] if parts[1] else ""
            self.date = parts[9] if parts[9] else ""
            
            # Latitude
            lat_str = parts[3]
            lat_dir = parts[4]
            if lat_str and lat_dir:
                lat_deg = float(lat_str[:2])
                lat_min = float(lat_str[2:])
                self.latitude = lat_deg + (lat_min / 60.0)
                if lat_dir == 'S':
                    self.latitude = -self.latitude
            
            # Longitude
            lon_str = parts[5]
            lon_dir = parts[6]
            if lon_str and lon_dir:
                lon_deg = float(lon_str[:3])
                lon_min = float(lon_str[3:])
                self.longitude = lon_deg + (lon_min / 60.0)
                if lon_dir == 'W':
                    self.longitude = -self.longitude
            
            # Speed (knots to km/h)
            speed_knots = float(parts[7]) if parts[7] else 0.0
            self.speed = speed_knots * 1.852  # Convert to km/h
            
            # Fix status
            self.has_fix = (parts[2] == 'A')
            return True
            
        except Exception as e:
            return False
    
    def update(self):
        """Read and parse GPS data from UART"""
        if gps_uart and gps_uart.any():
            try:
                data = gps_uart.readline()
                if data:
                    try:
                        sentence = data.decode('utf-8').strip()
                        
                        # Parse different sentence types
                        if sentence.startswith('$GPGGA'):
                            if self.parse_gpgga(sentence):
                                return True
                        elif sentence.startswith('$GPRMC'):
                            if self.parse_gprmc(sentence):
                                return True
                    except UnicodeError:
                        pass
            except Exception as e:
                pass
        return False

# ADKeyboard Class
class ADKeyboard:
    def __init__(self, adc_pin, values_map, threshold=2000):
        self.adc = adc_pin
        self.values = sorted(values_map.items())  # Sort by ADC value
        self.threshold = threshold
        self.last_key = 'NONE'
        self.debounce_time = 200  # ms for button debounce
        self.last_press_time = 0
        self.current_option = 1
        self.screen_manager = 0
        
    def read_key(self):
        raw_value = self.adc.read_u16()
        current_time = ticks_ms()
        detected_key = 'NONE'
        
        # Find which key is pressed based on closest value
        for adc_val, key in self.values:
            if abs(raw_value - adc_val) <= self.threshold:
                detected_key = key
                break
        
        # Handle key press/release
        if detected_key != 'NONE' and detected_key != self.last_key:
            # New key pressed
            if ticks_diff(current_time, self.last_press_time) > self.debounce_time:
                self.last_key = detected_key
                self.last_press_time = current_time
                return detected_key
        
        elif detected_key == 'NONE' and self.last_key != 'NONE':
            # Key was released
            self.last_key = 'NONE'
            
        return 'NONE'
    
    def handle_navigation(self):
        global InitiateMeasurement, DataCollector, ReturnController
        key = self.read_key()
        
        if self.screen_manager == 0:  # In menu mode
            if key == 'UP':
                self.current_option = (self.current_option - 2) % 6 + 1
                ScreenManager.Menu(self.current_option)
                return True
            elif key == 'DOWN':
                self.current_option = (self.current_option % 6) + 1
                ScreenManager.Menu(self.current_option)
                return True
            elif key == 'SELECT':
                self.screen_manager = self.current_option
                ScreenManager.Option(self.current_option)
                return True
        elif self.screen_manager == 1:  # In HR measurement mode
            if key == 'SELECT':
                # Start measurement in option 1
                InitiateMeasurement = True
                self.screen_manager = 11  # Special state for HR measurement
                return True
            elif key == 'LEFT':  # Use LEFT as back button
                self.screen_manager = 0
                DataCollector = False
                ReturnController = True
                return True
        elif self.screen_manager == 11:  # Active HR measurement
            if key == 'SELECT':  # Stop measurement
                DataCollector = False
                self.screen_manager = 1
                ScreenManager.Option(1)
                return True
        elif self.screen_manager == 2 or self.screen_manager == 3:  # HRV or KUBIOS mode
            if key == 'SELECT':
                # Start measurement in options 2 or 3
                InitiateMeasurement = True
                return True
            elif key == 'LEFT':  # Use LEFT as back button
                self.screen_manager = 0
                DataCollector = False
                ReturnController = True
                return True
        elif self.screen_manager == 4:  # SENSORS mode
            if key == 'LEFT':  # Use LEFT as back button
                self.screen_manager = 0
                ReturnController = True
                return True
        elif self.screen_manager == 5:  # GPS mode
            if key == 'LEFT':  # Use LEFT as back button
                self.screen_manager = 0
                ReturnController = True
                return True
        elif self.screen_manager == 6:  # ENVIRONMENT mode
            if key == 'LEFT':  # Use LEFT as back button
                self.screen_manager = 0
                ReturnController = True
                return True
        elif self.screen_manager >= 20:  # In analysis result screens
            if key == 'SELECT' or key == 'LEFT':  # Go back to menu
                self.screen_manager = 0
                ReturnController = True
                return True
        
        return False

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

# Initialize sensor objects
mlx_sensor = None
mpu_sensor = None
bmp_sensor = None
gps_parser = GPSParser()

try:
    # Try to initialize BMP280
    bmp_sensor = BMP280(i2c_sensors)
    print("BMP280 initialized")
except Exception as e:
    print(f"BMP280 not found: {e}")

try:
    # Try to initialize MLX90614
    mlx_sensor = MLX90614(i2c_sensors)
    print("MLX90614 initialized")
except Exception as e:
    print(f"MLX90614 not found: {e}")

try:
    # Try to initialize MPU6050
    mpu_sensor = MPU6050(i2c_sensors)
    print("MPU6050 initialized")
except Exception as e:
    print(f"MPU6050 not found: {e}")

# Main program
if __name__ == "__main__":
    # Connect to WLAN
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
        
        # Updated menu with more options
        menu_options = [
            ('1.MEASURE HR', 10),
            ('2.HRV ANALYSIS', 25),
            ('3.KUBIOS', 40),
            ('4.SENSORS', 55),
            ('5.GPS', 70),
            ('6.ENVIRONMENT', 85)
        ]
        
        # Display menu options with checkboxes for selection
        for i, (option_text, y_position) in enumerate(menu_options):
            # Draw checkbox icon for selected option
            if i == selected_option - 1:
                oled.fill_rect(0, y_position + 1, 5, 5, 1)
            # Display menu option text
            oled.text(option_text, 10, y_position, 1)
        oled.show()

    @staticmethod
    def Option(selected_option):
        oled.fill(0)
        oled.text('PRESS SELECT TO', 0, 5)
        if selected_option in [1, 2, 3]:
            oled.text('START-->', 0, 20)
        elif selected_option == 4:
            oled.text('VIEW SENSORS', 0, 20)
        elif selected_option == 5:
            oled.text('VIEW GPS', 0, 20)
        elif selected_option == 6:
            oled.text('VIEW ENV DATA', 0, 20)
        oled.text('PRESS LEFT TO', 0, 35)
        oled.text('GO BACK', 0, 50)
        oled.show()
    
    @staticmethod
    def ShowSensors():
        oled.fill(0)
        oled.text('SENSOR DATA', 30, 5)
        
        row = 20
        # Show temperature if available
        if mlx_sensor:
            obj_temp = mlx_sensor.read_object_temp()
            amb_temp = mlx_sensor.read_ambient_temp()
            if obj_temp:
                oled.text(f'Obj: {obj_temp:.1f}C', 0, row)
                row += 12
            if amb_temp:
                oled.text(f'Amb: {amb_temp:.1f}C', 0, row)
                row += 12
        else:
            oled.text('No Temp Sensor', 0, row)
            row += 12
        
        # Show motion data if available
        if mpu_sensor:
            try:
                accel_x, accel_y, accel_z = mpu_sensor.get_accel_data()
                oled.text(f'X:{accel_x:.1f}g', 0, row)
                oled.text(f'Y:{accel_y:.1f}g', 64, row)
                row += 12
                oled.text(f'Z:{accel_z:.1f}g', 0, row)
                row += 12
                
                angle_x, angle_y = mpu_sensor.get_angle()
                oled.text(f'AX:{angle_x:.0f}', 0, row)
                oled.text(f'AY:{angle_y:.0f}', 64, row)
            except Exception as e:
                oled.text('MPU Error', 0, row)
                row += 12
        else:
            oled.text('No MPU Sensor', 0, row)
        
        oled.show()
    
    @staticmethod
    def ShowGPS():
        global gps_parser
        oled.fill(0)
        oled.text('GPS DATA', 40, 5)
        
        # Update GPS data
        gps_parser.update()
        
        if gps_parser.has_fix:
            oled.text(f'Sats: {gps_parser.satellites}', 0, 20)
            oled.text(f'Lat: {gps_parser.latitude:.4f}', 0, 32)
            oled.text(f'Lon: {gps_parser.longitude:.4f}', 0, 44)
            oled.text(f'Alt: {gps_parser.altitude:.0f}m', 0, 56)
            oled.text(f'Spd: {gps_parser.speed:.1f}km/h', 0, 68)
        else:
            oled.text('NO GPS FIX', 20, 40)
            oled.text(f'Sats: {gps_parser.satellites}', 30, 60)
        
        oled.text('Scanning...', 20, 110)
        oled.show()
    
    @staticmethod
    def ShowEnvironment():
        oled.fill(0)
        oled.text('ENV DATA', 40, 5)
        
        row = 20
        
        # Show BMP280 data if available
        if bmp_sensor:
            try:
                temp = bmp_sensor.read_temperature()
                pressure = bmp_sensor.read_pressure()
                altitude = bmp_sensor.read_altitude()
                
                if temp is not None:
                    oled.text(f'Temp: {temp:.1f}C', 0, row)
                    row += 12
                
                if pressure is not None:
                    oled.text(f'Press: {pressure:.1f}hPa', 0, row)
                    row += 12
                
                if altitude is not None:
                    oled.text(f'Alt: {altitude:.1f}m', 0, row)
                    row += 12
            except Exception as e:
                oled.text('BMP Error', 0, row)
                row += 12
        else:
            oled.text('No BMP280', 0, row)
            row += 12
        
        # Show MLX90614 ambient temp for comparison
        if mlx_sensor:
            amb_temp = mlx_sensor.read_ambient_temp()
            if amb_temp:
                oled.text(f'IR Temp: {amb_temp:.1f}C', 0, row)
        
        oled.show()

# class MeasurementProcessor to execute MQTT and KUBIOS
class MeasurementProcessor:
    
    @staticmethod
    def DataAnalysis():
        global adkey
        
        oled.fill(0)
        oled.text('ANALYZING...', 0, 30)
        oled.show()
        
        temp = 0
        meanRR = sum(intervals) // 20
        meanHR = 60000 // meanRR
        for i in intervals:
            temp += (i - meanRR)**2
        sdnn = int(math.sqrt(temp / 19))
        temp = 0
        for i in range(19):
            temp += (intervals[i+1] - intervals[i])**2
        rmssd = int(math.sqrt(temp / 19))
        
        oled.fill(0)
        oled.text('HRV Analysis: ',10,0)
        oled.text('Mean HR: '+str(meanHR)+' bpm',0, 15)
        oled.text('Mean PPI: '+str(meanRR),0, 27)
        oled.text('RMSSD: '+str(rmssd),0, 51)
        oled.text('SDNN: '+str(sdnn),0, 39)
        oled.show()
        
        # Construct HRV data dictionary with sensor data
        measurement = {
            "mean_hr": meanHR,
            "mean_ppi": meanRR,
            "rmssd": rmssd,
            "sdnn": sdnn
        }
        
        # Add temperature data if available
        if mlx_sensor:
            obj_temp = mlx_sensor.read_object_temp()
            amb_temp = mlx_sensor.read_ambient_temp()
            if obj_temp:
                measurement["object_temp"] = round(obj_temp, 1)
            if amb_temp:
                measurement["ambient_temp"] = round(amb_temp, 1)
        
        # Add motion data if available
        if mpu_sensor:
            try:
                accel_x, accel_y, accel_z = mpu_sensor.get_accel_data()
                angle_x, angle_y = mpu_sensor.get_angle()
                measurement["acceleration"] = {
                    "x": round(accel_x, 2),
                    "y": round(accel_y, 2),
                    "z": round(accel_z, 2)
                }
                measurement["angle"] = {
                    "x": round(angle_x, 1),
                    "y": round(angle_y, 1)
                }
            except:
                pass  # Skip motion data if there's an error
        
        # Add environmental data if available
        if bmp_sensor:
            try:
                bmp_temp = bmp_sensor.read_temperature()
                pressure = bmp_sensor.read_pressure()
                altitude = bmp_sensor.read_altitude()
                
                if bmp_temp is not None:
                    measurement["bmp_temperature"] = round(bmp_temp, 1)
                if pressure is not None:
                    measurement["pressure"] = round(pressure, 1)
                if altitude is not None:
                    measurement["bmp_altitude"] = round(altitude, 1)
            except:
                pass  # Skip environmental data if there's an error

        # Convert data to JSON string
        json_message = ujson.dumps(measurement)

        # Send message to MQTT broker
        topic = "HRV Analysis"
        mqtt_client.publish(topic, json_message)
        
        # Set screen manager state for showing results
        adkey.screen_manager = 20
    
    @staticmethod
    def CloudAnalysis():  
        APIKEY = "pbZRUi49X48I56oL1Lq8y8NDjq6rPfzX3AQeNo3a" 
        CLIENT_ID = "3pjgjdmamlj759te85icf0lucv" 
        CLIENT_SECRET = "111fqsli1eo7mejcrlffbklvftcnfl4keoadrdv1o45vt9pndlef"
        
        TOKEN_URL = "https://kubioscloud.auth.eu-west-1.amazoncognito.com/oauth2/token"

        response = requests.post( 
            url = TOKEN_URL, 
            data = 'grant_type=client_credentials&client_id={}'.format(CLIENT_ID), 
            headers = {'Content-Type':'application/x-www-form-urlencoded'}, 
            auth = (CLIENT_ID, CLIENT_SECRET))
        response = response.json()
        access_token = response["access_token"]
                
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
        response = requests.post(
            url = "https://analysis.kubioscloud.com/v2/analytics/analyze", 
            headers = { 
                "Authorization": "Bearer {}".format(access_token),
                "X-Api-Key": APIKEY 
            }, 
            json = data_set)
        response = response.json()
        
        # Print out the SNS and PNS values on the OLED screen
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
        
        # Set screen manager state for showing results
        global adkey
        adkey.screen_manager = 21

# function to draw hr pulse graphic and bpm
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

# Initialize ADKeyboard
adkey = ADKeyboard(adkey_pin, ADKEY_VALUES)

# Main animation
welcome_text = "WELCOME"
group_text = "GROUP 11"
pulse_text = "PULSE WAVE"

move_text_animation(welcome_text, pulse_text, group_text, 0, oled_width+2, start_y_welcome, "left_to_right")

ecg_data = generate_ecg_data()
draw_ecg_wave(ecg_data, speed=0.005)

loading()

# Initial screen
ScreenManager.Menu(adkey.current_option)

# Global variables
InitiateMeasurement = False
DataCollector = False
ReturnController = True
DataList = []

while True:
    # Check keyboard input
    adkey.handle_navigation()
    
    if ReturnController: # check if go back from executing MeasurementProcessor to selecting mode screen
        ReturnController = False
        ScreenManager.Menu(adkey.current_option)
        
    if InitiateMeasurement: # check if SELECT has been pressed to start new measurement
        InitiateMeasurement = False
        DataCollector = True
        
        # Clear display and show measurement screen
        oled.fill(0)
        oled.text('MEASURING...', 30, 5)
        oled.text('Press SELECT', 15, 100)
        oled.text('to stop', 40, 110)
        oled.show()
        
        pulse = False
        pulseTime = [0, 0] # store timestamps of 2 consecutive pulses
        bpm = 0 # store calculated bpm
        minimum_val = 65535 # minimum_val to calculating threshold
        maximum_val = 0 # maximum_val to calculating threshold
        sampleCount = 0 # count samples to recalculate threshold
        sumppi = 0 # sum of amount of ppi to calculate mean value of ppi
        pulseCount = 0 # count number of pulses to calculate a average ppi
        intervals = []
        intervalsCount = 0
        
        for a in range(200):
            data_value = sensor.read_u16()
            sleep_ms(4)
            DataList.append(data_value)
            DataList = DataList[-5:]
            # find max and min value to display pulse line on oled
            if data_value > maximum_val:
                maximum_val = data_value
            if data_value < minimum_val:
                minimum_val = data_value
        # calculate threshold
        High_thrsld = (minimum_val + maximum_val * 3) // 4   
        Low_thrsld = (minimum_val + maximum_val) // 2      
        Range_of_values = maximum_val - minimum_val
        
    if DataCollector: # check if MeasurementProcessor is executing a measurement
        if intervalsCount < 20 or adkey.current_option == 1:
            DataList.append(sensor.read_u16())
            sleep_ms(4)                
            DataList = DataList[-5:]
            data_value = sum(DataList) // 5
            sampleCount += 1 # count number of data_value
            # find max and min value to display pulse line on oled
            if data_value > maximum_val:
                maximum_val = data_value
            if data_value < minimum_val:
                minimum_val = data_value
       
            if data_value > High_thrsld and not pulse:
                pulse = True
                dt = ticks_ms() 
                pulseTime.append(dt) # add new timestamp to a queue
                pulseTime = pulseTime[-2:] # limit the timestamp list to 2 items
                # calculate inter-pulse-interval ppi: calculate a timespan between 2 consecutive pulses
                ppi = ticks_diff(pulseTime[-1], pulseTime[-2])
                # calculate mean ppi then calculate bpm
                if 250 < ppi < 1500: # limit range of bpm from 40-240 bpm
                    sumppi += ppi
                    pulseCount += 1
                    # calculate mean ppi after pulseCount
                    if pulseCount == 1:
                        pulseCount = 0
                        avrppi = sumppi
                        bpm = 60000 // avrppi
                        if adkey.current_option != 1:  # Only store intervals for analysis modes
                            intervals.append(avrppi)
                            intervalsCount += 1
                        sumppi = 0
            if data_value < Low_thrsld and pulse: # ignore all data_value < threshold
                pulse = False
            # update data and draw pulse line to oled after every calculated mean data_value
            DisplayUpdater(data_value, minimum_val, maximum_val, bpm)
            if sampleCount > 200:
                sampleCount = 0                 
                High_thrsld = (minimum_val + maximum_val * 3) // 4  
                Low_thrsld = (minimum_val + maximum_val) // 2      
                Range_of_values = maximum_val - minimum_val
                minimum_val = 65535
                maximum_val = 0
                
            # For option 1, we don't need intervals, just continuous measurement
            if adkey.current_option == 1:
                intervalsCount = 0 # reset intervalsCount to get infinite measurement in option 1
            
        else:
            DataCollector = False
            
            if adkey.current_option == 1:
                # Show final BPM for HR measurement
                oled.fill(0)
                oled.text('FINAL HR:', 30, 40)
                oled.text(str(bpm) + ' BPM', 30, 60)
                oled.show()
                sleep(2)
                adkey.screen_manager = 1
                ScreenManager.Option(1)
            elif adkey.current_option == 2: # MQTT
                MeasurementProcessor.DataAnalysis()
            elif adkey.current_option == 3: # KUBIOS
                MeasurementProcessor.CloudAnalysis()
    
    # Handle sensor, GPS, and environment screens
    elif adkey.screen_manager == 4:  # Sensors screen
        ScreenManager.ShowSensors()
        sleep(0.5)  # Update every 0.5 seconds
    elif adkey.screen_manager == 5:  # GPS screen
        ScreenManager.ShowGPS()
        sleep(1)  # Update every second
    elif adkey.screen_manager == 6:  # Environment screen
        ScreenManager.ShowEnvironment()
        sleep(1)  # Update every second