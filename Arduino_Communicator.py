from smbus import SMBus
import struct
import traceback
import time


class ArduinoCommunicator:
    def __init__(self, arduino_address):
        self.addr = arduino_address
        self.bus = SMBus(1)
        self.write_attempt = 0
        self.data_from_arduino = dict.fromkeys(["IMU data"])
        self.UPDATE_SEND_REGISTER = 11
        
        # TODO expand this dictionary to the mapping you want
        # For the Arduino data
        self.data_to_arduino_register = {
            "c_x_p": 1,
            "c_y_p": 2
            }

        # These need to match the Arduino side commands
        self.arduino_commands = {
            "STOP": 100,
            "TURN": 101,
            "DEAD_RECKONING": 102,
            "SENT_CAMERA_DATA": 103
            }
        self.SIZE_OF_ARDUINO_SEND_REGISTERS = 8
        
    def write_string(self, word):
        # convert a string into a bytearray object in Python
        data = bytearray(word, 'utf8')
        # convert the byte array object to a list as required by write_i2c_block_data
        data = list(data)
        self._write(10, data)
    
    def write_float_to_register(self, num, register):
        """
        Writes a float into the Arduino's receive register
        """
        
        if register < 0 or register > 7:
            raise Exception("Error in write float: register must be in [0,7]")
        data = list(bytearray(str(num), 'utf8'))
        self._write(register, data)
        
    def trigger_method(self, trigger_key):
        """
        Tells the arduino to do something. Make sure you map the command appropriately
        on the arduino
        """
        if trigger_key not in self.arduino_commands.keys():
            raise Exception("Attempting to trigger non-existent command")
        else:
            data = list(bytearray(str(0), 'utf8'))
            self._write(self.arduino_commands[trigger_key], data)
        
        
            
    def _write(self, register, data):
        # private method do not call directly. tries to write for
        # a maximum of TIMEOUT tries
        TIMEOUT = 10
        try:
            self.bus.write_i2c_block_data(self.addr, register, data)
            self.write_attempt = 0
        except Exception as e:
            self.write_attempt += 1
            
            if self.write_attempt < TIMEOUT:
                print("Failed to write due to Exception " + str(e) + ". Trying again")
                #self._write(register, data)
            else:
                print("Timed out writing")
                traceback.print_exc()
                
    def get_arduino_data_from_register(self, register):
        TIMEOUT = 10
        """
        Read's data currently stored in the Arduino's send register
        """
        if register < 0 or register > 7:
            raise Exception("Error in write float: register must be in [0,7]")
        try:
            self._write(self.UPDATE_SEND_REGISTER, list(bytearray(str(register), 'utf8')))
            data = self.bus.read_i2c_block_data(self.addr,register,8)
            word = "".join([chr(i) for i in data])
            return float(word)
        except Exception as e:
            self.write_attempt += 1
            if self.write_attempt < TIMEOUT:
                print("Failed to write due to Exception " + str(e) + ". Trying again")
                self._write(register, data)
            else:
                print("Timed out writing")
                traceback.print_exc()