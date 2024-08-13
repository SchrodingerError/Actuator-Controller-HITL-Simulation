from mcculw import ul as mcculw
from mcculw.enums import (AiChanType, BoardInfo, Status, InterfaceType,
                          FunctionType, InfoType, ScanOptions,
                          ULRange, AnalogInputMode, DigitalPortType, DigitalIODirection)

import time


class MCC3114():
    def __init__(self):
        self.range = ULRange.BIP10VOLTS
        self.board_num = self.setup_device()
        if self.board_num == -1:
            print("Could not setup MCC3114")

        self.config_outputs()

        self.current_voltage_outputs: list[float] = [0.0]*16
        
        self.set_all_to_zero()

    def __del__(self):
        self.set_all_to_zero()

    def setup_device(self) -> int:
        '''Sets up the device without Instacal configuration files'''
        board_num = 0
        board_index = 0
        find_device = "USB-3114"
        board_num = -1
        dev_list = mcculw.get_daq_device_inventory(InterfaceType.USB)
        if len(dev_list) > 0:
            for device in dev_list:
                if str(device) == find_device:
                    board_num = board_index
                    mcculw.create_daq_device(board_num, device)
                board_index = board_index + 1
            if board_num == -1:
                print(f"Device {find_device} not found")
                return board_num
        else:
            print("No devices detected")
            return board_num
        return board_num
    
    def config_outputs(self):
        if self.board_num == -1:
            return
        for ch in range(16):
            mcculw.set_config(
                InfoType.BOARDINFO, self.board_num, ch,
                BoardInfo.DACRANGE, self.range
            )


    def set_all_to_zero(self):
        '''Make the voltage outputs be 0.0 V when the program exits or we delete the object'''
        if self.board_num == -1:
            return
        for i in range(16):
            self.voltage_write(channel=i, voltage=0.0)
    
    def voltage_write(self, channel:int, voltage: float):
        if self.board_num == -1:
            return
        if channel < 0 or channel > 15:
            print(f"Channel: {channel} is out of range for the MCC USB-3114. Valid range is [0, 15]")
            return
        if voltage < -10:
            print(f"Voltage: {voltage} is out of range for the MCC USB-3114 in this configuration. Valid range is [-10, 10] V")
            voltage = -10
        if voltage > 10:
            print(f"Voltage: {voltage} is out of range for the MCC USB-3114 in this configuration. Valid range is [-10, 10] V")
            voltage = 10
        try:
            mcculw.v_out(self.board_num, channel, self.range, voltage)
            self.current_voltage_outputs[channel] = voltage
        except Exception as exc:
            print(f"Exception occurred doing voltage write to MCC USB-3114: {exc}")




class MCC202():
    def __init__(self):
        self.range = ULRange.BIP10VOLTS
        self.digital_type = DigitalPortType.AUXPORT
        self.board_num = self.setup_device()

        if self.board_num == -1:
            print("Could not setup MCC202")

        self.current_analog_inputs: list[float] = [0.0] * 8
        self.current_digital_inputs: list[int] = [0] * 8

    def __del__(self):
        pass  # No specific cleanup required for inputs

    def setup_device(self) -> int:
        '''Sets up the device without Instacal configuration files'''
        board_num = 0
        board_index = 0
        find_device = "USB-202"
        board_num = -1
        dev_list = mcculw.get_daq_device_inventory(InterfaceType.USB)
        if len(dev_list) > 0:
            for device in dev_list:
                if str(device) == find_device:
                    board_num = board_index
                    mcculw.create_daq_device(board_num, device)
                board_index = board_index + 1
            if board_num == -1:
                print(f"Device {find_device} not found")
                return board_num
        else:
            print("No devices detected")
            return board_num
        return board_num
    
    def config_inputs(self):
        if self.board_num == -1:
            return
        for ch in range(8):
            mcculw.set_config(
                InfoType.BOARDINFO, self.board_num, ch,
                BoardInfo.RANGE, self.range
            )
        for ch in range(8):
            mcculw.d_config_port(self.board_num, self.digital_type, DigitalIODirection.IN)

    def print_all_channels(self):
        for i in range(8):
            print(f"Analog channel {i} reads {self.current_analog_inputs[i]} Volts")
        for i in range(8):
            print(f"Digital channel {i} reads {self.current_digital_inputs[i]}")

    def voltage_read(self, channel: int) -> float:
        if self.board_num == -1:
            return 0.0
        if channel < 0 or channel > 7:
            print(f"Channel: {channel} is out of range for the MCC USB-202. Valid range is [0, 7]")
            return 0.0
        try:
            voltage = mcculw.v_in(self.board_num, channel, self.range)
            self.current_analog_inputs[channel] = voltage
            return voltage
        except Exception as exc:
            print(f"Exception occurred doing voltage read from MCC USB-202: {exc}")
            return 0.0
    
    def read_all_analog_channels(self):
        '''Read all analog input channels and update the current_analog_inputs list'''
        for i in range(8):
            self.voltage_read(i)
        
    def digital_read(self, channel: int) -> int:
        if self.board_num == -1:
            return -1
        if channel < 0 or channel > 7:
            print(f"Digital channel: {channel} is out of range for the MCC USB-202. Valid range is [0, 7]")
            return -1
        try:
            state = mcculw.d_bit_in(self.board_num, self.digital_type, channel)
            self.current_digital_inputs[channel] = state
            return state
        except Exception as exc:
            print(f"Exception occurred doing digital read from MCC USB-202: {exc}")
            return -1
    
    def read_all_digital_channels(self):
        '''Read all digital input channels and update the current_digital_inputs list'''
        for i in range(8):
            self.digital_read(i)    
    

    def read_all_channels(self):
        for i in range(8):
            self.voltage_read(i)
            self.digital_read(i)


def main():
    mcculw.ignore_instacal() # ONLY CALL ONCE
    #mcc3114 = MCC3114()

    mcc202 = MCC202()

    while True:
        mcc202.read_all_channels()
        mcc202.print_all_channels()
        time.sleep(3)


    #mcc3114.voltage_write(1, 5)
    #mcc3114.voltage_write(7, 5)
    #time.sleep(100)
    

if __name__ == "__main__":
    main()