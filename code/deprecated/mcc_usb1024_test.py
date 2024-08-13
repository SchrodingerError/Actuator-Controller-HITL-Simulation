from mcculw import ul
from mcculw.enums import DigitalIODirection
from mcculw.ul import ULError

# Define the board number
BOARD_NUM = 0

# Define the ports and the direction of data flow
PORT_A = 10
PORT_B = 11
PORT_C_LOW = 12
PORT_C_HIGH = 13

# Configure ports as input or output
def configure_ports():
    try:
        ul.d_config_port(BOARD_NUM, PORT_A, DigitalIODirection.OUT)
        ul.d_config_port(BOARD_NUM, PORT_B, DigitalIODirection.IN)
        ul.d_config_port(BOARD_NUM, PORT_C_LOW, DigitalIODirection.OUT)
        ul.d_config_port(BOARD_NUM, PORT_C_HIGH, DigitalIODirection.IN)
        print("Ports configured successfully.")
    except ULError as e:
        print(f"Error configuring ports: {e}")

# Write data to a port
def write_data(port, data):
    try:
        ul.d_out(BOARD_NUM, port, data)
        print(f"Data {data} written to port {port} successfully.")
    except ULError as e:
        print(f"Error writing data to port {port}: {e}")

# Read data from a port
def read_data(port):
    try:
        data = ul.d_in(BOARD_NUM, port)
        print(f"Data read from port {port}: {data}")
        return data
    except ULError as e:
        print(f"Error reading data from port {port}: {e}")
        return None

def main():
    # Configure the ports
    configure_ports()
    
    # Write some data to PORT_A and PORT_C_LOW
    write_data(PORT_A, 0xFF)  # Write 0xFF (255) to PORT_A
    write_data(PORT_C_LOW, 0xAA)  # Write 0xAA (170) to PORT_C_LOW
    
    # Read data from PORT_B and PORT_C_HIGH
    data_b = read_data(PORT_B)
    data_c_high = read_data(PORT_C_HIGH)
    
    # Perform additional processing as needed
    # For example, you might want to perform some logic based on the input data
    
if __name__ == "__main__":
    main()
