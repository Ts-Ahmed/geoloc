from time import sleep
from pyubx2 import UBXMessage, POLL
from ubxstreamer import UBXStreamer

if __name__ == "__main__":

    # set PORT, BAUDRATE and TIMEOUT as appropriate
    PORT = 'COM11'
    BAUDRATE = 9600
    TIMEOUT = 0

    print("Instantiating UBXStreamer class...")
    ubp = UBXStreamer(PORT, BAUDRATE, TIMEOUT)
    print(f"Connecting to serial port {PORT} at {BAUDRATE} baud...")
    ubp.connect()
    print("Starting reader thread...")
    ubp.start_read_thread()

    print("\nPolling receiver...\n\n")
    msg1 = UBXMessage('AID', 'AID-EPH', POLL)
    msg2 = UBXMessage('NAV', 'NAV-TIMEGPS', POLL)
    # msg = UBXMessage('RXM', 'RXM-RAWX', POLL)
    ubp.send(msg1.serialize(), msg2.serialize())

    sleep(3)

    print("\n\nStopping reader thread...")
    ubp.stop_read_thread()
    print("Disconnecting from serial port...")
    ubp.disconnect()
    print("Done")
