from time import sleep
from pyubx2 import UBXMessage, POLL
from ubxstreamer import UBXStreamer

if __name__ == "__main__":

    # set PORT, BAUDRATE and TIMEOUT as appropriate
    PORT = 'COM5'
    BAUDRATE = 9600
    TIMEOUT = 0

    print("Instantiating UBXStreamer class...")
    ubxs = UBXStreamer(PORT, BAUDRATE, TIMEOUT)
    print(f"Connecting to serial port {PORT} at {BAUDRATE} baud...")
    ubxs.connect()
    print("Starting reader thread...")
    ubxs.start_read_thread()

    print("\nPolling receiver...\n\n")
    msg1 = UBXMessage('NAV', 'NAV-TIMEGPS', POLL)
    msg2 = UBXMessage('AID', 'AID-EPH', POLL)
    # msg = UBXMessage('RXM', 'RXM-RAWX', POLL)
    ubxs.send(msg1.serialize(), msg2.serialize())

    sleep(3)

    print("\n\nStopping reader thread...")
    ubxs.stop_read_thread()
    print("Disconnecting from serial port...")
    ubxs.disconnect()
    print("Done")
