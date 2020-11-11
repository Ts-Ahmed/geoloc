from sys import platform
from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2 import UBXMessage, POLL, UBX_CONFIG_MESSAGES, SET, GET
from pyubx2.ubxreader import UBXReader
from pyubx2.exceptions import UBXStreamError
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube
import pyubx2.ubxtypes_core as ubt


class UBXStreamer:
    """
    UBXStreamer class.
    """

    def __init__(self, port, baudrate, timeout=5):
        """
        Constructor.
        """

        self._serial_object = None
        self._serial_thread = None
        self._ubxreader = None
        self._connected = False
        self._reading = False
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout

    def __del__(self):
        """
        Destructor.
        """

        self.stop_read_thread()
        self.disconnect()

    def connect(self):
        """
        Open serial connection.
        """

        try:
            self._serial_object = Serial(self._port,
                                         self._baudrate,
                                         timeout=self._timeout)
            self._ubxreader = UBXReader(BufferedReader(self._serial_object), False)
            self._connected = True
        except (SerialException, SerialTimeoutException) as err:
            print(f"Error connecting to serial port {err}")

    def disconnect(self):
        """
        Close serial connection.
        """

        if self._connected and self._serial_object:
            try:
                self._serial_object.close()
            except (SerialException, SerialTimeoutException) as err:
                print(f"Error disconnecting from serial port {err}")
        self._connected = False

    def start_read_thread(self):
        """
        Start the serial reader thread.
        """

        if self._connected:
            self._reading = True
            self._serial_thread = Thread(target=self._read_thread, daemon=False)
            self._serial_thread.start()

    def stop_read_thread(self):
        """
        Stop the serial reader thread.
        """

        if self._serial_thread is not None:
            self._reading = False
            self._serial_thread.join()

    def send(self, data):
        """
        Send data to serial connection.
        """

        self._serial_object.write(data)

    def flush(self):
        """
        Flush input buffer
        """

        self._serial_object.reset_input_buffer()

    def waiting(self):
        """
        Check if any messages remaining in the input buffer
        """

        return self._serial_object.in_waiting

    def _read_thread(self):
        """
        THREADED PROCESS
        Reads and parses UBX message data from stream
        """

        while self._reading and self._serial_object:
            if self._serial_object.in_waiting:
                try:
                    (raw_data, parsed_data) = self._ubxreader.read()
#                     if raw_data:
#                         print(raw_data)
                    if parsed_data:
                        print(parsed_data)
                        # print(raw_data)
                        print(raw_data.hex(sep=' ', bytes_per_sep=2))
                except (ube.UBXStreamError, ube.UBXMessageError, ube.UBXTypeError,
                        ube.UBXParseError) as err:
                    print(f"Something went wrong {err}")
                    continue


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
    msg = UBXMessage('AID', 'AID-EPH', POLL)
    # msg = UBXMessage('RXM', 'RXM-RAWX', POLL)
    ubp.send(msg.serialize())

    sleep(3)

    print("\n\nStopping reader thread...")
    ubp.stop_read_thread()
    print("Disconnecting from serial port...")
    ubp.disconnect()
    print("Done")