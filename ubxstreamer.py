from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2.ubxreader import UBXReader
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube

from ephemeris import Ephemeris_Raw, Ephemeris_Parsed
from gpssystime import GpsSysTime
from position import get_wgs84_position


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
        self.ephemeris_raw = {i: Ephemeris_Raw() for i in range(32)}
        self.ephemeris_parsed = {i: None for i in range(32)}
        self.gps_sys_time = GpsSysTime()

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

    def send(self, data1, data2):
        """
        Send data to serial connection.
        """

        for data in (data1, data2):
            self._serial_object.write(data)
            sleep(1)

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
                    if parsed_data:
                        # print(parsed_data)
                        if hasattr(parsed_data, "svid"): print('svid: ', parsed_data.svid)
                        if hasattr(parsed_data, "svid"):
                            self.ephemeris_raw[parsed_data.svid - 1].set_data(raw_data)  # Fills up the ephemeris class

                            if not self.ephemeris_raw[parsed_data.svid - 1].sf_empty:
                                self.ephemeris_parsed[parsed_data.svid - 1] = \
                                    Ephemeris_Parsed(self.ephemeris_raw[parsed_data.svid - 1])
                                self.ephemeris_parsed[parsed_data.svid - 1].special_print()

                        if hasattr(parsed_data, "iTOW"):
                            self.gps_sys_time.set_data(raw_data)
                            print("GPS System time: ", self.gps_sys_time.time)

                        if hasattr(parsed_data, "svid") and \
                                self.ephemeris_parsed[parsed_data.svid - 1] is not None:
                            position = get_wgs84_position(self.ephemeris_parsed[parsed_data.svid - 1],
                                                          self.gps_sys_time.time)
                            print(position)

                except (ube.UBXStreamError, ube.UBXMessageError, ube.UBXTypeError,
                        ube.UBXParseError) as err:
                    print(f"Something went wrong {err}")
                    continue
