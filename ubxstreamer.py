"""
The UBX Streamer is the central piece of this app.
Its purpose is to establish a two-way communication channel, with the u-blox receiver.
The Streamer manages the serial connection via the 'connect' and 'disconnect' methods.
The Streamer manages a thread to send and receive data via the 'start_read_thread' and 'stop_read_thread' methods.
The Streamer sends data via the 'send' method and parses the data received via the '_read_thread' method (both require a
thread to be running).
"""
from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2.ubxreader import UBXReader
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube

from almanac import Almanac_Raw, Almanac_Parsed
from config import C
from ephemeris import Ephemeris_Raw, Ephemeris_Parsed
from position import get_wgs84_sat_position, xyz_to_latlongalt, XYZPosition


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
        self.almanac_raw = {i: Almanac_Raw() for i in range(32)}  # Almanac_Raw object for each GPS Satellite
        self.almanac_parsed = {i: None for i in range(32)}  # Almanac_Parsed object for each GPS Satellite
        self.ephemeris_raw = {i: Ephemeris_Raw() for i in range(32)}  # Ephemeris_Raw object for each GPS Satellite
        self.ephemeris_parsed = {i: None for i in range(32)}  # Ephemeris_Parsed object for each GPS Satellite
        self.pseudorange = {i: None for i in range(32)}  # Pseudorange for each GPS Satellite
        self.snr = {i: None for i in range(32)}  # Signal to Noise Ration for each GPS Satellite
        self.sat_position = {i: None for i in range(32)}  # Satellite position in the sky for each GPS Satellite
        self.clockBias_dist = 0  # Clock bias (as a distance in meters)
        self.receiver_time = 0  # Receiver time in seconds

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

    def send(self, data1, data2, data3=None):
        """
        Send data to serial connection.
        The number of data (i.e. UBX messages) in parameters here is arbitrary.
        There is no theoretical limit, it's up to the user to define how many messages will be sent.
        """

        for data in (data1, data2, data3):
            if data is not None:
                self._serial_object.write(data)
                sleep(1)  # Arbitrary timeout to account for delay in getting a response to requests.
                          # Could theoretically be lower

    def _read_thread(self):
        """
        THREADED PROCESS
        Reads and parses UBX message data from the stream.
        In addition to parsing messages, this method will do further processing for specific messages: UBX-NAV-CLOCK,
        UBX-RXM-RAW, UBX-AID-ALM, and UBX-AID-EPH. Theoretically, this part could be done elsewhere.
        """

        while self._reading and self._serial_object:
            if self._serial_object.in_waiting:
                try:
                    (raw_data, parsed_data) = self._ubxreader.read()
                    if parsed_data is not None:
                        print(parsed_data)
                    if parsed_data:
                        """
                        'parsed_data' is an object which takes all the fields of the UBX message 
                        (as defined in the U-blox documentation) as attributes.
                        """
                        if parsed_data.identity == "NAV-CLOCK":
                            self.clockBias_dist = parsed_data.clkB * (10 ** -9) * C  # Multiplying to account for units.
                            self.receiver_time = parsed_data.iTOW * (10 ** -3)
                            print("Receiver clock bias: ", self.clockBias_dist)
                            print("GPS System Time: ", self.receiver_time)
                            print("\n")

                        if parsed_data.identity == "RXM-RAW":
                            self.receiver_time = parsed_data.iTOW * (10 ** -3)
                            numSV = parsed_data.numSV
                            """
                            The number of attributes for RXM-RAW depends on the amount of satellites detected.
                            As such, there is a repeated block of attributes indexed from 1 to numSV 
                            (= number of satellites detected). 
                            """
                            if numSV != 0:
                                for i in range(1, numSV + 1):
                                    prMes_num = "prMes_0" + str(i) if i < 10 else "prMes_" + str(i)
                                    sv_num = "sv_0" + str(i) if i < 10 else "sv_" + str(i)
                                    snr_num = "cno_0" + str(i) if i < 10 else "cno_" + str(i)
                                    """
                                    The satellite identifier 'sv' is indexed from 1, the corresponding python
                                    dictionary is indexed from 0.
                                    """
                                    self.pseudorange[int(getattr(parsed_data, sv_num)) - 1] = \
                                        getattr(parsed_data, prMes_num)
                                    self.snr[int(getattr(parsed_data, sv_num)) - 1] = getattr(parsed_data, snr_num)

                        if parsed_data.identity == "AID-ALM":
                            self.almanac_raw[parsed_data.svid - 1].set_data(raw_data)  # Fills up the ephemeris class
                            if not self.almanac_raw[parsed_data.svid - 1].sf_empty:
                                self.almanac_parsed[parsed_data.svid - 1] = \
                                    Almanac_Parsed(self.almanac_raw[parsed_data.svid - 1])
                                self.almanac_parsed[parsed_data.svid - 1].special_print()
                                print("\n")

                        if parsed_data.identity == "AID-EPH" and parsed_data.how != 0:
                            print('svid: ', parsed_data.svid)
                            self.ephemeris_raw[parsed_data.svid - 1].set_data(raw_data)  # Fills up the ephemeris class

                            if not self.ephemeris_raw[parsed_data.svid - 1].sf_empty:
                                self.ephemeris_parsed[parsed_data.svid - 1] = \
                                    Ephemeris_Parsed(self.ephemeris_raw[parsed_data.svid - 1])
                                self.ephemeris_parsed[parsed_data.svid - 1].special_print()

                        if parsed_data.identity == "AID-EPH" and parsed_data.how != 0 and \
                                self.ephemeris_parsed[parsed_data.svid - 1] is not None:
                            X, Y, Z = get_wgs84_sat_position(self.ephemeris_parsed[parsed_data.svid - 1],
                                                             self.receiver_time,
                                                             self.pseudorange[parsed_data.svid - 1])
                            self.sat_position[parsed_data.svid - 1] = XYZPosition(X, Y, Z)
                            print("XYZ: ", (X, Y, Z))
                            Lat, Long, Alt = xyz_to_latlongalt(X, Y, Z)
                            print("LatLongAlt: ", (Lat, Long, Alt))
                            print("\n")

                except (ube.UBXStreamError, ube.UBXMessageError, ube.UBXTypeError,
                        ube.UBXParseError) as err:
                    print(f"Something went wrong {err}")
                    continue
