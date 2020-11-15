from config import twos_comp


class GpsSysTime:
    def __init__(self):
        """
        Constructor.
        """
        self.itow = None
        self.ftow = None
        self.time = None

    def set_data(self, raw_data):
        """

        :param raw_data:
        """
        lenm = len(raw_data)
        raw_data_payload = raw_data[6: lenm - 2]
        itow = int.from_bytes(raw_data_payload[0:4], "little", signed=False)
        ftow_signed = twos_comp(int.from_bytes(raw_data_payload[4:8], "little", signed=False), 32)
        self.itow = itow
        self.ftow = ftow_signed
        self.time = itow * (10 ** -3) + ftow_signed * (10 ** -9)
