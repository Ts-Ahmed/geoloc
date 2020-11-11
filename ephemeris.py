class Ephemeris:
    def __init__(self) -> None:
        """
        Constructor.

        NB: u-blox receiver only transmits subframes 1 through 3.
        """

        self.svid = None
        self.how = None
        self.sf1 = Subframe()
        self.sf2 = Subframe()
        self.sf3 = Subframe()

    def set_data(self, raw_data) -> None:
        """
        Fills class with data received

        :param raw_data: bytes:
        """
        lenm = len(raw_data)
        lenp = raw_data[4:6]
        lenp = int.from_bytes(lenp, "little", signed=False)
        raw_data_payload = raw_data[6: lenm - 2]
        self.svid = int.from_bytes(raw_data_payload[0:4], "little", signed=False)
        raw_data_payload = raw_data_payload[4:]
        self.how = Word(int.from_bytes(raw_data_payload[0:4], "little", signed=False))
        raw_data_payload = raw_data_payload[4:]
        if lenp > 8:
            for i, attr in zip(range(8), ("word3",
                                          "word4",
                                          "word5",
                                          "word6",
                                          "word7",
                                          "word8",
                                          "word9",
                                          "word10")):
                setattr(self.sf1, attr,
                        Word(int.from_bytes(raw_data_payload[i*4:i*4+4], "little", signed=False)))
                setattr(self.sf2, attr,
                        Word(int.from_bytes(raw_data_payload[32 + i*4:32 + i*4 + 4], "little", signed=False)))
                setattr(self.sf3, attr,
                        Word(int.from_bytes(raw_data_payload[64 + i*4:64 + i*4 + 4], "little", signed=False)))
                

class Subframe:
    def __init__(self) -> None:
        """
        Constructor.

        NB: u-blox receiver only transmits words 3 through 10 in of subframes 1 through 3.
        """

        self.word3 = None
        self.word4 = None
        self.word5 = None
        self.word6 = None
        self.word7 = None
        self.word8 = None
        self.word9 = None
        self.word10 = None


class Word:
    def __init__(self, value=None) -> None:
        """
        Constructor.

        Decimal and binary (24-bit) value of the word.
        """
        self.dec = value
        self.bin = bin(value)[2:].zfill(24) if isinstance(value, int) else None
