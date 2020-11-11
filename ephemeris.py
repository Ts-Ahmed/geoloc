class Ephemeris:
    def __init__(self):
        """
        Constructor.

        NB: u-blox receiver only transmits subframes 1 through 3.
        """

        self.svid = None
        self.sf1 = Subframe()
        self.sf2 = Subframe()
        self.sf3 = Subframe()

    def _set_data(self, raw_data):
        """
        Fills class with data received

        :param raw_data:
        """


class Subframe:
    def __init__(self):
        """
        Constructor.

        NB: u-blox receiver only transmits words 3 through 10 in of subframes 1 through 3.
        """

        self.word3 = Word()
        self.word4 = Word()
        self.word5 = Word()
        self.word6 = Word()
        self.word7 = Word()
        self.word8 = Word()
        self.word9 = Word()
        self.word10 = Word()


class Word:
    def __init__(self):
        """
        Constructor.
        Decimal and binary value of the word.
        """
        self.dec = None
        self.bin: str = ''
