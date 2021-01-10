MU = 3.986005 * (10 ** 14)
OMEGA_E_DOT = 7.292115 * (10 ** -5)
# set PORT, BAUDRATE and TIMEOUT as appropriate
PORT = 'COM11'
BAUDRATE = 9600
TIMEOUT = 0


def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
