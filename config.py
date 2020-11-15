def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is


def special_print(self) -> None:
    attr_list = [attr for attr in dir(self) if not attr.startswith('__') and not attr.startswith('special')]
    str_rtr = ''
    for attr in attr_list:
        str_rtr += attr + " = " + str(getattr(self, attr)) + "   "
    print(str_rtr[:-1])
