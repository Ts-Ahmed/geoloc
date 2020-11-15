from math import sqrt, atan, sin, cos

from config import MU, OMEGA_E_DOT
from ephemeris import Ephemeris_Parsed
from gpssystime import GpsSysTime


def get_wgs84_position(eph: Ephemeris_Parsed, gps_time: GpsSysTime):
    m0 = eph.m0
    dn = eph.delta_n
    a = eph.sqrt_a ** 2
    e = eph.e
    w = eph.omega
    w0 = eph.omega0
    wdot = eph.omega_dot
    cuc = eph.cuc
    cus = eph.cus
    crc = eph.crc
    crs = eph.crs
    cic = eph.cic
    cis = eph.cis
    i0 = eph.i0
    idot = eph.idot
    toe = eph.toe

    Tk = gps_time - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    Mk = m0 + ((sqrt(MU) / sqrt(a ** 3)) + dn) * Tk

    Ek = kepler_solve(Mk, e)

    Vk = atan((sqrt(1 - e**2)*sin(Ek)/(1 - e*cos(Ek)))/((cos(Ek)-e)/(1-e*cos(Ek))))

    phik = w + Vk

    du = cuc * cos(2 * phik) + cus * sin(2 * phik)
    dr = crc * cos(2 * phik) + crs * sin(2 * phik)
    di = cic * cos(2 * phik) + cis * sin(2 * phik)

    Uk = phik + du
    Rk = a*(1-e*cos(Ek)) + dr
    Ik = i0 + idot*Tk + di

    OmegaK = w0 + (wdot - OMEGA_E_DOT)*Tk - OMEGA_E_DOT*toe

    Xk = Rk * cos(Uk)
    Yk = Rk * sin(Uk)

    X = Xk * cos(OmegaK) - Yk * cos(Ik) * sin(OmegaK)
    Y = Xk * sin(OmegaK) + Yk * cos(Ik) * sin(OmegaK)
    Z = Yk * sin(Ik)

    position = (X, Y, Z)
    return position


def kepler_solve(Mk, e):
    Ek = 0
    for i in range(10):
        Ek = Mk + e * sin(Ek)
    return Ek
