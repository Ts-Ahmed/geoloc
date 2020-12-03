from math import sqrt, atan, sin, cos, pi

from config import MU, OMEGA_E_DOT
from ephemeris import Ephemeris_Parsed
from gpssystime import GpsSysTime


def get_wgs84_position(eph: Ephemeris_Parsed, gps: GpsSysTime):
    m0 = eph.m0
    dn = eph.delta_n
    a = eph.sqrt_a ** 2
    ecc = eph.e
    omega = eph.omega
    omega0 = eph.omega0
    omegadot = eph.omega_dot
    cuc = eph.cuc
    cus = eph.cus
    crc = eph.crc
    crs = eph.crs
    cic = eph.cic
    cis = eph.cis
    i0 = eph.i0
    idot = eph.idot
    toe = eph.toe

    Tk = gps.time - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    Mk = m0 + ((sqrt(MU) / sqrt(a ** 3)) + dn) * Tk

    Ek = kepler_solve(Mk, ecc)

    sinVk = (sin(Ek) * sqrt(1 - ecc**2)) / (1 - ecc * cos(Ek))

    cosVk = (cos(Ek) - ecc) / (1 - ecc * cos(Ek))

    phik = arctan2(sinVk, cosVk) + omega

    delta_uk = cuc * cos(2 * phik) + cus * sin(2 * phik)
    delta_rk = crc * cos(2 * phik) + crs * sin(2 * phik)
    delta_ik = cic * cos(2 * phik) + cis * sin(2 * phik)

    Uk = phik + delta_uk
    Rk = a*(1-ecc*cos(Ek)) + delta_rk
    Ik = i0 + idot*Tk + delta_ik

    OmegaK = omega0 + (omegadot - OMEGA_E_DOT)*Tk - OMEGA_E_DOT*toe

    Xk = Rk * cos(Uk)
    Yk = Rk * sin(Uk)

    X = Xk * cos(OmegaK) - Yk * cos(Ik) * sin(OmegaK)
    Y = Xk * sin(OmegaK) + Yk * cos(Ik) * cos(OmegaK)
    Z = Yk * sin(Ik)

    return X, Y, Z


def kepler_solve(Mk, ecc):
    Ek = Mk
    while abs(Mk - Ek + ecc * sin(Ek)) > 10 ** -12:
        Ek = Ek - (Ek - ecc * sin(Ek) - Mk) / (1 - ecc * cos(Ek))
    return Ek


def arctan2(sinus, cosinus):
    if cosinus > 0:
        return atan(sinus / cosinus)
    elif sinus > 0:
        return pi - atan(sinus / (-1 * cosinus))
    elif sinus < 0:
        return (-1 * pi) - atan(sinus / (-1 * cosinus))


def xyz_to_latlongalt(X, Y, Z):

    a = 6378137
    b = 6356752.3142
    ec = 0.00669437999014
    eprimec = 0.00673949674228

    # Calcul de la Longitude
    lamb_da = arctan2(Y, X)

    # Calcul de la Latitude
    p = sqrt(X ** 2 + Y ** 2)
    tanu = (Z / p) * (a / b)
    fin_boucle = 0
    while fin_boucle == 0:
        cosu = (1 / (1 + tanu ** 2)) ** 0.5
        sinu = (1 - cosu ** 2) ** 0.5
        tanPhi = (Z + eprimec * b * sinu ** 3) / (p - ec * a * cosu ** 3)

        if abs(tanu - (b / a) * tanPhi) < 0.0000000001:
            fin_boucle = 1
        else:
            tanu = (b / a) * tanPhi

    phi = atan(a / b * tanu)

    # Calcul de l'Altitude
    N = a / sqrt(1 - ec * sin(phi) ** 2)
    h = p / cos(phi) - N

    Lat = 180 / pi * phi
    Long = 180 / pi * lamb_da
    Alt = h
    return Lat, Long, Alt
