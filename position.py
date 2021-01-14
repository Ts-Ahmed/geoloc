from math import sqrt, atan, sin, cos, pi

from config import MU, OMEGA_E_DOT, C, REF_X, REF_Y, REF_Z
from ephemeris import Ephemeris_Parsed

import numpy as np


class XYZPosition:
    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z


def get_wgs84_sat_position(eph: Ephemeris_Parsed, time):
    af0 = eph.af0
    af1 = eph.af1
    af2 = eph.af2
    m0 = eph.m0
    delta_n = eph.delta_n
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
    toc = eph.toc

    t_sv = time

    # Initializing this value at 0 because Ek hasn't been calculated yet
    delta_tr = 0
    # Code phase offset
    delta_t_sv = af0 + af1 * (t_sv - toc) + af2 * (t_sv - toc) ** 2 + delta_tr
    t = t_sv - delta_t_sv
    # t = t - tpdist
    Tk = t - toe
    # Tk = gps.time - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    n0 = sqrt(MU / (a ** 3))

    N = n0 + delta_n

    Mk = m0 + N * Tk

    Ek = kepler_solve(Mk, ecc)
    # Relativistic correction term
    delta_tr = (-4.442807633 ** -10) * ecc * sqrt(a) * sin(Ek)

    # Recalcul de Tk
    delta_t_sv = af0 + af1 * (t_sv - toc) + af2 * ((t_sv - toc) ** 2) + delta_tr
    t = t_sv - delta_t_sv
    # t = t - tpdist
    Tk = t - toe
    # Tk = gps.time - toe
    if Tk > 302400:
        Tk -= 604800
    elif Tk < -302400:
        Tk += 604800

    sinVk = (sin(Ek) * sqrt(1 - ecc ** 2)) / (1 - ecc * cos(Ek))

    cosVk = (cos(Ek) - ecc) / (1 - ecc * cos(Ek))

    phik = arctan2(sinVk, cosVk) + omega

    delta_uk = cuc * cos(2 * phik) + cus * sin(2 * phik)
    delta_rk = crc * cos(2 * phik) + crs * sin(2 * phik)
    delta_ik = cic * cos(2 * phik) + cis * sin(2 * phik)

    Uk = phik + delta_uk
    Rk = a * (1 - ecc * cos(Ek)) + delta_rk
    Ik = i0 + idot * Tk + delta_ik

    OmegaK = omega0 + (omegadot - OMEGA_E_DOT) * Tk - OMEGA_E_DOT * toe

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


def get_delta_tr():
    pass


def xyz_to_latlongalt(X, Y, Z):
    a = 6378137
    b = 6356752.3142
    ec = 0.00669437999014
    eprimec = 0.00673949674228

    # Longitude
    lamb_da = arctan2(Y, X)

    # Latitude
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

    # Altitude
    N = a / sqrt(1 - ec * sin(phi) ** 2)
    h = p / cos(phi) - N

    Lat = 180 / pi * phi
    Long = 180 / pi * lamb_da
    Alt = h
    return Lat, Long, Alt


def get_receiver_position(eph, pseudorange, satPosition, lli, clockBias, rTime):
    def update_h_matrix(Hrow, sat_position: XYZPosition, receiver_position):
        distance = sqrt((sat_position.X - receiver_position[0]) ** 2 +
                        (sat_position.Y - receiver_position[1]) ** 2 +
                        (sat_position.Z - receiver_position[2]) ** 2)
        Hrow[0] = (sat_position.X - receiver_position[0]) / distance
        Hrow[1] = (sat_position.Y - receiver_position[1]) / distance
        Hrow[2] = (sat_position.Z - receiver_position[2]) / distance

    pr_sv_available = list(k for k, v in pseudorange.items() if v is not None)
    eph_sv_available = list(k for k, v in eph.items() if v is not None)
    lli = list(k for k, v in lli.items() if v is not None and v > 0)

    sv_list = set(pr_sv_available).intersection(eph_sv_available)
    sv_list = list(sv_list.intersection(lli))
    print(sv_list)
    if len(sv_list) < 4:
        print("Not enough satellite with strong signal")
        return

    pr_measured = [pseudorange[x] for x in sv_list]
    pr_over_c = [x / C for x in pr_measured]
    approx_sv_clock_error = [eph[x].af0 + eph[x].af1 * (rTime - eph[x].toc) + eph[x].af2 * (rTime - eph[x].toc) ** 2
                             for x in sv_list]
    transmit_time = [rTime - x - y for x, y in zip(pr_over_c, approx_sv_clock_error)]

    sv_ecef_x = [satPosition[x].X for x in sv_list]
    sv_ecef_y = [satPosition[x].Y for x in sv_list]
    sv_ecef_z = [satPosition[x].Z for x in sv_list]
    get_delta_tr()

    pr_measured_corrected = [x +
                             C*(eph[y].af0 + eph[y].af1 * (rTime - eph[y].toc) + eph[y].af2 * (rTime - eph[y].toc) ** 2)
                             for x, y in zip(pr_measured, sv_list)]

    last_receiver_position = np.array([REF_X, REF_Y, REF_Z, clockBias])

    H = -np.ones((len(sv_list), 4))
    delta_x = np.array([42])
    while np.linalg.norm(delta_x[:3]) > 10**-6:
        for i in range(len(sv_list)):
            update_h_matrix(H[i], satPosition[sv_list[i]], last_receiver_position)

        pr_calculated = [sqrt((satPosition[sv_id].X - last_receiver_position[0]) ** 2
                              + (satPosition[sv_id].Y - last_receiver_position[1]) ** 2
                              + (satPosition[sv_id].Z - last_receiver_position[2]) ** 2)
                         + clockBias for sv_id in sv_list]
        delta_rho = np.array([x - y for x,y in zip(pr_calculated, pr_measured_corrected)])

        delta_x = np.dot(np.dot(np.linalg.inv(np.dot(H.transpose(), H)), H.transpose()), delta_rho)
        last_receiver_position = last_receiver_position + delta_x
        Lat, Long, Alt = \
            xyz_to_latlongalt(last_receiver_position[0], last_receiver_position[1], last_receiver_position[2])
        print("Longitude: ", Long)
        print("Latitude: ", Lat)
        print("Altitude: ", Alt)
        print("\n")
