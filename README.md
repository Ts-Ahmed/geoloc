# Geoloc
The goal of this project is to provide a tool to extract and properly format data from a u-blox receiver for post-processing.
For u-blox receivers compatible with the RXM-RAW message (i.e. able to provide raw data), getUserPosition will compute the position of the receiver.
Documentation on how to compute satellite position can be found in the official [GPS ICD](https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf).
With ephemeris and pseudorange raw data, the position is calculated using [least-squares estimation](http://indico.ictp.it/event/a12180/session/21/contribution/12/material/0/0.pdf).
