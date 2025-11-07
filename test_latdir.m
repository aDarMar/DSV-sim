close all; clear;clc

lats = [35,20;45,34.7];
lng = [17,20.7;27,55.7];
flg = {'N','N','E','E'};

GEO = GeoClass();

cnv = GEO.LatLon2Rad([lats;lng],flg);
Rv = GEO.rhumbLine(cnv(1:2),cnv(3:4));