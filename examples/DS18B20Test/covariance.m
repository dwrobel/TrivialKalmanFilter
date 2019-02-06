#!/usr/bin/octave-cli -qf
#
# Covariance calculator.
#
# Copyright (c) 2018 Damian Wrobel <dwrobel@ertelnet.rybnik.pl>
#
# THIS MATERIAL IS PROVIDED AS IS, WITH ABSOLUTELY NO WARRANTY EXPRESSED
# OR IMPLIED.  ANY USE IS AT YOUR OWN RISK.
#
# Permission is hereby granted to use or copy this program
# for any purpose,  provided the above notices are retained on all copies.
# Permission to modify the code and to distribute modified code is granted,
# provided the above notices are retained, and a notice that the code was
# modified is included with the above copyright notice.
#
#
# Usage: ./covariance.m <input-file> <0-indexed-column-number>
#
# Example:
# $ ./covariance.m DS18B20Test.csv 0
#   0.00468921411002
#

infile = argv(){1};
column = str2num(argv(){2});

printf("%.12g\n", cov(dlmread(infile, " ", [0, column, inf, column])));
