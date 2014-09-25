#!/bin/bash
MEM=`free -m | head -n 2 | tail -n 1 | sed 's/Mem: *//' | sed 's/  *.*//'`
CPU=`grep "model name" /proc/cpuinfo | head -n 1 | sed 's/[^:]*: *//'`
OS=`uname -a`
BOGO=`grep bogomips /proc/cpuinfo | sed 's/.*: //'`
echo $HOSTNAME, $CPU, $MEM, $OS
