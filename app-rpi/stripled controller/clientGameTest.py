#!/usr/bin/env python
# coding: utf-8

import socket
import time
import sys
import tty
import termios
import random

hote = "localhost"
port = 10000

server_address = (hote, port)
def sendCommand(cmd):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print "Connection on {}".format(port)
    sent = sock.sendto(cmd, server_address)
    print("Command ", cmd, " sent")
    sock.close()
    # time.sleep(0.5)

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
        inkey = _Getch()
        while(1):
                k=inkey()
		print (str(k))
                if k!='':break
        if k=='\x1b[A':
                print "up: restart game"
                sendCommand("("+str(0)+")")
        elif k=='\x1b[B':
                print "down"
        elif k=='\x1b[C':
                print "right: player 2 move"
                sendCommand("("+str(2)+")")
        elif k=='\x1b[D':
                print "left: player 1 move"
                sendCommand("("+str(1)+")")
        elif k=='q':
                print "bye bye"
                sys.exit()
        else:
                print "not an arrow key!"


if __name__ == '__main__':
    sendCommand("(0)")
    for x in range(60):
        get()
        # sendCommand("("+str(random.randint(1,2))+")")

