#!/usr/bin/python3

import glob
import subprocess as sp
import signal
import sys
import atexit
from curses import *


@atexit.register
def goodbye():                                                                                                                                                            
  nocbreak()
  scr.keypad(False)
  endwin()
  echo()


def signal_handler(sig, frame):
  #return
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


files = glob.glob("*.mid")


scr = initscr()
noecho()
cbreak()
curs_set(False)

vh=7
lines,cols = scr.getmaxyx()
selected=2

win = newwin(lines-2, cols-6, 1,3)
win.keypad(True)

logo = tuple(open("tj" if sys.argv[0] else "tj2", 'r'))

loop=sp.Popen("cat /dev/midi1 > /dev/midi1", shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL)

while True:

  win.clear()

  win.addstr(3+vh, 4, "Use the attached midi keyboard to play the music box!")
  win.addstr(5+vh, 4, "Or, select a MIDI file below to play it (arrow keys / Enter)")

  for i,j in enumerate(logo):
    win.addstr(i+5, 90, j)

  win.box()

  for i,j in enumerate(files):
    win.addstr(i+9+vh, 4, j, A_REVERSE if i==selected else 0)

  k = win.getch()
  if k == KEY_UP:
    selected = (selected-1) % len(files)
  elif k == KEY_DOWN:
    selected = (selected+1) % len(files)
  elif k == KEY_ENTER or k==10 or k==13:

    # find and kill redirection from pid
    sp.call("ps -ef | awk '$3 == \"" + str(loop.pid) + "\" {print $2}' | xargs kill -9 2>&1 >/dev/null", shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL)

    mid=sp.Popen(["aplaymidi","-p","20:0",files[selected]])
    win2 = newwin(7,70,2+vh,7)
    win2.box()
    win2.addstr(2,3, "Now playing \"%s\""%files[selected])
    win2.addstr(4,3, "Press any key to stop")

    win2.getch()
    mid.kill()

    loop=sp.Popen("cat /dev/midi1 > /dev/midi1", shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL )


