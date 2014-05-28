#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import os, sys
from commands import getoutput
from PyQt4.QtCore import QT_VERSION_STR, PYQT_VERSION_STR
from PyQt4.QtGui import QApplication, QMessageBox

import shared, qtsixa_main, qtsixa_manage


#--------------- main ------------------
if __name__ == '__main__':

  #For easy debugging, print version information to terminal
  print "Qt version:", QT_VERSION_STR
  print "PyQt version:", PYQT_VERSION_STR
  print "QtSixA version: 1.5.1"

  #Check for root tool
  shared.ROOT = getoutput("qtsixa "+"--get-root")
  print "Will use '"+shared.ROOT.split()[0]+"' for root actions"

  shared.init_config(None)
  shared.editProfile = ""

  shared.app = QApplication(sys.argv)
  if (shared.Globals.close_to_tray): shared.app.setQuitOnLastWindowClosed(False)
  gui = qtsixa_main.MainW()
  #gui = qtsixa_manage.ManageW()
  if (shared.Globals.start_minimized and shared.Globals.systray_enabled):
      gui.hide()
  else:
      gui.show()
  gui.show()
  shared.app.exec_()

