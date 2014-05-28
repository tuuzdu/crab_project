#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import os, shared
from PyQt4.QtCore import SIGNAL
from PyQt4.QtGui import QDialog, QIcon, QTableWidgetItem
import ui_qtsixa_managew, qtsixa_newdev, qtsixa_newprofile


class ManageW(QDialog, ui_qtsixa_managew.Ui_ManageW):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))

        self.b_edit.setEnabled(False)
        self.b_remove.setEnabled(False)

        self.listDev.setColumnWidth(0, 150)

        self.connect(self.listDev, SIGNAL('itemSelectionChanged()'), self.func_changedListDev)
        self.connect(self.listProf, SIGNAL('currentRowChanged(int)'), self.func_changedListProf)
        self.connect(self.tabWidget, SIGNAL('currentChanged(int)'), self.func_changedTab)

        self.connect(self.b_add, SIGNAL('clicked()'), self.func_Add)
        self.connect(self.b_remove, SIGNAL('clicked()'), self.func_Remove)
        self.connect(self.b_edit, SIGNAL('clicked()'), self.func_Edit)

        if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/profiles/"):
          os.mkdir(os.getenv("HOME")+"/.qtsixa2/profiles/")

        if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/pics/"):
          os.mkdir(os.getenv("HOME")+"/.qtsixa2/pics/")

        if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/.setup_profiles"):
          os.system("cp /usr/share/qtsixa/profiles/* "+os.getenv("HOME")+"/.qtsixa2/profiles/")
          if os.path.exists(os.getenv("HOME")+"/.qtsixa2/profiles/KDE"):
            os.mknod(os.getenv("HOME")+"/.qtsixa2/.setup_profiles")

        if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/.setup_pics"):
          os.system("cp /usr/share/qtsixa/pics/* "+os.getenv("HOME")+"/.qtsixa2/pics/")
          if os.path.exists(os.getenv("HOME")+"/.qtsixa2/pics/KDE.png"):
            os.mknod(os.getenv("HOME")+"/.qtsixa2/.setup_pics")

        self.func_refreshList()


    def func_refreshList(self):
        self.listDev.setCurrentCell(-1, -1)
        self.listDev.clearContents()
        for i in range(self.listDev.rowCount()):
          self.listDev.removeRow(0)

        pos_h = 0

        if not os.path.exists("/var/lib/sixad/profiles/"):
         os.mkdir("/var/lib/sixad/profiles/")

        if not os.path.exists("/var/lib/sixad/profiles/default"):
          dev = shared.func_checkDeviceOptions("default")
          self.listDev.insertRow(pos_h)
          self.listDev.setItem(pos_h, 0, QTableWidgetItem("default"))
          self.listDev.setItem(pos_h, 1, QTableWidgetItem(dev[0]))
          self.listDev.setItem(pos_h, 2, QTableWidgetItem(dev[1]))
          self.listDev.setItem(pos_h, 3, QTableWidgetItem(dev[2]))
          pos_h += 1

        if not os.path.exists("/var/lib/sixad/profiles/hidraw"):
          dev = shared.func_checkDeviceOptions("hidraw")
          self.listDev.insertRow(pos_h)
          self.listDev.setItem(pos_h, 0, QTableWidgetItem("hidraw"))
          self.listDev.setItem(pos_h, 1, QTableWidgetItem(dev[0]))
          self.listDev.setItem(pos_h, 2, QTableWidgetItem(dev[1]))
          self.listDev.setItem(pos_h, 3, QTableWidgetItem(dev[2]))
          pos_h += 1

        devs = os.listdir("/var/lib/sixad/profiles/")
        for i in range(len(devs)):
            pos = i + pos_h
            dev = shared.func_checkDeviceOptions(devs[i])
            self.listDev.insertRow(pos)
            self.listDev.setItem(pos, 0, QTableWidgetItem(devs[i]))
            self.listDev.setItem(pos, 1, QTableWidgetItem(dev[0]))
            self.listDev.setItem(pos, 2, QTableWidgetItem(dev[1]))
            self.listDev.setItem(pos, 3, QTableWidgetItem(dev[2]))

        self.listProf.clear()
        files = os.listdir(os.getenv("HOME")+"/.qtsixa2/profiles/")
        for f in range(len(files)):
          self.listProf.addItem(files[f])
        self.listProf.sortItems()


    def func_changedListDev(self):
        index = self.listDev.currentRow()
        if (index < 0):
          self.b_edit.setEnabled(False)
          self.b_remove.setEnabled(False)
        elif (index < 2):
          self.b_edit.setEnabled(True)
          self.b_remove.setEnabled(False)
        else:
          self.b_edit.setEnabled(True)
          self.b_remove.setEnabled(True)

    def func_changedListProf(self, index):
        if (index < 0):
          self.b_edit.setEnabled(False)
          self.b_remove.setEnabled(False)
        else:
          self.b_edit.setEnabled(True)
          self.b_remove.setEnabled(True)

    def func_changedTab(self, index):
        self.listDev.setCurrentCell(-1, -1)
        self.listProf.setCurrentRow(-1)


    def func_Add(self):
        page = self.tabWidget.currentIndex()
        if (page == 0):
          shared.editDev = ""
          qtsixa_newdev.NewDevW().exec_()
        elif (page == 1):
          shared.editProfile = ""
          qtsixa_newprofile.NewProfileW().exec_()
        self.func_refreshList()

    def func_Remove(self):
        page = self.tabWidget.currentIndex()
        if (page == 0):
          os.remove("/var/lib/sixad/profiles/"+str(self.listDev.item(self.listDev.currentRow(), 0).text()))
        elif (page == 1):
          os.remove(os.getenv("HOME")+"/.qtsixa2/profiles/"+str(self.listProf.currentItem().text()))
        self.func_refreshList()

    def func_Edit(self):
        page = self.tabWidget.currentIndex()
        if (page == 0):
          shared.editDev = str(self.listDev.item(self.listDev.currentRow(), 0).text())
          qtsixa_newdev.NewDevW().exec_()
        elif (page == 1):
          shared.editProfile = str(self.listProf.currentItem().text())
          qtsixa_newprofile.NewProfileW().exec_()
        self.func_refreshList()

