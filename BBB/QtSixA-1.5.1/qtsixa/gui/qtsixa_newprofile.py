#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import os, shared
from PyQt4.QtCore import SIGNAL
from PyQt4.QtGui import QDialog, QIcon, QMessageBox
import ui_qtsixa_newprofilew


class NewProfileW(QDialog, ui_qtsixa_newprofilew.Ui_NewProfileW):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))

        self.dockWidget.setVisible(False)
        self.dockWidget.setFloating(True)

        self.connect(self.b_tips, SIGNAL('clicked()'), self.func_Tips)
        self.connect(self.b_done, SIGNAL('clicked()'), self.func_Done)
        self.connect(self.combo_left, SIGNAL('currentIndexChanged(QString)'), self.func_UpdateComboLeft)
        self.connect(self.combo_right, SIGNAL('currentIndexChanged(QString)'), self.func_UpdateComboRight)

        self.group_left_h.setEnabled(0)
        self.group_left_v.setEnabled(0)
        self.group_right_h.setEnabled(0)
        self.group_right_v.setEnabled(0)

        if (shared.editProfile != ""):
          fileC = open(os.getenv("HOME")+"/.qtsixa2/profiles/"+shared.editProfile, "r").read()
          self.s_select.setValue(int(fileC.split("key_select")[1].split()[0]))
          self.s_start.setValue(int(fileC.split("key_start")[1].split()[0]))
          self.s_ps.setValue(int(fileC.split("key_ps")[1].split()[0]))
          self.s_r2.setValue(int(fileC.split("key_r2")[1].split()[0]))
          self.s_r1.setValue(int(fileC.split("key_r1")[1].split()[0]))
          self.s_l2.setValue(int(fileC.split("key_l2")[1].split()[0]))
          self.s_l1.setValue(int(fileC.split("key_l1")[1].split()[0]))
          self.s_up.setValue(int(fileC.split("key_up")[1].split()[0]))
          self.s_right.setValue(int(fileC.split("key_right")[1].split()[0]))
          self.s_down.setValue(int(fileC.split("key_down")[1].split()[0]))
          self.s_left.setValue(int(fileC.split("key_left")[1].split()[0]))
          self.s_triangle.setValue(int(fileC.split("key_tri")[1].split()[0]))
          self.s_square.setValue(int(fileC.split("key_squ")[1].split()[0]))
          self.s_cross.setValue(int(fileC.split("key_cro")[1].split()[0]))
          self.s_circle.setValue(int(fileC.split("key_cir")[1].split()[0]))
          self.s_left_up.setValue(int(fileC.split("axis_left_up")[1].split()[0]))
          self.s_left_down.setValue(int(fileC.split("axis_left_down")[1].split()[0]))
          self.s_left_left.setValue(int(fileC.split("axis_left_left")[1].split()[0]))
          self.s_left_right.setValue(int(fileC.split("axis_left_right")[1].split()[0]))
          self.s_right_up.setValue(int(fileC.split("axis_right_up")[1].split()[0]))
          self.s_right_down.setValue(int(fileC.split("axis_right_down")[1].split()[0]))
          self.s_right_left.setValue(int(fileC.split("axis_right_left")[1].split()[0]))
          self.s_right_right.setValue(int(fileC.split("axis_right_right")[1].split()[0]))
          self.s_speed.setValue(int(fileC.split("axis_speed")[1].split()[0]))

          axis_left = int(fileC.split("axis_left_type")[1].split()[0])
          axis_right = int(fileC.split("axis_right_type")[1].split()[0])

          if (axis_left == 0):
            self.combo_left.setCurrentIndex(3)
          elif (axis_left == 2):
            self.combo_left.setCurrentIndex(2)
          elif (axis_left == 3):
            if (self.s_left_right.value() == 6):
              self.combo_left.setCurrentIndex(1)
            else:
              self.combo_left.setCurrentIndex(0)

          if (axis_right == 0):
            self.combo_right.setCurrentIndex(3)
          elif (axis_right == 2):
            self.combo_right.setCurrentIndex(2)
          elif (axis_right == 3):
            if (self.s_right_right.value() == 6):
              self.combo_right.setCurrentIndex(1)
            else:
              self.combo_right.setCurrentIndex(0)

          app_name = fileC.split("# Input - \"")[1].split("\"")[0]
          author_name = fileC.split("# Input - \"")[1].split("\"")[2]
          self.line_app.setText(app_name)
          self.line_author.setText(author_name)



    def func_Tips(self):
        QMessageBox.information(self, self.tr("QtSixA - Tips & Tricks"), self.tr(""
        "<body style=\"font-size:10pt;\">Some tips you may need:<p><body style=\"font-size:8pt;\">"
        "<b>1. </b>Click on the \"View available keys\" buttons to see a full list of available keys you can assign to a button;<br>"
        "<b>2. </b>Select the key's yellow number you want to a button spin-box;<br>"
        "<b>3. </b>If you don't want a button to work as a key set the spin-box to 0;<br>"
        "<b>4. </b>Don't forget about the 'Axis' part!<br>"
        "<b>6. </b>It's also possible to assign a mouse-button to joystick-button, check \"BTN_MOUSE\" section at the end of the key list.<br>"))

    def func_UpdateComboLeft(self):
        if (self.combo_left.currentIndex() == 2): #Custom Buttons
          self.group_left_h.setEnabled(1)
          self.group_left_v.setEnabled(1)
        else:
          self.group_left_h.setEnabled(0)
          self.group_left_v.setEnabled(0)

    def func_UpdateComboRight(self):
        if (self.combo_right.currentIndex() == 2): #Custom Buttons
          self.group_right_h.setEnabled(1)
          self.group_right_v.setEnabled(1)
        else:
          self.group_right_h.setEnabled(0)
          self.group_right_v.setEnabled(0)

    def func_Done(self):
        if (self.combo_left.currentIndex() == 0): #Mouse
          file_left_type = "3" #mouse
          file_left_left = "0"
          file_left_right = "0" #x
          file_left_up = "1" #y
          file_left_down = "0"
        elif (self.combo_left.currentIndex() == 1): #Scroll
          file_left_type = "3" #mouse
          file_left_left = "0"
          file_left_right = "6" #hwheel
          file_left_up = "8" #wheel
          file_left_down = "0"
        elif (self.combo_left.currentIndex() == 2): #Custom
          file_left_type = "2" #keys
          file_left_left = str(self.s_left_left.value())
          file_left_right = str(self.s_left_right.value())
          file_left_up = str(self.s_left_up.value())
          file_left_down = str(self.s_left_down.value())
        else:
          file_left_type = "0"
          file_left_left = "0"
          file_left_right = "0"
          file_left_up = "0"
          file_left_down = "0"

        if (self.combo_right.currentIndex() == 0): #Mouse
          file_right_type = "3" #mouse
          file_right_left = "0"
          file_right_right = "0" #x
          file_right_up = "1" #y
          file_right_down = "0"
        elif (self.combo_right.currentIndex() == 1): #Scroll
          file_right_type = "3" #mouse
          file_right_left = "0"
          file_right_right = "6" #hwheel
          file_right_up = "8" #wheel
          file_right_down = "0"
        elif (self.combo_right.currentIndex() == 2): #Custom
          file_right_type = "2" #keys
          file_right_left = str(self.s_right_left.value())
          file_right_right = str(self.s_right_right.value())
          file_right_up = str(self.s_right_up.value())
          file_right_down = str(self.s_right_down.value())
        else:
          file_right_type = "0"
          file_right_left = "0"
          file_right_right = "0"
          file_right_up = "0"
          file_right_down = "0"

        outFile = (""
                   "# Input - \""+str(self.line_app.text())+"\", by \""+str(self.line_author.text())+"\"\n"
                   "key_select "+str(self.s_select.value())+"\n"
                   "key_l3 0\n"
                   "key_r3 0\n"
                   "key_start "+str(self.s_start.value())+"\n"
                   "key_up "+str(self.s_up.value())+"\n"
                   "key_right "+str(self.s_right.value())+"\n"
                   "key_down "+str(self.s_down.value())+"\n"
                   "key_left "+str(self.s_left.value())+"\n"
                   "key_l2 "+str(self.s_l2.value())+"\n"
                   "key_r2 "+str(self.s_r2.value())+"\n"
                   "key_l1 "+str(self.s_l1.value())+"\n"
                   "key_r1 "+str(self.s_r1.value())+"\n"
                   "key_tri "+str(self.s_triangle.value())+"\n"
                   "key_cir "+str(self.s_circle.value())+"\n"
                   "key_squ "+str(self.s_square.value())+"\n"
                   "key_cro "+str(self.s_cross.value())+"\n"
                   "key_ps "+str(self.s_ps.value())+"\n"
                   "axis_left_type "+file_left_type+"\n"
                   "axis_left_up "+file_left_up+"\n"
                   "axis_left_right "+file_left_right+"\n"
                   "axis_left_down "+file_left_down+"\n"
                   "axis_left_left "+file_left_left+"\n"
                   "axis_right_type "+file_right_type+"\n"
                   "axis_right_up "+file_right_up+"\n"
                   "axis_right_right "+file_right_right+"\n"
                   "axis_right_down "+file_right_down+"\n"
                   "axis_right_left "+file_right_left+"\n"
                   "axis_speed "+str(self.s_speed.value())+"\n")

        if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/profiles/"):
          os.mkdir(os.getenv("HOME")+"/.qtsixa2/profiles/")
        fileW = open(os.getenv("HOME")+"/.qtsixa2/profiles/"+str(self.line_app.text()), "w")
        fileW.write(outFile)
        fileW.close()

        QMessageBox.information(self, self.tr("QtSixA - Done!"), self.tr("It's done!\nA new profile has been saved.\n \nFeel free to quit now"))


