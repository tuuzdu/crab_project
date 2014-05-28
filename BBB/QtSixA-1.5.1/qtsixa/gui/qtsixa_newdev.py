#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import os
from commands import getoutput
from PyQt4.QtCore import SIGNAL
from PyQt4.QtGui import QWizard, QIcon, QPixmap
import ui_qtsixa_newdevw, shared


class NewDevW(QWizard, ui_qtsixa_newdevw.Ui_NewDevW):
    def __init__(self, *args):
        QWizard.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))

        self.connect(self.co_input, SIGNAL('currentIndexChanged(QString)'), self.func_UpdatePreview)
        self.connect(self, SIGNAL('accepted()'), self.func_Done)
        self.connect(self, SIGNAL('currentIdChanged(int)'), self.func_ChangedPage)

        self.l_preview.setPixmap(QPixmap("/usr/share/qtsixa/pics/(None).png"))
        self.lastpage = -1

        self.warn_disablejoy_ico.setVisible(False)
        self.warn_disablejoy_txt.setVisible(False)

        if (shared.editDev == "hidraw"):
          self.ch_led.setChecked(False)
          self.ch_led.setEnabled(False)
          self.ch_rumble.setChecked(False)
          self.ch_rumble.setEnabled(False)
          self.ch_timeout.setChecked(False)
          self.ch_timeout.setEnabled(False)

        profs = os.listdir(os.getenv("HOME")+"/.qtsixa2/profiles/")
        profList = []
        for f in range(len(profs)):
          profList.append(profs[f])
        profList.sort()

        for g in range(len(profList)):
          self.co_input.addItem(profList[g])

        self.hidd_number_1 = ""
        self.hidd_number_2 = ""
        self.hidd_number_3 = ""
        self.hidd_number_4 = ""
        self.hidd_number_5 = ""
        self.hidd_number_6 = ""
        self.hidd_number_7 = ""
        self.hidd_number_8 = ""
        self.nOfDevices = 0

        self.Check4BluetoothDevices = getoutput("hcitool con")
        self.nOfDevices_str = getoutput("echo '"+self.Check4BluetoothDevices+"' | grep ACL -n | tail -n 1 | awk '{printf$1}' | awk 'sub(\":\",\"\")'")
        if self.nOfDevices_str != "": self.nOfDevices = int(self.nOfDevices_str) - 1
        if self.nOfDevices > 0: self.hidd_number_1 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 2 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 1: self.hidd_number_2 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 3 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 2: self.hidd_number_3 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 4 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 3: self.hidd_number_4 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 5 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 4: self.hidd_number_5 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 6 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 5: self.hidd_number_6 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 7 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 6: self.hidd_number_7 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 8 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 7: self.hidd_number_8 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 9 | tail -n 1 | awk '{printf$3}'")

        if (self.hidd_number_1 != ""):
            if (self.hidd_number_1 != "" and getoutput("hcitool name "+self.hidd_number_1+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_1)
            if (self.hidd_number_2 != "" and getoutput("hcitool name "+self.hidd_number_2+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_2)
            if (self.hidd_number_3 != "" and getoutput("hcitool name "+self.hidd_number_3+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_3)
            if (self.hidd_number_4 != "" and getoutput("hcitool name "+self.hidd_number_4+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_4)
            if (self.hidd_number_5 != "" and getoutput("hcitool name "+self.hidd_number_5+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_5)
            if (self.hidd_number_6 != "" and getoutput("hcitool name "+self.hidd_number_6+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_6)
            if (self.hidd_number_7 != "" and getoutput("hcitool name "+self.hidd_number_7+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_7)
            if (self.hidd_number_8 != "" and getoutput("hcitool name "+self.hidd_number_8+" | grep 'PLAYSTATION(R)3 Controller'") ): self.co_dev.addItem(self.hidd_number_8)

        if (shared.editDev != ""):
          self.setStartId(1)
          if (shared.editDev == "default" and not os.path.exists("/var/lib/sixad/profiles/default")):
            pass
          elif (shared.editDev == "hidraw" and not os.path.exists("/var/lib/sixad/profiles/hidraw")):
            pass
          else:
            dev = shared.editDev
            self.co_dev.addItem(dev)
            self.co_dev.setCurrentIndex(self.co_dev.count())
            if (os.path.exists("/var/lib/sixad/profiles/"+dev)):
              #Get values from config file (only change the default ones)
              if (not self.checkDevOpt(dev, "enable_leds")): self.ch_led.setChecked(False)
              if (not self.checkDevOpt(dev, "enable_joystick")): self.ch_js.setChecked(False)
              if (self.checkDevOpt(dev, "enable_input")): self.ch_input.setChecked(True)
              if (not self.checkDevOpt(dev, "enable_rumble")): self.ch_rumble.setChecked(False)
              if (not self.checkDevOpt(dev, "enable_timeout")): self.ch_timeout.setChecked(False)

              if (not self.checkDevOpt(dev, "led_anim")): self.ch_led_anim.setChecked(False)
              if (not self.checkDevOpt(dev, "led_n_auto")): self.r_manual_led.setChecked(True)
              self.r_led_n.setValue(self.checkDevOpt(dev, "led_n_number"))

              if (self.checkDevOpt(dev, "old_rumble_mode")): self.r_rumble_old.setChecked(True)

              if (self.checkDevOpt(dev, "enable_timeout")): self.ch_timeout.setChecked(True)
              self.r_timeout.setValue(self.checkDevOpt(dev, "timeout_mins"))

              if (not self.checkDevOpt(dev, "enable_buttons")): self.js_button.setChecked(False)
              if (not self.checkDevOpt(dev, "enable_sbuttons")): self.js_sbutton.setChecked(False)
              if (not self.checkDevOpt(dev, "enable_axis")): self.js_axis.setChecked(False)
              if (not self.checkDevOpt(dev, "enable_accel")): self.js_accel.setChecked(False)
              if (self.checkDevOpt(dev, "enable_accon")): self.js_accon.setChecked(True)
              if (self.checkDevOpt(dev, "enable_speed")): self.js_speed.setChecked(True)
              if (self.checkDevOpt(dev, "enable_pos")): self.js_pos.setChecked(True)

              app_name = open("/var/lib/sixad/profiles/"+dev, "r").read().split("# Input - \"")[1].split("\"")[0]
              for i in range(self.co_input.count()):
                if (app_name == str(self.co_input.itemText(i))):
                  self.co_input.setCurrentIndex(i)
                  break

            else: #defaut values
              pass

    def checkDevOpt(self, dev, opt):
        r = open("/var/lib/sixad/profiles/"+dev, "r").read()
        return int(r.split(opt)[1].split()[0])

    def func_UpdatePreview(self, text):
        pixmap = os.getenv("HOME")+"/.qtsixa2/pics/"+text+".png"
        if (os.path.exists(pixmap)):
          self.l_preview.setPixmap(QPixmap(pixmap))
        else:
          self.l_preview.setPixmap(QPixmap("/usr/share/qtsixa/pics/(NOT).png"))
        if (text == self.tr("Fake Joystick") or
	    text == self.tr("Fake Joystick 2") or
	    text == self.tr("Final Fantasy VIII") or
	    text == self.tr("Super Maryo Chronicles") or
	    text == self.tr("UltraStar Deluxe") or
	    text == self.tr("Wormux")):
	  self.warn_disablejoy_ico.setVisible(True)
	  self.warn_disablejoy_txt.setVisible(True)
	else:
	  self.warn_disablejoy_ico.setVisible(False)
	  self.warn_disablejoy_txt.setVisible(False)

    def func_ChangedPage(self, page):
        if (page == 2): #joystick
          if (not self.ch_js.isChecked()):
            if (self.lastpage == 1): self.next()
            elif (self.lastpage == 3): self.back()
            if (not self.ch_input.isChecked()):
              if (self.lastpage == 3): self.next()
              elif (self.lastpage == 4): self.back()
        elif (page == 3): #input
          if (not self.ch_input.isChecked()):
            if (self.lastpage == 2): self.next()
            elif (self.lastpage == 4): self.back()
        self.lastpage = self.currentId()

    def func_Done(self):
        dev = shared.editDev
        if not dev:
          dev = self.co_dev.currentText()
        inp = self.co_input.currentText()

        if (self.ch_led.isChecked()): led = "1"
        else: led = "0"
        if (self.ch_js.isChecked()): js = "1"
        else: js = "0"
        if (self.ch_input.isChecked()): in_ = "1"
        else: in_ = "0"
        if (self.ch_rumble.isChecked()): js_rumble = "1"
        else: js_rumble = "0"

        if (self.r_auto_led.isChecked()): led_auto = "1"
        else: led_auto = "0"
        led_n = str(self.r_led_n.value())
        if (self.ch_led_anim.isChecked()): led_anim = "1"
        else: led_anim = "0"

        if (self.r_rumble_new.isChecked()): old_rumble_mode = "0"
        else: old_rumble_mode = "1"

        if (self.ch_timeout.isChecked()): disc = "1"
        else: disc = "0"
        disc_time = str(self.r_timeout.value())

        if (self.js_button.isChecked()): js_button = "1"
        else: js_button = "0"
        if (self.js_sbutton.isChecked()): js_sbutton = "1"
        else: js_sbutton = "0"
        if (self.js_axis.isChecked()): js_axis = "1"
        else: js_axis = "0"
        if (self.js_accel.isChecked()): js_accel = "1"
        else: js_accel = "0"
        if (self.js_accon.isChecked()): js_accon = "1"
        else: js_accon = "0"
        if (self.js_speed.isChecked()): js_speed = "1"
        else: js_speed = "0"
        if (self.js_pos.isChecked()): js_pos = "1"
        else: js_pos = "0"

        if (self.ch_lr3.isChecked()): use_lr3 = "1"
        else: use_lr3 = "0"

        if (os.path.exists(os.getenv("HOME")+"/.qtsixa2/profiles/"+inp) and inp != "(None)"):
          text_input = open(os.getenv("HOME")+"/.qtsixa2/profiles/"+inp, "r").read()
        else:
          text_input = (""
          "# Input - \"(None)\"\n"
          "key_select 0\n"
          "key_l3 0\n"
          "key_r3 0\n"
          "key_start 0\n"
          "key_up 0\n"
          "key_right 0\n"
          "key_down 0\n"
          "key_left 0\n"
          "key_l2 0\n"
          "key_r2 0\n"
          "key_l1 0\n"
          "key_r1 0\n"
          "key_tri 0\n"
          "key_cir 0\n"
          "key_squ 0\n"
          "key_cro 0\n"
          "key_ps 0\n"
          "axis_left_type 0\n"
          "axis_left_up 0\n"
          "axis_left_right 0\n"
          "axis_left_down 0\n"
          "axis_left_left 0\n"
          "axis_right_type 0\n"
          "axis_right_up 0\n"
          "axis_right_right 0\n"
          "axis_right_down 0\n"
          "axis_right_left 0\n"
          "axis_speed 6\n")

        finalFile = (""
        "# ##########################\n"
        "# sixad configuration file #\n"
        "########################## #\n"
        "\n"
        "# Features\n"
        "enable_leds "+led+"\n"
        "enable_joystick "+js+"\n"
        "enable_input "+in_+"\n"
        "enable_rumble "+js_rumble+"\n"
        "enable_timeout "+disc+"\n"
        "\n"
        "# LED\n"
        "led_n_auto "+led_auto+"\n"
        "led_n_number "+led_n+"\n"
        "led_anim "+led_anim+"\n"
        "\n"
        "# Joystick\n"
        "enable_buttons "+js_button+"\n"
        "enable_sbuttons "+js_sbutton+"\n"
        "enable_axis "+js_axis+"\n"
        "enable_accel "+js_accel+"\n"
        "enable_accon "+js_accon+"\n"
        "enable_speed "+js_speed+"\n"
        "enable_pos "+js_pos+"\n"
        "\n"
        +text_input+
        "use_lr3 "+use_lr3+"\n"
        "\n"
        "# Rumble\n"
        "old_rumble_mode "+old_rumble_mode+"\n"
        "\n"
        "# Timeout\n"
        "timeout_mins "+disc_time+"\n"
        "")

        fileW = open("/var/lib/sixad/profiles/"+dev, "w")
        fileW.write(finalFile)
        fileW.close()

