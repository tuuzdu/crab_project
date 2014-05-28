#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import dbus, os, sys
from commands import getoutput
from functools import partial
from PyQt4.QtCore import QFile, QTimer, SIGNAL
from PyQt4.QtGui import QAction, QCursor, QIcon, QMainWindow, QMenu, QMessageBox, QSystemTrayIcon, QWizard

import shared, qtsixa_about, qtsixa_manage, qtsixa_newdev, qtsixa_preferences, qtsixa_reference
import ui_qtsixa_mainw, ui_qtsixa_sixpairw


def look4Root(self):
    if not "kdesudo" in shared.ROOT and "kdesu" in shared.ROOT: #Fix for openSUSE's kdesu not echoing to terminal (opens separate session for sudo)
        return 1
    elif "YES" in getoutput(shared.ROOT+" echo YES"):
        return 1
    else:
        QMessageBox.critical(self, self.tr("QtSixA - Error"), self.tr("Operation not permitted - Not enough rights"))
        return 0

def func_Check_BTs(self):
    if (getoutput("hcitool dev") == "Devices:"):
        QMessageBox.warning(self, self.tr("QtSixA - Warning"), self.tr(""
        "No bluetooth dongles detected.\n"
        "Connect over bluetooth will not be possible"))
    else:
        BT_VER = getoutput("hciconfig default version | grep \"HCI Ver\" | awk '{print$3}'")
        if (BT_VER == "1.0"):
            QMessageBox.critical(self, self.tr("QtSixA - Error"), self.tr(""
            "You're using a _really_ old bluetooth dongle,\n"
            "the Sixaxis will just not work!"))
        elif (BT_VER == "1.1"):
            QMessageBox.warning(self, self.tr("QtSixA - Warning"), self.tr(""
            "You're using a very old bluetooth dongle,\n"
            "the Sixaxis will not work properly!"))




class SixpairW(QWizard, ui_qtsixa_sixpairw.Ui_SixpairW):
    def __init__(self, *args):
        QWizard.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))
        self.connect(self, SIGNAL("currentIdChanged(int)"), self.changedPage)

    def changedPage(self, pageN):
        if (pageN == 1):
	  if self.r_keypad.isChecked(): cmd = " /usr/sbin/sixpair-kbd"
	  else: cmd = " /usr/sbin/sixpair"
          if look4Root(self): self.sixpair_report = getoutput(shared.ROOT+cmd)
          else: self.sixpair_report = self.tr("Not enough rights")
          self.textEdit.setText(self.sixpair_report)
          if (self.sixpair_report == self.tr("Not enough rights")):
            self.label_2.setText(self.tr("Sixpair needs root/admin privileges to run\n \nPlease go back or cancel."))
          elif ("found on USB busses" in self.sixpair_report):
            self.label_2.setText(self.tr("Sixpair reports that no QtSixA compatible device was found.\nIt seems like you forgot something...\n \nPlease go back or cancel.\n \n \nThe sixpair report:"))
          else:
            self.label_2.setText(self.tr("You\'re bluetooth stick/pen/device should now be \npaired with the PS3 devices.\n \nBelow you can see the sixpair report:"))



class MainW(QMainWindow, ui_qtsixa_mainw.Ui_QtSixAMainW):
    def __init__(self, *args):
        QMainWindow.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))

        self.autoListRefresh = QTimer()
        self.autoLQRefresh = QTimer()

        if (shared.Globals.show_warnings): func_Check_BTs(self)

        self.connect(self.b_disconnect, SIGNAL('clicked()'), self.func_Disconnect)
        self.connect(self.b_disconnect_all, SIGNAL('clicked()'), self.func_DiscEverything)
        self.connect(self.b_battery, SIGNAL('clicked()'), self.func_Battery)
        self.connect(self.b_rumble, SIGNAL('clicked()'), self.func_Rumble)
        self.connect(self.act_DiscSixaxis, SIGNAL('triggered()'), self.func_DiscAllSixaxis)
        self.connect(self.act_DiscDevices, SIGNAL('triggered()'), self.func_DiscEverything)
        self.connect(self.act_1, SIGNAL("triggered()"), partial(self.func_DiscSelected, 1))
        self.connect(self.act_2, SIGNAL("triggered()"), partial(self.func_DiscSelected, 2))
        self.connect(self.act_3, SIGNAL("triggered()"), partial(self.func_DiscSelected, 3))
        self.connect(self.act_4, SIGNAL("triggered()"), partial(self.func_DiscSelected, 4))
        self.connect(self.act_5, SIGNAL("triggered()"), partial(self.func_DiscSelected, 5))
        self.connect(self.act_6, SIGNAL("triggered()"), partial(self.func_DiscSelected, 6))
        self.connect(self.act_7, SIGNAL("triggered()"), partial(self.func_DiscSelected, 7))
        self.connect(self.act_8, SIGNAL("triggered()"), partial(self.func_DiscSelected, 8))
        self.connect(self.act_bt_start, SIGNAL('triggered()'), self.func_BT_Start)
        self.connect(self.act_bt_stop, SIGNAL('triggered()'), self.func_BT_Stop)
        self.connect(self.act_Pair, SIGNAL('triggered()'), self.func_Sixpair)
        self.connect(self.act_sixad_force, SIGNAL('triggered()'), self.func_sixad_Force)
        self.connect(self.act_sixad_stop, SIGNAL('triggered()'), self.func_sixad_Stop)

        self.connect(self.act_ManageProf, SIGNAL('triggered()'), self.func_Manage)
        self.connect(self.act_Configure, SIGNAL('triggered()'), self.func_Preferences)
        self.connect(self.act_Debug, SIGNAL('triggered()'), self.func_Debug)
        self.connect(self.act_RestoreDef, SIGNAL('triggered()'), self.func_RestoreDef)

        self.connect(self.act_Manual, SIGNAL('triggered()'), self.func_Manual)
        self.connect(self.act_WebPage, SIGNAL('triggered()'), self.func_WebPage)
        self.connect(self.act_Ubuntu, SIGNAL('triggered()'), self.func_UbuntuForums)
        self.connect(self.act_Bug, SIGNAL('triggered()'), self.func_Report_Bug)
        self.connect(self.act_Donate, SIGNAL('triggered()'), self.func_Donate)
        self.connect(self.act_Reference, SIGNAL('triggered()'), self.func_Reference)
        self.connect(self.act_About, SIGNAL('triggered()'), self.func_About)

        self.connect(self.b_game_help, SIGNAL("clicked()"), self.func_HelpGame)
        self.connect(self.b_game_apply, SIGNAL("clicked()"), self.func_ApplyGame)
        self.connect(self.b_apply_signal, SIGNAL("clicked()"), self.func_Apply_signal)
        #self.connect(self.b_apply_hidraw, SIGNAL("clicked()"), self.func_Apply_hidraw)
        self.connect(self.b_refresh_signal, SIGNAL("clicked()"), self.func_Refresh_signal)
        self.connect(self.b_refresh_hidraw, SIGNAL("clicked()"), self.func_Refresh_hidraw)
        self.connect(self.b_reset_signal, SIGNAL("clicked()"), self.func_Reset_signal)
        self.connect(self.b_tips_signal, SIGNAL("clicked()"), self.func_Tips_signal)
        self.connect(self.b_stop_sixadraw, SIGNAL("clicked()"), self.func_Kill_sixadraw)

        self.connect(self.autoListRefresh, SIGNAL('timeout()'), self.func_UpdateListOfDevices)
        self.connect(self.autoLQRefresh, SIGNAL('timeout()'), self.func_UpdateDeviceLQ)
        self.connect(self.listOfDevices, SIGNAL('currentRowChanged(int)'), self.func_UpdateDeviceStats)
        self.connect(self.listOfDevices, SIGNAL('customContextMenuRequested(QPoint)'), self.func_ProfileMenu)
        self.connect(self.listOfGames, SIGNAL('currentIndexChanged(int)'), self.func_UpdateGames)
        self.connect(self.listOfGames, SIGNAL('currentIndexChanged(int)'), self.func_Game_bOff)

        self.connect(self.radio_etracer_axis, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_etracer_accel, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_etracer_full, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_stk_digital, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_stk_axis, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_stk_accel, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_stk_full, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_neverball_axis, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_neverball_accel, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_game_epsxe_1, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_game_epsxe_axis, SIGNAL("clicked()"), self.func_Game_bOn)
        self.connect(self.radio_game_epsxe_drv, SIGNAL("clicked()"), self.func_Game_bOn)

        self.hidd_number_1 = ""
        self.hidd_number_2 = ""
        self.hidd_number_3 = ""
        self.hidd_number_4 = ""
        self.hidd_number_5 = ""
        self.hidd_number_6 = ""
        self.hidd_number_7 = ""
        self.hidd_number_8 = ""
        self.usb_number_1 = ""
        self.usb_number_2 = ""
        self.usb_number_3 = ""
        self.usb_number_4 = ""
        self.trayTooltip = "Aaah!!"
        self.SixaxisProfile = ""
        self.trayIsActive = False
        #self.wEP.setVisible(True)
        self.wET.setVisible(False)
        self.wNE.setVisible(False)
        self.wSTK.setVisible(False)
        self.b_game_apply.setEnabled(False)

        self.func_UpdateListOfDevices()
        self.func_UpdateDeviceStats()
        self.autoListRefresh.start(2000)

        if (shared.Globals.systray_enabled):
            self.createTrayIcon()
            self.trayIsActive = True
        else:
            self.trayIsActive = False

        if not os.path.exists("/usr/sbin/hcidump"):
            self.b_battery.setVisible(False)


    def func_Sixpair(self):
      SixpairW().exec_()

    def func_RestoreDef(self):
      os.system("rm -rf $HOME/.qtsixa2")
      os.system("rm -rf /var/lib/sixad/*")
      QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("Please restart QtSixA Now"))

    def func_Debug(self):
      if (os.path.exists("/etc/default/sixad")):
        debug_mode = int(open("/etc/default/sixad").read().split("DEBUG=")[1][0])
        ask = QMessageBox.question(self, self.tr("QtSixA - Debug"), self.tr("Debug mode is %1.\n\nClick 'Yes' to enable it or 'No' to disable it.").arg(
                                         "enabled" if debug_mode else "disabled"), QMessageBox.Yes | QMessageBox.No)
        if (ask == QMessageBox.Yes and not debug_mode):
          if look4Root(self):
            os.system(shared.ROOT+" sed -i s/DEBUG=0/DEBUG=1/ /etc/default/sixad")
        elif (ask == QMessageBox.No and debug_mode):
          if look4Root(self):
            os.system(shared.ROOT+" sed -i s/DEBUG=1/DEBUG=0/ /etc/default/sixad")
      else:
        QMessageBox.warning(self, self.tr("QtSixA - Warning"), self.tr("Could not check debug mode.<br><i>The file '/etc/default/sixad' does not exist!</i>"))

    def func_Rumble(self):
      print "test rumble"

    def func_ProfileMenu(self):
      if (self.listOfDevices.currentRow() < 0):
        return

      # Create Menu
      cMenu = QMenu()
      act_x_specific = cMenu.addAction(self.tr("Configure this specific Device"))
      act_x_default = cMenu.addAction(self.tr("Configure default profile"))
      act_x_hidraw = cMenu.addAction(self.tr("Configure hidraw profile"))
      cMenu.addSeparator()
      act_x_manage = cMenu.addAction(self.act_ManageProf)
      cMenu.addSeparator()
      act_x_disconnect = cMenu.addAction(self.tr("&Disconnect"))
      act_x_disconnect.setIcon(QIcon(":/icons/eject.png"))

      if (self.listOfDevices.currentRow() >= 9):
        act_x_specific.setVisible(False)
        act_x_disconnect.setVisible(False)
      else:
        act_x_hidraw.setVisible(False)

      # Show Menu at cursor position
      newPos = QCursor.pos()
      cMenu.setGeometry(newPos.x(), newPos.y(), 0, 0)
      act_x_sel = cMenu.exec_()

      if (act_x_sel == act_x_default):
        shared.editDev = "default"
        self.func_x_manage()
      elif (act_x_sel == act_x_hidraw):
        shared.editDev = "hidraw"
        self.func_x_manage()
      elif (act_x_sel == act_x_specific):
        shared.editDev = str(self.listOfDevices.currentItem().text())
        self.func_x_manage()
      elif (act_x_sel == act_x_disconnect):
        self.func_Disconnect()

    def func_x_manage(self):
        qtsixa_newdev.NewDevW().exec_()
        self.func_refreshDevProfile()

    def func_BT_Start(self):
        if look4Root(self):
            os.system("if [ -f /lib/udev/rules.d/97-bluetooth.rules ]; then "+shared.ROOT+" bluetoothd --udev; else "+shared.ROOT+" /etc/init.d/bluetooth start; fi")
            QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("Bluetooth should now be started (restored)"))

    def func_BT_Stop(self):
        if look4Root(self):
            os.system(shared.ROOT+" pkill -KILL bluetoothd")
            QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("Bluetooth should now be stopped"))

    def func_sixad_Force(self):
        if look4Root(self):
            os.system(shared.ROOT+" sixad "+"--force "+"&")
            QMessageBox.information(self, self.tr("QtSixA - Forced"), self.tr(""
            "The sixad driver has now been forced to start.<p>"
            "You should be able to connect your devices with no problem now<br>"
            "<i>(but please note that bluetooth is not workable for anything else in this mode, "
            "so you should stop sixad when you need to do something else)</i>"))

    def func_sixad_Stop(self):
        if look4Root(self):
            os.system(shared.ROOT+" sixad "+"--stop")
            QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("The modules have been stopped"))

    def func_Manage(self):
      qtsixa_manage.ManageW().exec_()
      self.func_refreshDevProfile()

    def func_Preferences(self):
      qtsixa_preferences.PreferencesW().exec_()

    def func_Manual(self):
        os.system("xdg-open file:///usr/share/doc/qtsixa/manual/manual_index.html")

    def func_WebPage(self):
        os.system("xdg-open \"http://qtsixa.sourceforge.net/\"")

    def func_UbuntuForums(self):
        os.system("xdg-open \"http://ubuntuforums.org/showthread.php?p=7472939\"")

    def func_Report_Bug(self):
        os.system("xdg-open \"https://bugs.launchpad.net/qtsixa\"")

    def func_Donate(self):
        os.system("xdg-open \"https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=9305140\"")

    def func_Reference(self):
      qtsixa_reference.ReferenceW().exec_()

    def func_About(self):
      qtsixa_about.AboutW().exec_()

    def func_refreshDevProfile(self):
      if self.listOfDevices.currentRow() > 0:
        if self.listOfDevices.currentRow() >= 9:
            devAdr = "default"
        else:
            devAdr = str(self.listOfDevices.currentItem().text())
        dev = shared.func_checkDeviceOptions(devAdr)
        self.lineProfile.setText(self.tr("LED: %1 | Joystick: %2 | Input: %3").arg(dev[0]).arg(dev[1]).arg(dev[2]))


#Disconnect stuff  ### ---------------------------------------------------------------------------------------

    def func_DBusDisconnect(self, mode, name, mac):
        bus = dbus.SystemBus()
        try:
            bluez_bus = bus.get_object('org.bluez', '/')
            bluez_id = bluez_bus.DefaultAdapter(dbus_interface='org.bluez.Manager')
            adapter_bus = bus.get_object('org.bluez', bluez_id)
            listDev = adapter_bus.ListDevices(dbus_interface='org.bluez.Adapter')
        except:
            print self.tr("Could not disconnect device(s) through DBus, will use hcitool")
            listDev = ""
            if (mode == "single"):
                if look4Root(self): os.system(shared.ROOT+" hcitool "+"dc "+mac)
            elif (mode == "sixaxis"):
                self.look4Sixaxis = getoutput("hcitool con | grep ACL | awk '{printf$3\" \"}'").split()
                if look4Root(self):
                    s = 0
                    while s < len(self.look4Sixaxis):
                        if "PLAYSTATION(R)3 Controller" in getoutput("hcitool "+"name "+self.look4Sixaxis[s]):
                            os.system(shared.ROOT+" hcitool "+"dc "+self.look4Sixaxis[s])
                        s += 1
            elif (mode == "all"):
                if "ACL" in getoutput("hcitool con | grep ACL"):
                    if look4Root(self): os.system(shared.ROOT+" `hcitool con | grep ACL | awk '{printf\"hcitool dc \"$3\"\\n\"}'`")
            else:
                print self.tr("Could not disconnect some devices; Maybe bluetooth is off or you don't have permissions?")
                QMessageBox.warning(self, self.tr("QtSixA - Information"), self.tr(""
                "Could not disconect some devices.<br>"
                "Maybe bluetooth is off or you don't have permissions?"))

        j = 0
        while j < len(listDev):
            device_bus = bus.get_object('org.bluez', listDev[j])
            idev = dbus.Interface(device_bus, dbus_interface='org.bluez.Device')
            if mode == "single":
                if name in listDev[j]: idev.Disconnect()
            elif mode == "sixaxis":
                if "PLAYSTATION(R)3 Controller" in str(idev.GetProperties()): idev.Disconnect()
            elif mode == "all":
                idev.Disconnect()
            j += 1


    def func_Disconnect(self):
        self.DeviceToDisconnect = self.listOfDevices.currentRow()
        if (self.DeviceToDisconnect == 1): self.selectedDevice = self.hidd_number_1
        elif (self.DeviceToDisconnect == 2): self.selectedDevice = self.hidd_number_2
        elif (self.DeviceToDisconnect == 3): self.selectedDevice = self.hidd_number_3
        elif (self.DeviceToDisconnect == 4): self.selectedDevice = self.hidd_number_4
        elif (self.DeviceToDisconnect == 5): self.selectedDevice = self.hidd_number_5
        elif (self.DeviceToDisconnect == 6): self.selectedDevice = self.hidd_number_6
        elif (self.DeviceToDisconnect == 7): self.selectedDevice = self.hidd_number_7
        elif (self.DeviceToDisconnect == 8): self.selectedDevice = self.hidd_number_8
        else:
            print self.tr("Device not connected; Cannot disconnect")
            return
        self.selectedDeviceParsed = getoutput("echo "+self.selectedDevice+" | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' ")
        self.func_DBusDisconnect("single", self.selectedDeviceParsed, self.selectedDevice)
        self.listOfDevices.setCurrentRow(-1)

    def func_Battery(self):
        self.label_bat.setEnabled(1)
        self.barBattery.setEnabled(1)
        self.barBattery.setTextVisible(1)
        self.barBattery.setMaximum(5)
        self.DeviceToCheckBattery = self.listOfDevices.currentRow()
        if (self.DeviceToCheckBattery == 1): self.DeviceToCheck = self.hidd_number_1
        elif (self.DeviceToCheckBattery == 2): self.DeviceToCheck = self.hidd_number_2
        elif (self.DeviceToCheckBattery == 3): self.DeviceToCheck = self.hidd_number_3
        elif (self.DeviceToCheckBattery == 4): self.DeviceToCheck = self.hidd_number_4
        elif (self.DeviceToCheckBattery == 5): self.DeviceToCheck = self.hidd_number_5
        elif (self.DeviceToCheckBattery == 6): self.DeviceToCheck = self.hidd_number_6
        elif (self.DeviceToCheckBattery == 7): self.DeviceToCheck = self.hidd_number_7
        elif (self.DeviceToCheckBattery == 8): self.DeviceToCheck = self.hidd_number_8
        else: print str(self.tr("Device not connected; Cannot check battery"))
        if look4Root(self): self.SixaxisBat = str(getoutput(shared.ROOT+" hcidump "+"-R "+"-O '"+self.DeviceToCheck+"' "+"| "+"head "+"-n "+"5 "+"| "+"tail "+"-n "+"1 "+"| "+"awk "+"'{printf$1}' "+"& "+"sleep "+"1 "+"&& "+shared.ROOT+" killall "+"hcidump "+"> "+"/dev/null"))
        else: self.SixaxisBat = ""
        if self.SixaxisBat.isdigit():
          self.barBattery.setValue(int(self.SixaxisBat))
        else:
            if self.SixaxisBat == "EE": self.barBattery.setMaximum(0)
            elif self.SixaxisBat == "HCI": print "Device not connected; Cannot check battery (2)" #(2) - to know what is the exact error
            else: print "Error while trying to check battery"

    def func_DiscAllSixaxis(self):
        self.func_DBusDisconnect("sixaxis", "NULL", "NULL")
        self.listOfDevices.setCurrentRow(-1)

    def func_DiscEverything(self):
        self.func_DBusDisconnect("all", "NULL", "NULL")
        self.listOfDevices.setCurrentRow(-1)

    def func_DiscSelected(self, number):
        if number == 1: self.selectedDevice = self.hidd_number_1
        elif number == 2: self.selectedDevice = self.hidd_number_2
        elif number == 3: self.selectedDevice = self.hidd_number_3
        elif number == 4: self.selectedDevice = self.hidd_number_4
        elif number == 5: self.selectedDevice = self.hidd_number_5
        elif number == 6: self.selectedDevice = self.hidd_number_6
        elif number == 7: self.selectedDevice = self.hidd_number_7
        elif number == 8: self.selectedDevice = self.hidd_number_8
        self.selectedDeviceParsed = getoutput("echo "+self.selectedDevice+" | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' | awk 'sub(\":\",\"_\")' ")
        self.func_DBusDisconnect("single", self.selectedDeviceParsed, self.selectedDevice)
        self.listOfDevices.setCurrentRow(-1)

#Disconnect stuff  ### ---------------------------------------------------------------------------------------


#Managing stuff  ### -----------------------------------------------------------------------------------------

    def func_UpdateListOfDevices(self):
        self.Check4BluetoothDevices = getoutput("hcitool con")
        self.Check4USBDevices = getoutput("lsusb")

        if not "ACL" in self.Check4BluetoothDevices:
            self.listOfDevices.item(1).setHidden(1)
            self.listOfDevices.item(2).setHidden(1)
            self.listOfDevices.item(3).setHidden(1)
            self.listOfDevices.item(4).setHidden(1)
            self.listOfDevices.item(5).setHidden(1)
            self.listOfDevices.item(6).setHidden(1)
            self.listOfDevices.item(7).setHidden(1)
            self.listOfDevices.item(8).setHidden(1)
            self.act_1.setVisible(0)
            self.act_2.setVisible(0)
            self.act_3.setVisible(0)
            self.act_4.setVisible(0)
            self.act_5.setVisible(0)
            self.act_6.setVisible(0)
            self.act_7.setVisible(0)
            self.act_8.setVisible(0)
            self.act_none.setVisible(1)
            self.hidd_number_1 = self.hidd_number_2 = self.hidd_number_3 = self.hidd_number_4 = self.hidd_number_5 = self.hidd_number_6 = self.hidd_number_7 = self.hidd_number_8 = ""
            if (not "054c:03a0" in self.Check4USBDevices) and (not "054c:0306" in self.Check4USBDevices) and (not "0079:0006" in self.Check4USBDevices) and (not "054c:0268" in self.Check4USBDevices):
                self.listOfDevices.item(9).setHidden(1)
                self.listOfDevices.item(10).setHidden(1)
                self.listOfDevices.item(11).setHidden(1)
                self.listOfDevices.item(12).setHidden(1)
                self.listOfDevices.item(0).setHidden(0)
                self.listOfDevices.setSelectionMode(0)
                self.usb_number_1 = self.usb_number_2 = self.usb_number_3 = self.usb_number_4 = ""
        else:
            self.func_UpdateBluetoothNames()
            self.listOfDevices.setSelectionMode(1)
            self.listOfDevices.item(0).setHidden(1)
            self.act_none.setVisible(0)
            if (self.hidd_number_1 == ""): self.listOfDevices.item(1).setHidden(1), self.act_1.setVisible(0)
            else:
                self.listOfDevices.item(1).setText(self.hidd_number_1)
                self.listOfDevices.item(1).setHidden(0)
                self.act_1.setVisible(1)
                self.act_1.setText(self.hidd_number_1)
            if (self.hidd_number_2 == ""): self.listOfDevices.item(2).setHidden(1), self.act_2.setVisible(0)
            else:
                self.listOfDevices.item(2).setText(self.hidd_number_2)
                self.listOfDevices.item(2).setHidden(0)
                self.act_2.setVisible(1)
                self.act_2.setText(self.hidd_number_2)
            if (self.hidd_number_3 == ""): self.listOfDevices.item(3).setHidden(1), self.act_3.setVisible(0)
            else:
                self.listOfDevices.item(3).setText(self.hidd_number_3)
                self.listOfDevices.item(3).setHidden(0)
                self.act_3.setVisible(1)
                self.act_3.setText(self.hidd_number_3)
            if (self.hidd_number_4 == ""): self.listOfDevices.item(4).setHidden(1), self.act_4.setVisible(0)
            else:
                self.listOfDevices.item(4).setText(self.hidd_number_4)
                self.listOfDevices.item(4).setHidden(0)
                self.act_4.setVisible(1)
                self.act_4.setText(self.hidd_number_4)
            if (self.hidd_number_5 == ""): self.listOfDevices.item(5).setHidden(1), self.act_5.setVisible(0)
            else:
                self.listOfDevices.item(5).setText(self.hidd_number_5)
                self.listOfDevices.item(5).setHidden(0)
                self.act_5.setVisible(1)
                self.act_5.setText(self.hidd_number_5)
            if (self.hidd_number_6 == ""): self.listOfDevices.item(6).setHidden(1), self.act_6.setVisible(0)
            else:
                self.listOfDevices.item(6).setText(self.hidd_number_6)
                self.listOfDevices.item(6).setHidden(0)
                self.act_6.setVisible(1)
                self.act_6.setText(self.hidd_number_6)
            if (self.hidd_number_7 == ""): self.listOfDevices.item(7).setHidden(1), self.act_7.setVisible(0)
            else:
                self.listOfDevices.item(7).setText(self.hidd_number_7)
                self.listOfDevices.item(7).setHidden(0)
                self.act_7.setVisible(1)
                self.act_7.setText(self.hidd_number_7)
            if (self.hidd_number_8 == ""): self.listOfDevices.item(8).setHidden(1), self.act_8.setVisible(0)
            else:
                self.listOfDevices.item(8).setText(self.hidd_number_8)
                self.listOfDevices.item(8).setHidden(0)
                self.act_8.setVisible(1)
                self.act_8.setText(self.hidd_number_8)

        if (not "054c:03a0" in self.Check4USBDevices) and (not "054c:0306" in self.Check4USBDevices) and (not "0079:0006" in self.Check4USBDevices) and (not "054c:0268" in self.Check4USBDevices):
            self.listOfDevices.item(9).setHidden(1)
            self.listOfDevices.item(10).setHidden(1)
            self.listOfDevices.item(11).setHidden(1)
            self.listOfDevices.item(12).setHidden(1)
            self.usb_number_1 = self.usb_number_2 = self.usb_number_3 = self.usb_number_4 = ""
        else:
            self.func_UpdateUSBNames()
            self.listOfDevices.item(0).setHidden(1)
            self.listOfDevices.item(9).setHidden(0)
            self.listOfDevices.item(10).setHidden(0)
            self.listOfDevices.item(11).setHidden(0)
            self.listOfDevices.item(12).setHidden(0)
            self.listOfDevices.setSelectionMode(1)
            if (self.usb_number_1 == ""): self.listOfDevices.item(9).setHidden(1)
            else: self.listOfDevices.item(9).setText(self.usb_number_1)
            if (self.usb_number_2 == ""): self.listOfDevices.item(10).setHidden(1)
            else: self.listOfDevices.item(10).setText(self.usb_number_2)
            if (self.usb_number_3 == ""): self.listOfDevices.item(11).setHidden(1)
            else: self.listOfDevices.item(11).setText(self.usb_number_3)
            if (self.usb_number_4 == ""): self.listOfDevices.item(12).setHidden(1)
            else: self.listOfDevices.item(12).setText(self.usb_number_4)

        self.func_UpdateTrayTooltip()


    def func_UpdateBluetoothNames(self):
        self.hidd_number_1 = self.hidd_number_2 = self.hidd_number_3 = self.hidd_number_4 = self.hidd_number_5 = self.hidd_number_6 = self.hidd_number_7 = self.hidd_number_8 = ""
        self.nOfDevices = int(getoutput("echo '"+self.Check4BluetoothDevices+"' | grep ACL -n | tail -n 1 | awk '{printf$1}' | awk 'sub(\":\",\"\")'")) - 1
        if self.nOfDevices > 0: self.hidd_number_1 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 2 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 1: self.hidd_number_2 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 3 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 2: self.hidd_number_3 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 4 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 3: self.hidd_number_4 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 5 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 4: self.hidd_number_5 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 6 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 5: self.hidd_number_6 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 7 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 6: self.hidd_number_7 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 8 | tail -n 1 | awk '{printf$3}'")
        if self.nOfDevices > 7: self.hidd_number_8 = getoutput("echo '"+self.Check4BluetoothDevices+"' | head -n 9 | tail -n 1 | awk '{printf$3}'")

    def func_UpdateUSBNames(self):
        self.usb_number_1 = self.usb_number_2 = self.usb_number_3 = self.usb_number_4 = ""
        self.nOfDevices = int(getoutput("echo '"+self.Check4USBDevices+"' | grep -e \"054c:03a0\" -e \"054c:0306\" -e \"054c:0268\" -e \"0079:0006\" | grep -e \"054c:03a0\" -e \"054c:0306\" -e \"054c:0268\" -e \"0079:0006\"  -n | tail -n 1 | awk '{printf$1}' | awk 'sub(\":Bus\",\"\")'"))
        if self.nOfDevices > 0: self.usb_number_1 = getoutput("echo '"+self.Check4USBDevices+"' | grep -e \"054c:03a0\" -e \"054c:0306\" -e \"054c:0268\" -e \"0079:0006\" | awk '{printf$2\":\"$4$6\"\\n\"}' | head -n 1 | tail -n 1")
        if self.nOfDevices > 1: self.usb_number_2 = getoutput("echo '"+self.Check4USBDevices+"' | grep -e \"054c:03a0\" -e \"054c:0306\" -e \"054c:0268\" -e \"0079:0006\" | awk '{printf$2\":\"$4$6\"\\n\"}' | head -n 2 | tail -n 1")
        if self.nOfDevices > 2: self.usb_number_3 = getoutput("echo '"+self.Check4USBDevices+"' | grep -e \"054c:03a0\" -e \"054c:0306\" -e \"054c:0268\" -e \"0079:0006\" | awk '{printf$2\":\"$4$6\"\\n\"}' | head -n 3 | tail -n 1")
        if self.nOfDevices > 3: self.usb_number_4 = getoutput("echo '"+self.Check4USBDevices+"' | grep -e \"054c:03a0\" -e \"054c:0306\" -e \"054c:0268\" -e \"0079:0006\" | awk '{printf$2\":\"$4$6\"\\n\"}' | head -n 4 | tail -n 1")

    def func_UpdateDeviceStats(self):
        if self.listOfDevices.currentRow() <= 0: #No device
            self.b_battery.setEnabled(0)
            self.b_rumble.setEnabled(0)
            self.groupDevice.setEnabled(0)
            self.groupBluetooth.setEnabled(0)
            self.groupProfile.setEnabled(0)
            self.groupActions.setEnabled(0)
            self.lineName.setText("")
            self.lineType.setText("")
            self.lineID.setText("")
            self.lineMode.setText("")
            self.lineAddress.setText("")
            self.lineProfile.setText("")
            self.barLQ.setTextVisible(0)
            self.barLQ.setValue(0)
            self.barBattery.setTextVisible(0)
            self.barBattery.setValue(0)
            self.autoLQRefresh.stop()
            self.lineProfile.setText("")
            self.lineDate.setText("")
        elif self.listOfDevices.currentRow() >= 9: #USB selected
            self.parsedID = str(self.listOfDevices.item(self.listOfDevices.currentRow()).text()).split(":")
            if (self.parsedID[2] == "054c" and self.parsedID[3] == "0268"):
                self.devName = "PLAYSTATION(R)3 Controller"
                self.devType = "Joystick"
            elif (self.parsedID[2] == "054c" and self.parsedID[3] == "03a0"):
                self.devName = "Wireless Keypad"
                self.devType = "Keypad"
            elif (self.parsedID[2] == "054c" and self.parsedID[3] == "0306"):
                self.devName = "PLAYSTATION(R)3 Remote"
                self.devType = "Remote"
            elif (self.parsedID[2] == "0079" and self.parsedID[3] == "0006"):
                self.devName = "Generic USB Joystick"
                self.devType = "Joystick"
            else:
                self.devName = self.tr("Unknown")
                self.devType = self.tr("Unknown")
            self.b_battery.setEnabled(0)
            self.b_rumble.setEnabled(0)
            self.groupDevice.setEnabled(1)
            self.groupBluetooth.setEnabled(0)
            self.groupProfile.setEnabled(1)
            self.groupActions.setEnabled(0)
            self.lineName.setText(self.devName)
            self.lineType.setText(self.devType)
            self.lineID.setText(self.parsedID[2]+":"+self.parsedID[3])
            self.lineMode.setText("USB")
            self.lineAddress.setText("")
            self.lineProfile.setText("(default)")
            self.barLQ.setTextVisible(0)
            self.barLQ.setValue(0)
            self.barBattery.setTextVisible(0)
            self.barBattery.setValue(0)
            self.autoLQRefresh.stop()
            dev = shared.func_checkDeviceOptions("default")
            self.lineProfile.setText(self.tr("LED: %1 | Joystick: %2 | Input: %3").arg(dev[0]).arg(dev[1]).arg(dev[2]))
            self.lineDate.setText("")
        else: #Bluetooth
            self.devName = getoutput("hcitool name "+str(self.listOfDevices.item(self.listOfDevices.currentRow()).text()))
            if (self.devName == "PLAYSTATION(R)3 Controller"):
                self.devType = "Joystick"
                self.devID = "054c:0268"
            elif (self.devName == "Wireless Keypad"):
                self.devType = "Keypad"
                self.devID = "054c:03a0"
            elif (self.devName == "PLAYSTATION(R)3 Remote"):
                self.devType = "Remote"
                self.devID = "054c:0306"
            else:
                self.devType = self.tr("Unknown")
                self.devID = self.tr("Unknown")
            self.groupDevice.setEnabled(1)
            self.groupBluetooth.setEnabled(1)
            self.groupProfile.setEnabled(1)
            self.groupActions.setEnabled(1)
            if self.devType == "Joystick":
              self.b_battery.setEnabled(1)
              self.b_rumble.setEnabled(0) #TODO
            else:
              self.b_battery.setEnabled(0)
              self.b_rumble.setEnabled(0)
            self.lineName.setText(self.devName)
            self.lineType.setText(self.devType)
            self.lineID.setText(self.devID)
            self.lineMode.setText("Bluetooth")
            address = str(self.listOfDevices.item(self.listOfDevices.currentRow()).text())
            self.lineAddress.setText(address)
            if (os.path.exists("/var/lib/sixad/profiles/"+address)):
              self.lineProfile.setText(getoutput("cat /var/lib/sixad/profiles/"+address+" | grep name | head -n 1 | awk 'sub(\"name \",\"\")'"))
            else:
              self.lineProfile.setText("(default)")
            self.barLQ.setTextVisible(1)
            devTextInfo = str(self.listOfDevices.currentItem().text())
            self.devLQ = getoutput("hcitool lq "+devTextInfo+" | awk '{printf$3}'")
            if not ("o" in self.devLQ): self.barLQ.setValue(int(self.devLQ))
            self.label_bat.setEnabled(0)
            self.barBattery.setEnabled(0)
            self.barBattery.setTextVisible(0)
            self.barBattery.setValue(0)
            self.autoLQRefresh.start(5000)
            dev = shared.func_checkDeviceOptions(devTextInfo)
            self.lineProfile.setText(self.tr("LED: %1 | Joystick: %2 | Input: %3 | Rumble: %4").arg(dev[0]).arg(dev[1]).arg(dev[2]).arg(dev[3]))
            dirs = os.listdir("/var/lib/sixad")
            self.lineDate.setText("Unknown")
            for z in range(len(dirs)):
              fileZ = "/var/lib/sixad/"+dirs[z]+"/lastused"
              if os.path.exists(fileZ):
                if (devTextInfo in open(fileZ, "r").read()):
                  fileX = open(fileZ, "r").read()
                  text = fileX.split(devTextInfo+" ")[1].split("\n")[0]
                  if (text): self.lineDate.setText(text)
                break

    def func_UpdateDeviceLQ(self):
        self.devLQ = getoutput("hcitool lq "+str(self.listOfDevices.item(self.listOfDevices.currentRow()).text())+" | awk '{printf$3}'")
        if not ("o" in self.devLQ): self.barLQ.setValue(int(self.devLQ))

#Managing stuff  ### -----------------------------------------------------------------------------------------


#Games stuff  ### --------------------------------------------------------------------------------------------

    def func_UpdateGames(self):
        self.wEP.setVisible(0)
        self.wET.setVisible(0)
        self.wNE.setVisible(0)
        self.wSTK.setVisible(0)
        if self.listOfGames.currentText() == "ePSXe (Wine)":
            self.wEP.setVisible(1)
        elif self.listOfGames.currentText() == "Extreme Tux Racer":
            self.wET.setVisible(1)
        elif self.listOfGames.currentText() == "Neverball / Neverputt":
            self.wNE.setVisible(1)
        elif self.listOfGames.currentText() == "Super Tux Kart":
            self.wSTK.setVisible(1)

    def func_ApplyGame(self):
        if self.listOfGames.currentText() == "ePSXe (Wine)":
            if os.path.exists((os.getenv("HOME"))+"/.wine/user.reg"):
                self.applied = 1
                if self.radio_game_epsxe_1.isChecked():
                    os.system("regedit "+"/usr/share/qtsixa/game-profiles/wine-epsxe_axis.reg")
                elif self.radio_game_epsxe_axis.isChecked():
                    os.system("regedit "+"/usr/share/qtsixa/game-profiles/wine-epsxe_accel-mov.reg")
                else:
                    os.system("regedit "+"/usr/share/qtsixa/game-profiles/wine-epsxe_accel-driv.reg")
            else: self.applied = 0
        elif self.listOfGames.currentText() == "Extreme Tux Racer":
            if os.path.exists((os.getenv("HOME"))+"/.etracer/options"):
                self.applied = 1
                if self.radio_etracer_axis.isChecked():
                    content = getoutput('PREV=`cat $HOME/.etracer/options | head -n 150`; MOD=`cat /usr/share/qtsixa/game-profiles/etracer_axis`; NEXT=`cat $HOME/.etracer/options | tail -n 325`; echo "$PREV $MOD $NEXT"')
                elif self.radio_etracer_accel.isChecked():
                    content = getoutput('PREV=`cat $HOME/.etracer/options | head -n 150`; MOD=`cat /usr/share/qtsixa/game-profiles/etracer_accel`; NEXT=`cat $HOME/.etracer/options | tail -n 325`; echo "$PREV $MOD $NEXT"')
                else:
                    content = getoutput('PREV=`cat $HOME/.etracer/options | head -n 150`; MOD=`cat /usr/share/qtsixa/game-profiles/etracer_full`; NEXT=`cat $HOME/.etracer/options | tail -n 325`; echo "$PREV $MOD $NEXT"')
                newFile = QFile('/tmp/etracer_options')
                if not newFile.open(QFile.WriteOnly | QFile.Text):
                    QMessageBox.warning(self, self.tr("QtSixA - Error"), self.tr("Cannot write to file.\nPlease check if the location you selected is not read-only or if you enough space left on disk."))
                else:
                    newFile.writeData(content)
                    newFile.close()
                    os.system("cp /tmp/etracer_options $HOME/.etracer/options")
            else: self.applied = 0
        elif self.listOfGames.currentText() == "Neverball / Neverputt":
            if os.path.exists((os.getenv("HOME"))+"/.neverball/neverballrc"):
                self.applied = 1
                if self.radio_neverball_axis.isChecked():
                    content = getoutput('PREV=`cat $HOME/.neverball/neverballrc | head -n 27`; MOD=`cat /usr/share/qtsixa/game-profiles/neverballrc_axis`; NEXT=`cat $HOME/.neverball/neverballrc | tail -n 26`; echo "$PREV $MOD $NEXT"')
                else:
                    content = getoutput('PREV=`cat $HOME/.neverball/neverballrc | head -n 27`; MOD=`cat /usr/share/qtsixa/game-profiles/neverballrc_accel`; NEXT=`cat $HOME/.neverball/neverballrc | tail -n 26`; echo "$PREV $MOD $NEXT"')
                newFile = QFile('/tmp/neverballrc')
                if not newFile.open(QFile.WriteOnly | QFile.Text):
                    QMessageBox.warning(self, self.tr("QtSixA - Error"), self.tr("Cannot write to file.\nPlease check if the location you selected is not read-only or if you enough space left on disk."))
                else:
                    newFile.writeData(content)
                    newFile.close()
                    os.system("cp /tmp/neverballrc $HOME/.neverball/neverballrc")
            else: self.applied = 0
        elif self.listOfGames.currentText() == "Super Tux Kart":
            if os.path.exists((os.getenv("HOME"))+"/.supertuxkart/config"):
                self.applied = 1
                self.config_n_stk_1 = getoutput("cat $HOME/.supertuxkart/config | grep -n \"player 1 settings\" | awk '{printf$1}' | awk 'sub(\":\",\"\")'")
                self.config_n_stk = str(  (int(self.config_n_stk_1) + 4) )
                if self.radio_stk_digital.isChecked():
                    content = getoutput('PREV=`cat $HOME/.supertuxkart/config | head -n '+self.config_n_stk+'`; MOD=`cat /usr/share/qtsixa/game-profiles/stk_digital`; echo "$PREV $MOD"')
                elif self.radio_stk_axis.isChecked():
                    content = getoutput('PREV=`cat $HOME/.supertuxkart/config | head -n '+self.config_n_stk+'`; MOD=`cat /usr/share/qtsixa/game-profiles/stk_axis`; echo "$PREV $MOD"')
                elif self.radio_stk_accel.isChecked():
                    content = getoutput('PREV=`cat $HOME/.supertuxkart/config | head -n '+self.config_n_stk+'`; MOD=`cat /usr/share/qtsixa/game-profiles/stk_accel`; echo "$PREV $MOD"')
                else:
                    content = getoutput('PREV=`cat $HOME/.supertuxkart/config | head -n '+self.config_n_stk+'`; MOD=`cat /usr/share/qtsixa/game-profiles/stk_full`; echo "$PREV $MOD"')
                newFile = QFile('/tmp/supertuxkart_config')
                if not newFile.open(QFile.WriteOnly | QFile.Text):
                    QMessageBox.warning(self, self.tr("QtSixA - Error"), self.tr("Cannot write to file.\nPlease check if the location you selected is not read-only or if you enough space left on disk."))
                else:
                    newFile.writeData(content)
                    newFile.close()
                    os.system("cp /tmp/supertuxkart_config $HOME/.supertuxkart/config")
            else: self.applied = 0
        else: return

        if (self.applied): self.func_Game_msg()
        else: self.func_Game_msgNO()


    def func_Game_bOn(self):
        self.b_game_apply.setEnabled(1)

    def func_Game_bOff(self):
        self.b_game_apply.setEnabled(0)

    def func_Game_msg(self):
        QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("Done!\nNow just launch the game to start the fun!"))

    def func_Game_msgNO(self):
        QMessageBox.warning(self, self.tr("QtSixA - Information"), self.tr("Error!\nThe chosen game has never been started before...\nIt's configuration file does not exist!"))

    def func_HelpGame(self):
        QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr(""
        "This is easy:<p>"
        "<b>1.</b> Select the game from the combo-box in the middle<br>"
        "<b>2.</b> Choose a configuration on the buttons below<br>"
        "<b>3.</b> Click the apply button on the bottom right<p>"
        "<i>Please be sure to run the game at least once before clicking \"Apply\",<br>"
        "or the profile may not be applied sucessfully</i>"))

#Games stuff  ### --------------------------------------------------------------------------------------------


#Advanced  ### -----------------------------------------------------------------------------------------------
    def func_Apply_signal(self):
        if (self.line_signal_bash.displayText() == ""):
            QMessageBox.warning(self, self.tr("QtSixA - Information"), self.tr("Something is missing...\n(Empty slot is not possible!)"))
        else:
            if self.box_signal_disc.isChecked():
                self.signal_disc = "1"
            else:
                self.signal_disc = "0"
            QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("The action has been set."))
            if self.combo_signal_operation.currentIndex() == 0: self.operation = "lower"
            elif self.combo_signal_operation.currentIndex() == 1: self.operation = "higher"
            elif self.combo_signal_operation.currentIndex() == 2: self.operation = "equal"
            else: self.operation = "error"
            os.system('sixad-lq' + ' ' + str(self.combo_signal_device.currentText()) + ' ' + str(self.operation) + ' ' + str(self.spin_signal.value()) + ' ' + str(self.signal_disc) + ' ' + str(self.line_signal_bash.displayText()) + ' &' )

    def func_Reset_signal(self):
        os.system("pkill -KILL sixad-lq > /dev/null")
        QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr("All previous set-up actions were stopped."))

    def func_Tips_signal(self):
        QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr(""
        "Commonly used commands:"
        "<p>"
        "<b>1.</b> Any suggestions ??"
        ""))

    #def func_Apply_hidraw(self):
        #if look4Root(self):
            #os.system(shared.ROOT+" /sbin/modprobe uinput")
            #self.rawReport = getoutput(shared.ROOT+" /usr/sbin/sixad-raw "+"--test "+str(self.combo_hidraw.currentText()))
            #if (self.rawReport == "Found a Sixaxis"):
                #os.system(shared.ROOT+" /usr/sbin/sixad-raw "+str(self.combo_hidraw.currentText())+" &")
                #QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr(""
                #"The sixad driver has been initialized on the selected hidraw device sucessfully"))
            #else:
                #QMessageBox.warning(self, self.tr("QtSixA - Error"), self.tr(""
                #"The sixad driver could not start.\nAre you sure you selected a Sixaxis?"))

    def func_Refresh_signal(self):
        self.combo_signal_device.clear()
        if not "ACL" in getoutput("hcitool con"):
            self.combo_signal_device.addItem(self.tr("No Sixaxis found"))
            self.combo_signal_device.setEnabled(0)
            self.b_apply_signal.setEnabled(0)
        else:
            self.combo_signal_device.setEnabled(1)
            self.b_apply_signal.setEnabled(1)
            if (self.hidd_number_1 != ""): self.combo_signal_device.addItem(self.hidd_number_1)
            if (self.hidd_number_2 != ""): self.combo_signal_device.addItem(self.hidd_number_2)
            if (self.hidd_number_3 != ""): self.combo_signal_device.addItem(self.hidd_number_3)
            if (self.hidd_number_4 != ""): self.combo_signal_device.addItem(self.hidd_number_4)
            if (self.hidd_number_5 != ""): self.combo_signal_device.addItem(self.hidd_number_5)
            if (self.hidd_number_6 != ""): self.combo_signal_device.addItem(self.hidd_number_6)
            if (self.hidd_number_7 != ""): self.combo_signal_device.addItem(self.hidd_number_7)
            if (self.hidd_number_8 != ""): self.combo_signal_device.addItem(self.hidd_number_8)

    def func_Refresh_hidraw(self):
        self.combo_hidraw.clear()
        self.listofhidraws = getoutput("ls /dev | awk '{printf\"/dev/\"$1\"\\n\"}' | grep hidraw").split()
        if (len(self.listofhidraws) == 0):
            self.combo_hidraw.addItem(self.tr("No hidraw devices found"))
            self.combo_hidraw.setEnabled(0)
            self.b_apply_hidraw.setEnabled(0)
        else:
            self.combo_hidraw.setEnabled(1)
            self.b_apply_hidraw.setEnabled(1)
            j = 0
            while j < len(self.listofhidraws):
                self.combo_hidraw.addItem(self.listofhidraws[j])
                j += 1

    def func_Kill_sixadraw(self):
        if look4Root(self):
            os.system(shared.ROOT+" pkill -TERM sixad-raw > /dev/null")
            QMessageBox.information(self, self.tr("QtSixA - Information"), self.tr(""
            "All back to normal now"))

#Advanced  ### -----------------------------------------------------------------------------------------------


#Systray Stuff  ### ------------------------------------------------------------------------------------------
    def createTrayIcon(self):
        self.actionShowSixA = QAction(self.tr("Show/Hide Main &Window"), self)
        self.connect(self.actionShowSixA, SIGNAL("triggered()"), self.func_Show_SixA)
        self.actionCloseSystray = QAction(QIcon('/usr/share/qtsixa/icons/close.png'), self.tr("&Close Systray"), self)
        self.connect(self.actionCloseSystray, SIGNAL("triggered()"), self.func_CloseSystray)

        self.trayIconMenu = QMenu(self)
        self.trayIconMenu.addAction(self.act_About)
        self.trayIconMenu.addAction(self.act_Configure)
        self.trayIconMenu.addAction(self.act_ManageProf)
        self.trayIconMenu.addSeparator()
        self.trayIconMenu.addAction(self.actionShowSixA)
        self.trayIconMenu.addSeparator()
        self.trayIconMenu.addMenu(self.menu_Disc)
        self.trayIconMenu.addAction(self.act_Pair)
        self.trayIconMenu.addMenu(self.menu_Root)
        self.trayIconMenu.addSeparator()
        self.trayIconMenu.addAction(self.actionCloseSystray)
        #self.trayIconMenu.addAction(self.act_Exit)

        self.trayIcon = QSystemTrayIcon(self)
        self.trayIcon.setContextMenu(self.trayIconMenu)
        try:
          self.trayIcon.activated.connect(self.func_Systray_Clicked) #Not available on some systems ( why ? )
        except:
          print self.tr("Your system doesn't suport double-click on systray")

        self.trayIcon.setToolTip(self.trayTooltip)
        self.trayIcon.setIcon(QIcon('/usr/share/qtsixa/icons/qtsixa_32.png'))
        self.trayIcon.show()

    def func_Systray_Clicked(self, reason):
        if (reason == 2 or reason == 3 or reason == 4): self.func_Show_SixA()

    def func_Show_SixA(self):
        if self.isHidden(): self.show()
        else: self.hide()

    def func_CloseSystray(self):
        self.trayIcon.hide()
        self.trayIsActive = 0
        if self.isHidden(): sys.exit(0)
        else: shared.app.setQuitOnLastWindowClosed(True)

    def func_UpdateTrayTooltip(self):
        self.trayTooltip = "<b> QtSixA 1.5.1 </b><br>"
        self.trayTooltip += "<p>"

        if (self.usb_number_1 == "") and (self.hidd_number_1 == ""):
            self.trayTooltip += self.tr("No devices found")
        else:
            self.trayTooltip += self.tr("<u>Connected devices:</u>")
        if (self.hidd_number_1 != ""):
            self.trayTooltip += "<br>"+self.listOfDevices.item(1).text()
            if (self.hidd_number_2 != ""):
                self.trayTooltip += "<br>"+self.listOfDevices.item(2).text()
                if (self.hidd_number_3 != ""):
                    self.trayTooltip += "<br>"+self.listOfDevices.item(3).text()
                    if (self.hidd_number_4 != ""):
                        self.trayTooltip += "<br>"+self.listOfDevices.item(4).text()
                        if (self.hidd_number_5 != ""):
                            self.trayTooltip += "<br>"+self.listOfDevices.item(5).text()
                            if (self.hidd_number_6 != ""):
                                self.trayTooltip += "<br>"+self.listOfDevices.item(6).text()
                                if (self.hidd_number_7 != ""):
                                    self.trayTooltip += "<br>"+self.listOfDevices.item(7).text()
                                    if (self.hidd_number_8 != ""):
                                        self.trayTooltip += "<br>"+self.listOfDevices.item(8).text()
        if (self.usb_number_1 != ""):
            self.trayTooltip += "<br>"+self.listOfDevices.item(9).text()+" (USB)"
            if (self.usb_number_2 != ""):
                self.trayTooltip += "<br>"+self.listOfDevices.item(10).text()+" (USB)"
                if (self.usb_number_3 != ""):
                    self.trayTooltip += "<br>"+self.listOfDevices.item(11).text()+" (USB)"
                    if (self.usb_number_4 != ""):
                        self.trayTooltip += "<br>"+self.listOfDevices.item(12).text()+" (USB)"
        if self.trayIsActive == 1: self.trayIcon.setToolTip(self.trayTooltip)

#Systray Stuff  ### ------------------------------------------------------------------------------------------

