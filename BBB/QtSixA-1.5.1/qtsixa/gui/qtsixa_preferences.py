#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import os
from PyQt4.QtCore import QFile, QIODevice, QTextStream, SIGNAL
from PyQt4.QtGui import QDialog, QDialogButtonBox, QIcon, QMessageBox
import shared, ui_qtsixa_preferencesw


class PreferencesW(QDialog, ui_qtsixa_preferencesw.Ui_PreferencesW):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))

        self.box_warn.setChecked(shared.Globals.show_warnings)
        self.box_1inst.setChecked(shared.Globals.only_one_instance)
        self.box_systray.setChecked(shared.Globals.systray_enabled)
        self.box_min.setChecked(shared.Globals.start_minimized)
        self.box_close.setChecked(shared.Globals.close_to_tray)

        self.buttonBox.button(QDialogButtonBox.Apply).setEnabled(False)

        if not os.path.exists("/usr/bin/notify-send"):
            self.prefTab.removeTab(1)

        # Notification stuff
        if (os.path.exists("/tmp/.sixad-notify")):
            self.box_notify.setChecked(True)
            self.box_notify_start.setEnabled(True)
            self.alreadyStartedNotify = True
        else:
            self.alreadyStartedNotify = False
            if (os.path.exists(os.getenv("HOME")+"/.config/autostart/sixad-notify.desktop")):
                self.box_notify.setChecked(True)
                self.box_notify_start.setChecked(True)
            else:
                self.box_notify.setChecked(False)
                self.box_notify_start.setChecked(False)

        if (os.path.exists(os.getenv("HOME")+"/.config/autostart/qtsixa.desktop")):
            self.box_startup.setChecked(True)
        else:
            self.box_startup.setChecked(False)


        self.connect(self.box_startup, SIGNAL('clicked()'), self.func_Apply_Enable)
        self.connect(self.box_warn, SIGNAL('clicked()'), self.func_Apply_Enable)
        self.connect(self.box_1inst, SIGNAL('clicked()'), self.func_Apply_Enable)
        self.connect(self.box_systray, SIGNAL('clicked()'), self.func_Apply_Enable)
        self.connect(self.box_min, SIGNAL('clicked()'), self.func_Apply_Enable)
        self.connect(self.box_close, SIGNAL('clicked()'), self.func_Apply_Enable)
        self.connect(self.box_notify, SIGNAL("clicked()"), self.func_Apply_Enable)
        self.connect(self.box_notify_start, SIGNAL("clicked()"), self.func_Apply_Enable)
        self.connect(self.buttonBox, SIGNAL("accepted()"), self.func_Apply)
        self.connect(self.buttonBox.button(QDialogButtonBox.Apply), SIGNAL("clicked()"), self.func_Apply)


    def func_Apply_Enable(self):
      self.buttonBox.button(QDialogButtonBox.Apply).setEnabled(True)


    def func_Apply(self):
        os.system("mkdir -p $HOME/.config/autostart/")

        if self.box_notify.isChecked():
            if (not self.alreadyStartedNotify):
                os.system("rm -rf "+"/tmp/.sixad-notify")
                os.system("sixad-notify &")
            if self.box_notify_start.isChecked():
                os.system("cp /usr/share/qtsixa/sixad-notify.desktop $HOME/.config/autostart/sixad-notify.desktop")
            else:
                os.system("rm -rf $HOME/.config/autostart/sixad-notify.desktop")
            self.alreadyStartedNotify = True
        else:
            os.system("rm -rf /tmp/.sixad-notify")
            os.system("rm -rf $HOME/.config/autostart/sixad-notify.desktop")
            self.alreadyStartedNotify = False

        if self.box_startup.isChecked():
            os.system("cp /usr/share/applications/qtsixa.desktop $HOME/.config/autostart/qtsixa.desktop")
        else:
            os.system("rm -rf $HOME/.config/autostart/qtsixa.desktop")

        self.func_Save_config()

        self.buttonBox.button(QDialogButtonBox.Apply).setEnabled(False)


    def func_Save_config(self):
        if (self.box_warn.isChecked()): conf_warn = "true"
        else: conf_warn = "false"
        if (self.box_1inst.isChecked()): conf_1inst = "true"
        else: conf_1inst = "false"
        if (self.box_systray.isChecked()): conf_systray = "true"
        else: conf_systray = "false"
        if (self.box_min.isChecked()): conf_min = "true"
        else: conf_min = "false"
        if (self.box_close.isChecked()): conf_close = "true"
        else: conf_close = "false"

        filename = QFile(os.getenv("HOME")+"/.qtsixa2/conf.xml")
        if not filename.open(QIODevice.WriteOnly):
          QMessageBox.critical(self, self.tr("QtSixA - Error"), self.tr("Cannot write QtSixA configuration file!"))
          raise IOError, unicode(filename.errorString())
        stream = QTextStream(filename)
        stream.setCodec("UTF-8")
        stream << ("<?xml version='1.0' encoding='UTF-8'?>\n"
                    "<!DOCTYPE QTSIXA>\n"
                    "<QTSIXA VERSION='1.5.1'>\n"
                    " <Configuration>\n"
                    "   <Main>\n"
                    "     <Show-Warnings>%s</Show-Warnings>\n"
                    "     <Only-One-Instance>%s</Only-One-Instance>\n"
                    "   </Main>\n"
                    "   <Systray>\n"
                    "     <Enable>%s</Enable>\n"
                    "     <Start-Minimized>%s</Start-Minimized>\n"
                    "     <Close-to-Tray>%s</Close-to-Tray>\n"
                    "   </Systray>\n"
                    " </Configuration>\n"
                    "</QTSIXA>\n" % ( conf_warn, conf_1inst, conf_systray, conf_min, conf_close )
                    )
        shared.Globals.show_warnings = self.box_warn.isChecked()
        shared.Globals.only_one_instance = self.box_1inst.isChecked()
        shared.Globals.systray_enabled = self.box_systray.isChecked()
        shared.Globals.start_minimized = self.box_min.isChecked()
        shared.Globals.close_to_tray   = self.box_close.isChecked()

