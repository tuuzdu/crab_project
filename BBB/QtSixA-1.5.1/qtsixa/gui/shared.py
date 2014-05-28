#! /usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
import os
from PyQt4.QtCore import QFile, QIODevice, QTextStream
from PyQt4.QtXml import QDomDocument


class Globals(object):
    __slots__ = [
            'show_warnings',
            'only_one_instance',
            'systray_enabled',
            'start_minimized',
            'close_to_tray',
            'ROOT',
            'app',
            'gui',
            'editDev',
            'editProfile',
    ]
Globals = Globals()


def init_config(self):
    if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/"):
      os.mkdir(os.getenv("HOME")+"/.qtsixa2/")
    if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/profiles"):
      os.mkdir(os.getenv("HOME")+"/.qtsixa2/profiles")

    if not os.path.exists(os.getenv("HOME")+"/.qtsixa2/conf.xml"):

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
                    "     <Show-Warnings>true</Show-Warnings>\n"
                    "     <Only-One-Instance>true</Only-One-Instance>\n"
                    "   </Main>\n"
                    "   <Systray>\n"
                    "     <Enable>true</Enable>\n"
                    "     <Start-Minimized>false</Start-Minimized>\n"
                    "     <Close-to-Tray>false</Close-to-Tray>\n"
                    "   </Systray>\n"
                    " </Configuration>\n"
                    "</QTSIXA>\n"
                    )
        Globals.show_warnings = True
        Globals.only_one_instance = True
        Globals.systray_enabled = True
        Globals.start_minimized = False
        Globals.close_to_tray   = False
    else:

        # Set Default Values, only change them if the config says so
        Globals.show_warnings = True
        Globals.only_one_instance = True
        Globals.systray_enabled = True
        Globals.start_minimized = False
        Globals.close_to_tray   = False

        # Read configuration file
        xml = QDomDocument()
        filename = QFile(os.getenv("HOME")+"/.qtsixa2/conf.xml")
        if not filename.open(QIODevice.ReadOnly):
          print "error here, 1"
        if not xml.setContent(filename):
          print "error here, 2"
        filename.close()

        # Check if project file is not corrupted
        content = xml.documentElement()
        if (content.tagName() != "QTSIXA"):
          print "error here, 3"

        # Get values from XML - the big code
        node = content.firstChild()
        while not node.isNull():
          if (node.toElement().tagName() == "Configuration"):
            configuration = node.toElement().firstChild()
            while not configuration.isNull():
              conf_tag = configuration.toElement().tagName()
              if (conf_tag == "Main"):
                conf_tag_main = configuration.toElement().firstChild()
                while not conf_tag_main.isNull():
                  name = conf_tag_main.toElement().tagName()
                  text = conf_tag_main.toElement().text()
                  if (name == "Show-Warnings"):
                    if (text == "0" or text == "false" or text == "False"):
                      Globals.show_warnings = False
                  elif (name == "Only-One-Instance"):
                    if (text == "0" or text == "false" or text == "False"):
                      Globals.only_one_instance = False
                  conf_tag_main = conf_tag_main.nextSibling()
              elif (conf_tag == "Systray"):
                conf_tag_systray = configuration.toElement().firstChild()
                while not conf_tag_systray.isNull():
                  name = conf_tag_systray.toElement().tagName()
                  text = conf_tag_systray.toElement().text()
                  if (name == "Enable"):
                    if (text == "0" or text == "false" or text == "False"):
                      Globals.systray_enabled = False
                  elif (name == "Start-Minimized"):
                    if (text == "1" or text == "true" or text == "True"):
                      Globals.start_minimized = True
                  elif (name == "Close-to-Tray"):
                    if (text == "1" or text == "true" or text == "True"):
                      Globals.close_to_tray = True
                  conf_tag_systray = conf_tag_systray.nextSibling()
              configuration = configuration.nextSibling()
          node = node.nextSibling()


def func_checkFileOption(file_t, option_t):
    if (os.path.exists("/var/lib/sixad/profiles/"+file_t)):
      r = open("/var/lib/sixad/profiles/"+file_t, "r").read()
      return r.split(option_t)[1].split()[0]
    elif (file_t == "default"):
      if (option_t == "enable_leds"):
        return "1"
      elif (option_t == "led_n_auto"):
        return "1"
      elif (option_t == "enable_joystick"):
        return "1"
      elif (option_t == "enable_input"):
        return "0"
      elif (option_t == "enable_rumble"):
        return "1"
    elif (file_t == "hidraw"):
      if (option_t == "enable_leds"):
        return "0"
      elif (option_t == "led_n_auto"):
        return "1"
      elif (option_t == "enable_joystick"):
        return "1"
      elif (option_t == "enable_input"):
        return "0"
      elif (option_t == "enable_rumble"):
        return "0"
    else:
      return "0"
    #else:
      #raise IOError, unicode(r.errorString())

def func_checkDeviceOptions(dev):
    str_led = ""
    str_js  = ""
    str_in  = ""
    str_rum = ""

    if (not os.path.exists("/var/lib/sixad/profiles/"+dev)):
      if (dev != "hidraw"):
        dev = "default"

    if (int(func_checkFileOption(dev, "enable_leds"))):
      if (int(func_checkFileOption(dev, "led_n_auto"))):
        str_led = "Yes, Auto"
      else:
        str_led = "Yes, "+func_checkFileOption(dev, "led_n_number")
    else:
      str_led = "No"

    if (int(func_checkFileOption(dev, "enable_joystick"))):
      str_js = "Yes"
    else:
      str_js = "No"

    if (int(func_checkFileOption(dev, "enable_input"))):
      str_in = "Yes"
    else:
      str_in = "No"

    if (int(func_checkFileOption(dev, "enable_rumble"))):
      str_rum = "Yes"
    else:
      str_rum = "No"

    return ( str_led, str_js, str_in, str_rum)
