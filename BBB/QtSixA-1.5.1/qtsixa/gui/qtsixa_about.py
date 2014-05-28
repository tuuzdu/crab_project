#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
from PyQt4.QtGui import QDialog, QIcon
import ui_qtsixa_aboutw


class AboutW(QDialog, ui_qtsixa_aboutw.Ui_AboutW):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))


