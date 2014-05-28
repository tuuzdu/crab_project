#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Imports
from PyQt4.QtGui import QDialog, QIcon
import ui_qtsixa_referencew


class ReferenceW(QDialog, ui_qtsixa_referencew.Ui_ReferenceW):
    def __init__(self, *args):
        QDialog.__init__(self, *args)
        self.setupUi(self)
        self.setWindowIcon(QIcon(":/icons/qtsixa.png"))
