# QtCreator project file

CONFIG = qt
QT -= gui

TEMPLATE = app

SOURCES = ../sixad-bin.cpp ../bluetooth.cpp ../shared.cpp ../textfile.cpp

HEADERS = ../bluetooth.h ../shared.h ../textfile.h

TARGET = sixad-bin

LIBS += -lbluetooth -lpthread
