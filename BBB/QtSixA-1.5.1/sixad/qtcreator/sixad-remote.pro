# QtCreator project file

CONFIG = qt
QT -= gui

TEMPLATE = app

SOURCES = ../sixad-remote.cpp ../remote.cpp ../shared.cpp ../uinput.cpp ../textfile.cpp

HEADERS = ../remote.h ../shared.h ../uinput.h ../textfile.h

TARGET = sixad-remote

LIBS += -lrt
