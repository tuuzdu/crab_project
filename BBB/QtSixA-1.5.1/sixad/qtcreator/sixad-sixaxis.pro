# QtCreator project file

CONFIG = qt
QT -= gui

TEMPLATE = app

SOURCES = ../sixad-sixaxis.cpp ../sixaxis.cpp ../shared.cpp ../uinput.cpp ../textfile.cpp

HEADERS = ../sixaxis.h ../shared.h ../uinput.h ../textfile.h

TARGET = sixad-sixaxis

LIBS += -lpthread -lrt
