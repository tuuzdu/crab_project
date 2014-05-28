# You know, there are pre-compile DEBs of this...

DESTDIR =


build:
	pyuic4 -o ./gui/ui_qtsixa_mainw.py ./gui/ui/qtsixa_mainw.ui
	pyuic4 -o ./gui/ui_qtsixa_aboutw.py ./gui/ui/qtsixa_aboutw.ui
	pyuic4 -o ./gui/ui_qtsixa_managew.py ./gui/ui/qtsixa_managew.ui
	pyuic4 -o ./gui/ui_qtsixa_newdevw.py ./gui/ui/qtsixa_newdevw.ui
	pyuic4 -o ./gui/ui_qtsixa_newprofilew.py ./gui/ui/qtsixa_newprofilew.ui
	pyuic4 -o ./gui/ui_qtsixa_preferencesw.py ./gui/ui/qtsixa_preferencesw.ui
	pyuic4 -o ./gui/ui_qtsixa_referencew.py ./gui/ui/qtsixa_referencew.ui
	pyuic4 -o ./gui/ui_qtsixa_sixpairw.py ./gui/ui/qtsixa_sixpairw.ui
	pyrcc4 -o ./gui/qtsixa_rc.py ./icons/qtsixa.qrc

clean:
	rm -f *~ gui/*~ gui/*.pyc gui/ui_*.py gui/qtsixa_rc.py

install:
	#Make directories
	install -d $(DESTDIR)/usr/bin/
	install -d $(DESTDIR)/usr/share/applications/
	install -d $(DESTDIR)/usr/share/pixmaps/
# 	install -d $(DESTDIR)/usr/share/doc/qtsixa/
# 	install -d $(DESTDIR)/usr/share/doc/qtsixa/manual/
	install -d $(DESTDIR)/usr/share/qtsixa/
	install -d $(DESTDIR)/usr/share/qtsixa/game-profiles/
	install -d $(DESTDIR)/usr/share/qtsixa/gui/
	install -d $(DESTDIR)/usr/share/qtsixa/icons/
	install -d $(DESTDIR)/usr/share/qtsixa/pics/
	install -d $(DESTDIR)/usr/share/qtsixa/profiles/
#	install -d $(DESTDIR)/usr/share/qtsixa/lang/
	#Install files
	install -m 655 qtsixa $(DESTDIR)/usr/bin/
	install -m 655 sixad-lq $(DESTDIR)/usr/bin/
	install -m 655 sixad-notify $(DESTDIR)/usr/bin/
# 	install -m 644 manual/* $(DESTDIR)/usr/share/doc/qtsixa/manual/
	install -m 644 game-profiles/* $(DESTDIR)/usr/share/qtsixa/game-profiles/
	install -m 644 gui/*.py $(DESTDIR)/usr/share/qtsixa/gui/
	install -m 644 icons/* $(DESTDIR)/usr/share/qtsixa/icons/
	install -m 644 pics/* $(DESTDIR)/usr/share/qtsixa/pics/
	install -m 644 profiles/* $(DESTDIR)/usr/share/qtsixa/profiles/
#	install -m 644 lang/* $(DESTDIR)/usr/share/qtsixa/lang/
	install -m 644 sixad-notify.desktop $(DESTDIR)/usr/share/qtsixa/
	install -m 644 qtsixa.desktop $(DESTDIR)/usr/share/applications/
	install -m 644 qtsixa.xpm $(DESTDIR)/usr/share/pixmaps/

uninstall:
	rm -rf $(DESTDIR)/usr/share/doc/qtsixa/
	rm -rf $(DESTDIR)/usr/share/qtsixa/
	rm -rf $(DESTDIR)/usr/bin/sixad-lq
	rm -rf $(DESTDIR)/usr/bin/sixad-notify
	rm -rf $(DESTDIR)/usr/bin/qtsixa
	rm -rf $(DESTDIR)/usr/share/applications/qtsixa.desktop
	rm -rf $(DESTDIR)/usr/share/pixmaps/qtsixa.xpm

