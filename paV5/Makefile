CC=g++ -Wall -O2
#`pkg-config --cflags opencv` generates this: -I/usr/local/include/opencv
#`pkg-config --libs opencv` generates this: -L/usr/local/lib -lcxcore -lcv -lhighgui -lcvaux

SRC=pamain.c
PROG=pamain

PROJNAME=pistAnal

INCLUDE2 = -I. -I./include \
	   -I/usr/lib/qt3/mkspecs/default \
	   -I/usr/local/include/libraw1394 \
	   -I/usr/local/include/libdc1394 \
	   -I/usr/local/include/opencv \
	   -I/usr/include \
	   -I$(QTDIR)/include \
	   -I/usr/X11R6/include\
	   -I../libimageaquisition/include \
	   -I../../ATLASIV/libimageaquisition/src # alternative for vs to previous line...

LDPATH   = -L./lib  \
	   -L/usr/local/lib \
	   -L$(QTDIR)/lib \
	   -L/usr/X11R6/lib\
	   -L/home/mike/progs/ATLASIV/libimageaquisition/lib\
	   -L../../ATLASIV/libimageaquisition/lib # alternative for vs for previous line \
	   #-L/home/mike/progs/ATLASIV/oCV5/lib # Not needed since it being compiled also now

LIBS     = -lpa -limageaquisition -lqt-mt -lXext -lX11 -lm -lpthread

LIBSRC=pa.c
LIBOBJ=pa.o
LIBRARY=./lib/libpa.a

LIBVER=V5

DEFINES=-DUSECAMERAS=0   #Make 1 to compile using cameras


#=================================================================================
.c.o:
	$(CC) -c `pkg-config --cflags opencv` $(INCLUDE2) $(DEFINES) -o $@ $<
#=================================================================================

$(PROG): $(LIBRARY) $(SRC)
	$(CC) `pkg-config --cflags opencv` $(INCLUDE2) $(DEFINES) -o $(PROG) $(SRC) `pkg-config --libs opencv` $(LDPATH) $(LIBS)

lib $(LIBRARY): $(LIBOBJ)
	ar rcs $(LIBRARY) $(LIBOBJ)

#=================================================================================
depend:
	makedepend `pkg-config --cflags opencv` $(INCLUDE2) $(DEFINES) $(SRC) $(LIBSRC)

clean:
	rm -f *.o *.out *~ *.bak *.a $(PROG) $(LIBRARY)

doc:
	@ doxygen Doxyfile >& /dev/null
	@ rm -rf latex
	#@ firefox ./html/index.html

pack:	$(LIBRARY) doc
	tar cf pa-$(LIBVER).tar ./html ./include ./lib
	rm -f pa-$(LIBVER).tar.gz
	gzip -9q pa-$(LIBVER).tar

toMiguel:
	cd .. ; tar cf pa-$(LIBVER).tar  pa$(LIBVER) ; rm -f pa-$(LIBVER).tar.gz ; gzip -9q pa-$(LIBVER).tar

mail: toMiguel
	@ cd .. ; \
	echo -e "Versao $(LIBVER) do $(PROJNAME) em anexo.\n\
      	--Vitor santos \n\
	NB. Este E-mail foi gerado e enviado automaticamente\n\
	    O autor declina responsabilidades por ma' formatacao :-)"\
	|  mail -s "Versao $(LIBVER) do $(PROJNAME)" -a pa-$(LIBVER).tar.gz -c vsantos@mec.ua.pt mriem@mec.ua.pt

# DO NOT DELETE

pamain.o: /usr/local/include/opencv/cv.h /usr/local/include/opencv/cxcore.h
pamain.o: /usr/local/include/opencv/cxtypes.h /usr/include/assert.h
pamain.o: /usr/include/features.h /usr/include/sys/cdefs.h
pamain.o: /usr/include/gnu/stubs.h /usr/include/stdlib.h
pamain.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stddef.h
pamain.o: /usr/include/string.h
pamain.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/float.h
pamain.o: /usr/include/math.h /usr/include/bits/huge_val.h
pamain.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
pamain.o: /usr/local/include/opencv/cxerror.h
pamain.o: /usr/local/include/opencv/cvtypes.h
pamain.o: /usr/local/include/opencv/cvcompat.h
pamain.o: /usr/local/include/opencv/highgui.h ./include/pa.h
pamain.o: /usr/include/stdio.h /usr/include/bits/types.h
pamain.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
pamain.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
pamain.o: /usr/include/bits/wchar.h /usr/include/gconv.h
pamain.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stdarg.h
pamain.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
pa.o: /usr/local/include/opencv/cv.h /usr/local/include/opencv/cxcore.h
pa.o: /usr/local/include/opencv/cxtypes.h /usr/include/assert.h
pa.o: /usr/include/features.h /usr/include/sys/cdefs.h
pa.o: /usr/include/gnu/stubs.h /usr/include/stdlib.h
pa.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stddef.h
pa.o: /usr/include/string.h
pa.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/float.h
pa.o: /usr/include/math.h /usr/include/bits/huge_val.h
pa.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
pa.o: /usr/local/include/opencv/cxerror.h /usr/local/include/opencv/cvtypes.h
pa.o: /usr/local/include/opencv/cvcompat.h ./include/pa.h
pa.o: /usr/include/stdio.h /usr/include/bits/types.h
pa.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
pa.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
pa.o: /usr/include/bits/wchar.h /usr/include/gconv.h
pa.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stdarg.h
pa.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
