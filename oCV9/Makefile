CC=g++ -Wall -O2
#`pkg-config --cflags opencv` generates this: -I/usr/local/include/opencv
#`pkg-config --libs opencv` generates this: -L/usr/local/lib -lcxcore -lcv -lhighgui -lcvaux

SRC=jmain.c
PROG=jmain

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

LIBS     = -ljoinimg -limageaquisition -lqt-mt -lXext -lX11 -lm -lpthread

LIBSRC=joinimg.c
LIBOBJ=joinimg.o
LIBRARY=./lib/libjoinimg.a

LIBVER=V9

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
	rm -f *.o *.out *~ *.bak *.a $(PROG)

doc:
	@ doxygen Doxyfile >& /dev/null
	@ rm -rf latex
	#@ firefox ./html/index.html

pack:	$(LIBRARY) doc
	tar cf joinimg-$(LIBVER).tar ./html ./include ./lib
	rm -f joinimg-$(LIBVER).tar.gz
	gzip -9q joinimg-$(LIBVER).tar

toMiguel: doc
	cd .. ; tar cf joinimg-$(LIBVER).tar  oC$(LIBVER) ; rm -f joinimg-$(LIBVER).tar.gz ; gzip -9q joinimg-$(LIBVER).tar

mail: toMiguel
	@ cd .. ; \
	echo -e "Versao $(LIBVER) do $(PROJNAME) em anexo.\n\
	--Vitor santos \n\
	NB. Este E-mail foi gerado e enviado automaticamente\n\
	O autor declina responsabilidades por ma' formatacao :-)"\
	|  mail -s "Versao $(LIBVER) do $(PROJNAME)" -a joinimg-$(LIBVER).tar.gz -c vsantos@mec.ua.pt mriem@mec.ua.pt


# DO NOT DELETE

jmain.o: /usr/local/include/opencv/cv.h /usr/local/include/opencv/cxcore.h
jmain.o: /usr/local/include/opencv/cxtypes.h /usr/include/assert.h
jmain.o: /usr/include/features.h /usr/include/sys/cdefs.h
jmain.o: /usr/include/gnu/stubs.h /usr/include/stdlib.h
jmain.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stddef.h
jmain.o: /usr/include/string.h
jmain.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/float.h
jmain.o: /usr/include/math.h /usr/include/bits/huge_val.h
jmain.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
jmain.o: /usr/local/include/opencv/cxerror.h
jmain.o: /usr/local/include/opencv/cvtypes.h
jmain.o: /usr/local/include/opencv/cvcompat.h
jmain.o: /usr/local/include/opencv/highgui.h ./include/joinimg.h
jmain.o: /usr/include/stdio.h /usr/include/bits/types.h
jmain.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
jmain.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
jmain.o: /usr/include/bits/wchar.h /usr/include/gconv.h
jmain.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stdarg.h
jmain.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
joinimg.o: /usr/local/include/opencv/cv.h /usr/local/include/opencv/cxcore.h
joinimg.o: /usr/local/include/opencv/cxtypes.h /usr/include/assert.h
joinimg.o: /usr/include/features.h /usr/include/sys/cdefs.h
joinimg.o: /usr/include/gnu/stubs.h /usr/include/stdlib.h
joinimg.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stddef.h
joinimg.o: /usr/include/string.h
joinimg.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/float.h
joinimg.o: /usr/include/math.h /usr/include/bits/huge_val.h
joinimg.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
joinimg.o: /usr/local/include/opencv/cxerror.h
joinimg.o: /usr/local/include/opencv/cvtypes.h
joinimg.o: /usr/local/include/opencv/cvcompat.h ./include/joinimg.h
joinimg.o: /usr/include/stdio.h /usr/include/bits/types.h
joinimg.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
joinimg.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
joinimg.o: /usr/include/bits/wchar.h /usr/include/gconv.h
joinimg.o: /usr/lib/gcc-lib/i586-suse-linux/3.3.4/include/stdarg.h
joinimg.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
