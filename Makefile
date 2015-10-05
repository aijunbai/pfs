#############################################################################
# Makefile for building: pfs
# Generated by qmake (2.01a) (Qt 4.8.1) on: Sun Oct 4 20:54:55 2015
# Project:  pfs.pro
# Template: app
# Command: /opt/bin/qmake -o Makefile pfs.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_WEBKIT -DQT_NO_DEBUG -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -m64 -pipe -O2 -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -m64 -pipe -msse4.1 -O2 -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I. -Isrc -Isrc/math -Isrc/util -I.
LINK          = g++
LFLAGS        = -m64 -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -lboost_program_options -lxml2 -lQtGui -lQtCore -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /opt/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = src/action.cpp \
		src/benchmark.cpp \
		src/cameraModel.cpp \
		src/combination.cpp \
		src/elevator_entering.cpp \
		src/gaussian.cpp \
		src/input_stream.cpp \
		src/intention.cpp \
		src/logger.cpp \
		src/main.cpp \
		src/murty.cpp \
		src/observation.cpp \
		src/open_area.cpp \
		src/params.cpp \
		src/particle.cpp \
		src/simulator.cpp \
		src/state.cpp \
		src/task.cpp \
		src/testCalibration.cpp \
		src/tracker.cpp \
		src/xmlUtil.cpp \
		src/math/eig3.cpp \
		src/math/gvector.cpp \
		src/util/helpers.cpp \
		src/util/proghelp.cc \
		src/util/pthread_utils.cpp \
		src/util/terminal_utils.cpp \
		src/util/timer.cc \
		src/util/watch_files.cpp moc_benchmark.cpp
OBJECTS       = action.o \
		benchmark.o \
		cameraModel.o \
		combination.o \
		elevator_entering.o \
		gaussian.o \
		input_stream.o \
		intention.o \
		logger.o \
		main.o \
		murty.o \
		observation.o \
		open_area.o \
		params.o \
		particle.o \
		simulator.o \
		state.o \
		task.o \
		testCalibration.o \
		tracker.o \
		xmlUtil.o \
		eig3.o \
		gvector.o \
		helpers.o \
		proghelp.o \
		pthread_utils.o \
		terminal_utils.o \
		timer.o \
		watch_files.o \
		moc_benchmark.o
DIST          = /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		pfs.pro
QMAKE_TARGET  = pfs
DESTDIR       = 
TARGET        = pfs

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: pfs.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		/usr/lib/x86_64-linux-gnu/libQtGui.prl \
		/usr/lib/x86_64-linux-gnu/libQtCore.prl
	$(QMAKE) -o Makefile pfs.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
/usr/lib/x86_64-linux-gnu/libQtGui.prl:
/usr/lib/x86_64-linux-gnu/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -o Makefile pfs.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/pfs1.0.0 || $(MKDIR) .tmp/pfs1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/pfs1.0.0/ && $(COPY_FILE) --parents src/action.h src/benchmark.h src/cameraModel.h src/combination.h src/common.h src/elevator_entering.h src/gaussian.h src/input_stream.h src/intention.h src/logger.h src/murty.h src/observation.h src/open_area.h src/params.h src/particle.h src/simulator.h src/state.h src/statistic.h src/task.h src/testCalibration.h src/tracker.h src/xmlUtil.h src/math/eig3.h src/math/eigen_helper.h src/math/geomalgo.h src/math/geometry.h src/math/gvector.h src/math/line.h src/math/quaternion_helper.h src/math/triangle.h src/math/util.h src/util/ansicolor.h src/util/extensions.h src/util/helpers.h src/util/popt_pp.h src/util/proghelp.h src/util/pthread_utils.h src/util/sstring.h src/util/terminal_utils.h src/util/timer.h src/util/watch_files.h .tmp/pfs1.0.0/ && $(COPY_FILE) --parents src/action.cpp src/benchmark.cpp src/cameraModel.cpp src/combination.cpp src/elevator_entering.cpp src/gaussian.cpp src/input_stream.cpp src/intention.cpp src/logger.cpp src/main.cpp src/murty.cpp src/observation.cpp src/open_area.cpp src/params.cpp src/particle.cpp src/simulator.cpp src/state.cpp src/task.cpp src/testCalibration.cpp src/tracker.cpp src/xmlUtil.cpp src/math/eig3.cpp src/math/gvector.cpp src/util/helpers.cpp src/util/proghelp.cc src/util/pthread_utils.cpp src/util/terminal_utils.cpp src/util/timer.cc src/util/watch_files.cpp .tmp/pfs1.0.0/ && (cd `dirname .tmp/pfs1.0.0` && $(TAR) pfs1.0.0.tar pfs1.0.0 && $(COMPRESS) pfs1.0.0.tar) && $(MOVE) `dirname .tmp/pfs1.0.0`/pfs1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/pfs1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: moc_benchmark.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) moc_benchmark.cpp
moc_benchmark.cpp: src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h \
		src/simulator.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/state.h \
		src/tracker.h \
		src/particle.h \
		src/cameraModel.h \
		src/benchmark.h
	/usr/lib/x86_64-linux-gnu/qt4/bin/moc $(DEFINES) $(INCPATH) src/benchmark.h -o moc_benchmark.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean 

####### Compile

action.o: src/action.cpp src/action.h \
		src/params.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h \
		src/tracker.h \
		src/particle.h \
		src/state.h \
		src/intention.h \
		src/observation.h \
		src/combination.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o action.o src/action.cpp

benchmark.o: src/benchmark.cpp src/benchmark.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h \
		src/simulator.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/state.h \
		src/tracker.h \
		src/particle.h \
		src/cameraModel.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o benchmark.o src/benchmark.cpp

cameraModel.o: src/cameraModel.cpp src/cameraModel.h \
		src/xmlUtil.h \
		src/params.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o cameraModel.o src/cameraModel.cpp

combination.o: src/combination.cpp src/combination.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o combination.o src/combination.cpp

elevator_entering.o: src/elevator_entering.cpp src/elevator_entering.h \
		src/task.h \
		src/logger.h \
		src/intention.h \
		src/action.h \
		src/params.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/simulator.h \
		src/observation.h \
		src/combination.h \
		src/state.h \
		src/tracker.h \
		src/particle.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o elevator_entering.o src/elevator_entering.cpp

gaussian.o: src/gaussian.cpp src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o gaussian.o src/gaussian.cpp

input_stream.o: src/input_stream.cpp src/input_stream.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/observation.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h \
		src/combination.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o input_stream.o src/input_stream.cpp

intention.o: src/intention.cpp src/intention.h \
		src/action.h \
		src/params.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o intention.o src/intention.cpp

logger.o: src/logger.cpp src/logger.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/util/terminal_utils.h \
		src/tracker.h \
		src/particle.h \
		src/state.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o logger.o src/logger.cpp

main.o: src/main.cpp src/tracker.h \
		src/particle.h \
		src/state.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/logger.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/task.h \
		src/simulator.h \
		src/elevator_entering.h \
		src/murty.h \
		src/util/popt_pp.h \
		src/util/terminal_utils.h \
		src/util/timer.h \
		src/input_stream.h \
		src/util/helpers.h \
		src/testCalibration.h \
		src/benchmark.h \
		src/cameraModel.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o src/main.cpp

murty.o: src/murty.cpp src/tracker.h \
		src/particle.h \
		src/state.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/logger.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/util/terminal_utils.h \
		src/murty.h \
		src/simulator.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o murty.o src/murty.cpp

observation.o: src/observation.cpp src/observation.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h \
		src/xmlUtil.h \
		src/combination.h \
		src/particle.h \
		src/state.h \
		src/intention.h \
		src/action.h \
		src/tracker.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o observation.o src/observation.cpp

open_area.o: src/open_area.cpp src/open_area.h \
		src/task.h \
		src/logger.h \
		src/intention.h \
		src/action.h \
		src/params.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/simulator.h \
		src/observation.h \
		src/combination.h \
		src/state.h \
		src/tracker.h \
		src/particle.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o open_area.o src/open_area.cpp

params.o: src/params.cpp src/params.h \
		src/logger.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o params.o src/params.cpp

particle.o: src/particle.cpp src/particle.h \
		src/state.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/logger.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/tracker.h \
		src/murty.h \
		src/simulator.h \
		src/task.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o particle.o src/particle.cpp

simulator.o: src/simulator.cpp src/util/timer.h \
		src/simulator.h \
		src/logger.h \
		src/intention.h \
		src/action.h \
		src/params.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/observation.h \
		src/combination.h \
		src/state.h \
		src/tracker.h \
		src/particle.h \
		src/task.h \
		src/murty.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o simulator.o src/simulator.cpp

state.o: src/state.cpp src/state.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/logger.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/tracker.h \
		src/particle.h \
		src/elevator_entering.h \
		src/task.h \
		src/simulator.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o state.o src/state.cpp

task.o: src/task.cpp src/task.h \
		src/logger.h \
		src/intention.h \
		src/action.h \
		src/params.h \
		src/common.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/simulator.h \
		src/observation.h \
		src/combination.h \
		src/state.h \
		src/tracker.h \
		src/particle.h \
		src/open_area.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o task.o src/task.cpp

testCalibration.o: src/testCalibration.cpp src/cameraModel.h \
		src/xmlUtil.h \
		src/params.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/common.h \
		src/gaussian.h \
		src/statistic.h \
		src/logger.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o testCalibration.o src/testCalibration.cpp

tracker.o: src/tracker.cpp src/util/terminal_utils.h \
		src/tracker.h \
		src/particle.h \
		src/state.h \
		src/math/geometry.h \
		src/math/gvector.h \
		src/math/util.h \
		src/math/line.h \
		src/math/geomalgo.h \
		src/logger.h \
		src/common.h \
		src/params.h \
		src/gaussian.h \
		src/statistic.h \
		src/xmlUtil.h \
		src/intention.h \
		src/action.h \
		src/observation.h \
		src/combination.h \
		src/input_stream.h \
		src/util/timer.h \
		src/task.h \
		src/simulator.h \
		src/util/helpers.h \
		src/murty.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tracker.o src/tracker.cpp

xmlUtil.o: src/xmlUtil.cpp src/xmlUtil.h \
		src/params.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o xmlUtil.o src/xmlUtil.cpp

eig3.o: src/math/eig3.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o eig3.o src/math/eig3.cpp

gvector.o: src/math/gvector.cpp src/math/gvector.h \
		src/math/util.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o gvector.o src/math/gvector.cpp

helpers.o: src/util/helpers.cpp src/util/helpers.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o helpers.o src/util/helpers.cpp

proghelp.o: src/util/proghelp.cc src/util/timer.h \
		src/util/proghelp.h \
		src/util/sstring.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o proghelp.o src/util/proghelp.cc

pthread_utils.o: src/util/pthread_utils.cpp src/util/pthread_utils.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o pthread_utils.o src/util/pthread_utils.cpp

terminal_utils.o: src/util/terminal_utils.cpp src/util/terminal_utils.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o terminal_utils.o src/util/terminal_utils.cpp

timer.o: src/util/timer.cc src/util/timer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o timer.o src/util/timer.cc

watch_files.o: src/util/watch_files.cpp src/util/watch_files.h \
		src/util/sstring.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o watch_files.o src/util/watch_files.cpp

moc_benchmark.o: moc_benchmark.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_benchmark.o moc_benchmark.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

