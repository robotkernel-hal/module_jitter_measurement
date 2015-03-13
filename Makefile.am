ACLOCAL_AMFLAGS = -I m4 

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_jitter_measurement.la
BUILT_SOURCES = ln_messages.h
CLEANFILES = ln_messages.h

libmodule_jitter_measurement_la_SOURCES  = module_jitter_measurement.cpp jitter_measurement.cpp
libmodule_jitter_measurement_la_CXXFLAGS = -Wall -I$(top_srcdir)/src -I$(top_builddir) @ROBOTKERNEL_CFLAGS@ @LN_CFLAGS@ -Wno-write-strings @ARCH_CFLAGS@
libmodule_jitter_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

VPATH = $(srcdir):$(srcdir)/message_definitions 

LN_MESSAGES = robotkernel/jitter_measurement/jitter \
			  robotkernel/jitter_measurement/reset_max_ever \
			  robotkernel/jitter_measurement/get_cps

ln_messages.h: Makefile $(LN_MESSAGES)
	@echo creating ln_messages.h
	@LN_GENERATE@ -o $@ $(LN_MESSAGES)
	touch $@

install-exec-hook:
	mkdir -p $(prefix)/share/message_definitions
	cp -r $(srcdir)/message_definitions/* $(prefix)/share/message_definitions/
	mkdir -p $(prefix)/share/ln
	-cp -rf $(srcdir)/share/ln/* $(prefix)/share/ln/
