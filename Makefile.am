libdir = $(prefix)/lib/robotkernel/modules
lib_LTLIBRARIES = libmodule_jitter_measurement.la
BUILT_SOURCES = ln_messages.h
CLEANFILES = ln_messages.h

libmodule_jitter_measurement_la_SOURCES  = module_jitter_measurement.cpp jitter_measurement.cpp
libmodule_jitter_measurement_la_CXXFLAGS = -Wall -I$(top_srcdir)/src -I$(top_builddir) @YAMLCPP_CFLAGS@ @LN_CFLAGS@ -Wno-write-strings @ARCH_CFLAGS@
libmodule_jitter_measurement_la_LDFLAGS  = -Wall @MODULE_LDFLAGS@ @RT_LIBS@ @PTHREAD_LIBS@  -release $(PACKAGE_VERSION)

VPATH = $(srcdir):$(srcdir)/message_definitions 

LN_MESSAGES = robotkernel/jitter_measurement/jitter \
			  robotkernel/jitter_measurement/reset_max_ever \
			  robotkernel/jitter_measurement/get_cps

include $(top_srcdir)/Makefile.defs
	
