ACLOCAL_AMFLAGS = -I m4 

lib_LTLIBRARIES = libmodule_jitter_measurement.la

BUILT_SOURCES = $(top_builddir)/src/service_definitions.h
CLEANFILES = $(top_builddir)/src/service_definitions.h

libmodule_jitter_measurement_la_SOURCES  = jitter_measurement.cpp
libmodule_jitter_measurement_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                                           @ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@
libmodule_jitter_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

VPATH = $(srcdir):$(top_srcdir)/share/service_definitions

SERVICE_DEFINITIONS = module_jitter_measurement/reset_max_ever

$(top_builddir)/src/service_definitions.h: Makefile $(SERVICE_DEFINITIONS)
	@echo creating $@
	service_generate --indir $(top_srcdir)/share/service_definitions --out $@ $(SERVICE_DEFINITIONS)
	touch $@
