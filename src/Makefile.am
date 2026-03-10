ACLOCAL_AMFLAGS = -I m4 

lib_LTLIBRARIES = libmodule_jitter_measurement.la

BUILT_SOURCES = $(top_builddir)/src/gen_definitions.h
CLEANFILES = $(top_builddir)/src/gen_definitions.h

libmodule_jitter_measurement_la_SOURCES  = jitter_measurement.cpp
libmodule_jitter_measurement_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                                           @ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@
libmodule_jitter_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

VPATH = $(srcdir):$(top_srcdir)/share

GEN_DEFINITIONS = service_definitions/module_jitter_measurement/reset_max_ever pd_definitions/module_jitter_measurement/inputs

$(top_builddir)/src/gen_definitions.h: Makefile $(GEN_DEFINITIONS)
	@echo creating $@
	rk_generate --indir $(top_srcdir)/share --out $@ $(GEN_DEFINITIONS)
	touch $@
