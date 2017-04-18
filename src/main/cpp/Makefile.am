ACLOCAL_AMFLAGS = -I m4 -I scripts/m4

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_jitter_measurement.la

libmodule_jitter_measurement_la_SOURCES  = jitter_measurement.cpp
libmodule_jitter_measurement_la_CXXFLAGS = -std=c++11 -Wall -I$(srcdir) -I$(builddir) @ARCH_CFLAGS@ \
                                           @YAML_CPP_RMC_CFLAGS@ @ROBOTKERNEL_CFLAGS@ \
										   @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@
libmodule_jitter_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

