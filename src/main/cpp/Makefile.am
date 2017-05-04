ACLOCAL_AMFLAGS = -I m4 

lib_LTLIBRARIES = libmodule_jitter_measurement.la

libmodule_jitter_measurement_la_SOURCES  = jitter_measurement.cpp
libmodule_jitter_measurement_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                                           @YAML_CPP_CFLAGS@ @ROBOTKERNEL_CFLAGS@ \
										   @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@
libmodule_jitter_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

