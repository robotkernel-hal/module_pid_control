lib_LTLIBRARIES = libmodule_current_control.la

libmodule_current_control_la_SOURCES  = current_control.cpp
libmodule_current_control_la_CXXFLAGS = -Wall -fno-strict-aliasing -I$(srcdir) -I$(builddir) \
                                  @ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@
libmodule_current_control_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)


