lib_LTLIBRARIES = libmodule_pid_control.la

libmodule_pid_control_la_SOURCES  = pid_control.cpp
libmodule_pid_control_la_CXXFLAGS = -Wall -fno-strict-aliasing -I$(srcdir) -I$(builddir) \
                                  @ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@
libmodule_pid_control_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)


