# TODO better way to find firmatacpp
firmatadir?=$(HOME)/firmatacpp
bluedir?=${firmatadir}/libblepp-prefix

# on Ubuntu 14.04 set this to c++1y
CPPSTD?=gnu++14

CPPFLAGS+= -I$(firmatadir)/include
CPPFLAGS+= -I$(firmatadir)
CPPFLAGS+= -I$(firmatadir)/vendor/serial/include
CPPFLAGS+= $(if $(NO_BLUETOOTH),,-I$(bluedir)/include)
CPPFLAGS+= -g3 -std=$(CPPSTD)
CPPFLAGS+= $(if $(NO_BLUETOOTH),-DNO_BLUETOOTH)

LDFLAGS+= -L$(firmatadir)
LDFLAGS+= -L$(firmatadir)/vendor/serial
LDFLAGS+= -rdynamic
LDFLAGS+= -g3 -std=$(CPPSTD)
ifeq ($(NO_BLUETOOTH),)
LDFLAGS+= -L$(bluedir)/lib
LDFLAGS+= -Wl,-rpath,$(bluedir)
vpath %.so $(bluedir)/lib
endif

daemon:=scratchdaemon

$(daemon): $(daemon).o
$(daemon): $(firmatadir)/libfirmatacpp.a
$(daemon): $(firmatadir)/vendor/serial/libserial.a
$(daemon): $(if $(NO_BLUETOOTH),,-lble++) -lpthread -lrt
$(daemon): CC=$(CXX)

$(daemon).o: $(daemon).cpp

clean:
	rm -f $(daemon) $(daemon).o
