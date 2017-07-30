# TODO better way to find firmatacpp
firmatadir?=$(HOME)/firmatacpp-master
bluedir?=${firmatadir}/libblepp-prefix

CPPFLAGS+= -I$(firmatadir)/include
CPPFLAGS+= -I$(firmatadir)
CPPFLAGS+= -I$(firmatadir)/vendor/serial/include
CPPFLAGS+= -I$(bluedir)/include
CPPFLAGS+= -g3 -std=gnu++14

LDFLAGS+= -L$(firmatadir)
LDFLAGS+= -L$(firmatadir)/vendor/serial
LDFLAGS+= -L$(bluedir)/lib
LDFLAGS+= -rdynamic
LDFLAGS+= -Wl,-rpath,$(bluedir)
LDFLAGS+= -g3 -std=gnu++14
vpath %.so $(bluedir)/lib

daemon:=scratchdaemon

$(daemon): $(daemon).o
$(daemon): $(firmatadir)/libfirmatacpp.a
$(daemon): $(firmatadir)/vendor/serial/libserial.a
$(daemon): -lble++ -lpthread -lrt
$(daemon): CC=$(CXX)

$(daemon).o: $(daemon).cpp

clean:
	rm -f $(daemon) $(daemon).o
