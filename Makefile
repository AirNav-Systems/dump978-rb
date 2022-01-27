VERSION ?= $(shell git rev-parse --short HEAD)
ifneq ($(VERSION),)
  CPPFLAGS+=-DVERSION=\"$(VERSION)\"
endif

CC ?= gcc
CFLAGS += -Wall -Werror -O2 -g -Ilibs

CXX ?= g++
CXXFLAGS += -std=c++11 -Wall -Wno-psabi -Werror -O2 -g -Ilibs

LIBS=-lboost_system -lboost_program_options -lboost_regex -lboost_filesystem -lpthread
LIBS_SDR=-lSoapySDR

all: dump978-rb

dump978-rb: dump978_main.o socket_output.o message_dispatch.o fec.o libs/fec/init_rs_char.o libs/fec/decode_rs_char.o sample_source.o soapy_source.o convert.o demodulator.o uat_message.o stratux_serial.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LIBS) $(LIBS_SDR)

fec_tests: fec_tests.o libs/fec/init_rs_char.o libs/fec/decode_rs_char.o libs/fec/encode_rs_char.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@

format:
	clang-format -style=file -i *.cc *.h

clean:
	rm -f *.o libs/fec/*.o dump978-rb fec_tests
