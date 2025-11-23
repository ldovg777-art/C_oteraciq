CC = arm-linux-gnueabihf-gcc
CFLAGS = -O2 -Wall
INCLUDES = -I/usr/local/include -I./includes
LDFLAGS = -L/usr/local/lib -L./libs

# Libraries
LIBS_STEP = -lmodbus -ladamapi -lm
LIBS_SERVER = -lmodbus -lm

# Targets
all: adam_step server

adam_step: adam6224_iter_step.c
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES) $(LDFLAGS) $(LIBS_STEP)

server: iter_modbus_server.c
	$(CC) -static $(CFLAGS) $< -o iter_modbus_server_arm $(INCLUDES) $(LDFLAGS) $(LIBS_SERVER)

clean:
	rm -f adam_step iter_modbus_server_arm
