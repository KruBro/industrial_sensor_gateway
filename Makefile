# Industrial Sensor Gateway - Linux side (Modbus RTU master + tests)
#
#   make              build the master, hmi_client, and dbview -> build/master, build/hmi_client, build/dbview
#   make test         build and run unit tests (CRC16 + Modbus frame layer + M11 unit tests)
#   make test-master  run the master against a fake slave (needs python3)
#   make clean        remove build/
#
# Whole-frame writes are the default. To add a per-byte delay when sending
# (for emulators that drop fast bytes), build with:
#   make clean && make SIM_BYTE_DELAY_US=120000

CC       ?= gcc
CFLAGS   ?= -Wall -Wextra -Wshadow -O2 -g
CPPFLAGS += -Icommon -Igateway -Iserver
BUILD    := build

COMMON_SRC   := common/crc16.c common/modbus.c
GATEWAY_SRC  := gateway/main.c gateway/master.c gateway/main_helper.c gateway/serial_io.c gateway/poller.c gateway/db.c gateway/alarm.c
CLIENT_SRC   := client/hmi_client.c
SERVER_SRC   := server/hmi.c
TEST_SRC     := tests/test_modbus.c
TEST_M11_SRC := tests/test_m11.c

COMMON_OBJ   := $(COMMON_SRC:%.c=$(BUILD)/%.o)
GATEWAY_OBJ  := $(GATEWAY_SRC:%.c=$(BUILD)/%.o)
CLIENT_OBJ   := $(CLIENT_SRC:%.c=$(BUILD)/%.o)
SERVER_OBJ   := $(SERVER_SRC:%.c=$(BUILD)/%.o)
TEST_OBJ     := $(TEST_SRC:%.c=$(BUILD)/%.o)
TEST_M11_OBJ := $(TEST_M11_SRC:%.c=$(BUILD)/%.o)

ifdef SIM_BYTE_DELAY_US
$(BUILD)/gateway/master.o: CPPFLAGS += -DSIM_BYTE_DELAY_US=$(SIM_BYTE_DELAY_US)
endif

.PHONY: all test test-master clean dbview

all: $(BUILD)/master $(BUILD)/hmi_client $(BUILD)/dbview

$(BUILD)/master: $(GATEWAY_OBJ) $(SERVER_OBJ) $(COMMON_OBJ)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD)/hmi_client: $(CLIENT_OBJ)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD)/gateway/dbview.o: gateway/dbview.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/dbview: $(BUILD)/gateway/dbview.o $(BUILD)/gateway/db.o $(COMMON_OBJ)
	$(CC) $(CFLAGS) -o $@ $^

dbview: $(BUILD)/dbview

$(BUILD)/test_modbus: $(TEST_OBJ) $(COMMON_OBJ)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD)/test_m11: $(TEST_M11_OBJ) $(BUILD)/gateway/db.o $(BUILD)/gateway/alarm.o $(SERVER_OBJ) $(COMMON_OBJ)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

test: $(BUILD)/test_modbus $(BUILD)/test_m11
	$(BUILD)/test_modbus
	$(BUILD)/test_m11

# A second master binary with a 20 ms per-byte delay, to test that code path.
$(BUILD)/master_delay: $(GATEWAY_SRC) $(SERVER_SRC) $(COMMON_SRC)
	@mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -DSIM_BYTE_DELAY_US=20000 -o $@ $^

test-master: $(BUILD)/master $(BUILD)/master_delay
	python3 tests/fake_slave_test.py $(BUILD)/master $(BUILD)/master_delay

clean:
	rm -rf $(BUILD)

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)