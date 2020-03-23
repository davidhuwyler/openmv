# Sources
SRCS = $(wildcard src/*.c)
SRCS += $(wildcard src/drivers/*.c)
SRCS += $(wildcard src/system/*.c)
SRCS += $(wildcard src/usb/*.c)
SRCS += $(wildcard src/usb/device/source/*.c)
SRCS += $(wildcard src/usb/device/source/ehci/*.c)
SRCS += $(wildcard src/usb/device/class/*.c)
SRCS += $(wildcard src/usb/phy/*.c)
SRCS += $(wildcard src/usb/osa/*.c)
SRCS += $(wildcard src/xip/*.c)

OBJS = $(addprefix $(BUILD)/, $(SRCS:.c=.o))
OBJ_DIRS = $(sort $(dir $(OBJS)))

all: | $(OBJ_DIRS) $(OBJS)
$(OBJ_DIRS):
	$(MKDIR) -p $@

$(BUILD)/%.o : %.c
	$(ECHO) "CC $<"
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o : %.s
	$(ECHO) "AS $<"
	$(AS) $(AFLAGS) $< -o $@

-include $(OBJS:%.o=%.d)
