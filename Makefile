PROJECT = eeprom
BUILD_DIR = bin

#SHARED_DIR = ../common
CFILES = main.c

DEVICE=stm32G031f6

#OOCD_FILE =

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
