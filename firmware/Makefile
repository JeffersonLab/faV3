# File:
#    Makefile
#
# Description:
#	Makefile for fADC250 V3 firmware updating code (Linux ony)
#
#
DEBUG	?= 1
QUIET	?= 1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

#Check Operating system we are using
ifndef OSNAME
  OSNAME := $(subst -,_,$(shell uname))
endif

# Check for CODA 3 environment
ifdef CODA_VME

INC_CODA	= -I${CODA_VME}/include
LIB_CODA	= -L${CODA_VME_LIB}

endif

# Safe defaults
LINUXVME_LIB		?= ../../lib
LINUXVME_INC		?= ../../include
LINUXVME_BIN		?= ../../bin

CFLAGS			= -L. -L.. -L${LINUXVME_LIB} ${LIB_CODA} -lrt -ljvme -lfaV3
INCS			= -I. -I.. -I${LINUXVME_INC} ${INC_CODA}

ifdef DEBUG
CFLAGS	+= -Wall -g
else
CFLAGS	+= -O2
endif

SRC			= faV3FirmwareUpdate.c faV3GFirmwareUpdate.c
PROGS			= $(SRC:.c=)

DEPDIR := .deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d
DEPFILES := $(SRC:%.c=$(DEPDIR)/%.d)

all: $(PROGS)

%: %.c
%: %.c  $(DEPDIR)/%.d | $(DEPDIR)
	@echo " CC     $@"
	${Q}$(CC) $(DEPFLAGS) $(CFLAGS) $(INCS) -o $@ $<

install: $(PROGS)
	@echo " CP     ${PROGS}"
	${Q}cp ${PWD}/$(PROGS) $(LINUXVME_BIN)/

coda_install: $(PROGS)
	@echo " CODACP ${PROGS}"
	${Q}cp $(PROGS) $(CODA_VME_BIN)/

$(DEPDIR): ; @mkdir -p $@

$(DEPFILES):
include $(wildcard $(DEPFILES))

clean:
	@rm -vf *~ $(PROGS) $(DEPFILES)
