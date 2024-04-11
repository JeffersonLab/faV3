#
# File:
#    Makefile
#
# Description:
#    Makefile for fadc250 test programs
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

ifdef CODA_VME
CODA_VME_INC = -I${CODA_VME}/include
endif
ifdef CODA_VME_LIB
CODA_LIB = -L${CODA_VME_LIB}
endif

# linuxvme defaults, if they're not already defined
LINUXVME_INC	?= .
LINUXVME_LIB	?= .

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
INCS			= -I. -I../ -I${LINUXVME_INC} ${CODA_VME_INC}
CFLAGS			= -L. -L../ -L${LINUXVME_LIB} ${CODA_LIB} -lrt -ljvme -lfaV3
ifeq ($(DEBUG),1)
	CFLAGS		+= -Wall -Wno-unused -g
endif

SRC			= $(wildcard *.c)
PROGS			= $(SRC:.c=)

DEPDIR := .deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d
DEPFILES := $(SRC:%.c=$(DEPDIR)/%.d)

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(INCS) $(CPPFLAGS) $(TARGET_ARCH)

all: $(PROGS)

clean:
	@rm -vf $(PROGS) *~ $(DEPFILES)

%: %.c
%: %.c  $(DEPDIR)/%.d | $(DEPDIR)
	@echo " CC     $@"
	${Q}$(COMPILE.c) $(CFLAGS) $(INCS) -o $@ $<

$(DEPDIR): ; @mkdir -p $@

$(DEPFILES):
include $(wildcard $(DEPFILES))

.PHONY: all clean
