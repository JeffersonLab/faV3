#
# File:
#    Makefile
#
# Description:
#    Makefile for compilation of delay_scan_code program
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

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
INCS			= -I. -I.. -I${LINUXVME_INC}
CFLAGS			= -L. -L.. -L${LINUXVME_LIB} -lrt -ljvme -lfaV3
ifeq ($(DEBUG),1)
	CFLAGS		+= -Wall -g
endif

SRC			= $(wildcard *.c)
PROGS			= $(SRC:.c=)

DEPDIR := .deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d
DEPFILES := $(SRC:%.c=$(DEPDIR)/%.d)

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(INCS) $(CPPFLAGS) $(TARGET_ARCH)

all: $(PROGS)

clean distclean:
	@rm -f $(PROGS) *~ $(DEPFILES)

%: %.c
%: %.c  $(DEPDIR)/%.d | $(DEPDIR)
	@echo " CC     $@"
	${Q}$(COMPILE.c) $(CFLAGS) $(INCS) \
		-o $@ $<

$(DEPDIR): ; @mkdir -p $@

$(DEPFILES):
include $(wildcard $(DEPFILES))

.PHONY: all clean distclean
