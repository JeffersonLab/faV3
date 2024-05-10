#
# File:
#    Makefile
#
# Description:
#    Makefile for the fADC250 (v2) Library using a VME Controller
#      running vxWorks or Linux
#
#
BASENAME=faV3
#
# Uncomment DEBUG line, to include some debugging info ( -g and -Wall)
DEBUG	?= 1
QUIET	?= 1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

ARCH	?= $(shell uname -m)

# Check for CODA 3 environment
ifdef CODA_VME

INC_CODA	= -I${CODA_VME}/include
LIB_CODA	= -L${CODA_VME_LIB}

endif

# Defs and build for VxWorks
ifeq (${ARCH}, PPC)
VXWORKS_ROOT		?= /site/vxworks/5.5/ppc/target
OS			= VXWORKS

ifdef LINUXVME_INC
VME_INCLUDE             ?= -I$(LINUXVME_INC)
endif

CC			= ccppc
LD			= ldppc
DEFS			= -mcpu=604 -DCPU=PPC604 -DVXWORKS -D_GNU_TOOL -mlongcall \
				-fno-for-scope -fno-builtin -fvolatile -DVXWORKSPPC
INCS			= -I. -I$(VXWORKS_ROOT)/h  \
				$(VME_INCLUDE)
CFLAGS			= $(INCS) $(DEFS)

else
OS			= LINUX
endif #OS=VXWORKS#

# Defs and build for Linux
ifeq ($(OS),LINUX)

# Safe defaults
LINUXVME_LIB		?= ../lib
LINUXVME_INC		?= ../include

CC			= gcc
ifeq ($(ARCH),i686)
CC			+= -m32
endif
AR                      = ar
RANLIB                  = ranlib
CFLAGS			= -L. -L${LINUXVME_LIB} ${LIB_CODA}
INCS			= -I. -I${LINUXVME_INC} ${INC_CODA}

LIBS			= lib${BASENAME}.a lib${BASENAME}.so
endif #OS=LINUX#

ifdef DEBUG
CFLAGS			+= -Wall -g -Wno-unused
else
CFLAGS			+= -O2
endif

SRC			= ${BASENAME}Lib.c faV3Config.c faV3FirmwareTools.c faV3Itrig.c faV3-HallD.c
OBJ			= $(SRC:%.c=%.o)
HDRS			= $(SRC:%.c=%.h)

DEPDIR := .deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d
DEPFILES := $(SRC:%.c=$(DEPDIR)/%.d)

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(INCS) $(CPPFLAGS) $(TARGET_ARCH)

ifeq ($(OS),LINUX)
all: echoarch ${LIBS}
else
all: echoarch $(OBJ)
endif

%.o: %.c
%.o: %.c $(DEPDIR)/%.d | $(DEPDIR)
	@echo " CC     $@"
	${Q}$(COMPILE.c) -fPIC -c -o $@ $<

libfaV3.so: ${OBJ}
	@echo " CC     $@"
	${Q}$(COMPILE.c)  -shared -lm -o $@ ${OBJ}

%.a: $(OBJ)
	@echo " AR     $@"
	${Q}$(AR) ru $@ $<
	@echo " RANLIB $@"
	${Q}$(RANLIB) $@

ifeq ($(OS),LINUX)
install: $(LIBS)
	@echo " CP     $<"
	${Q}cp $(PWD)/$< $(LINUXVME_LIB)/$<
	@echo " CP     $(<:%.a=%.so)"
	${Q}cp $(PWD)/$(<:%.a=%.so) $(LINUXVME_LIB)/$(<:%.a=%.so)
	@echo " CP     ${HDRS}"
	${Q}cp $(PWD)/${HDRS} $(LINUXVME_INC)

coda_install: $(LIBS)
	@echo " CODACP $<"
	${Q}cp $(PWD)/$< $(CODA_VME_LIB)/$<
	@echo " CODACP $(<:%.a=%.so)"
	${Q}cp $(PWD)/$(<:%.a=%.so) $(CODA_VME_LIB)/$(<:%.a=%.so)
	@echo " CODACP ${HDRS}"
	${Q}cp $(PWD)/${HDRS} $(CODA_VME)/include

endif

clean:
	@rm -vf ${OBJ} ${LIBS} $(DEPFILES)

realclean: clean
	@rm -vf *~

echoarch:
	@echo "Make for $(OS)-$(ARCH)"

$(DEPDIR): ; @mkdir -p $@

$(DEPFILES):
include $(wildcard $(DEPFILES))

.PHONY: clean echoarch
