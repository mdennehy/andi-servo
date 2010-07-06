#-----------------------------------------------------------------------
#	Dagda 
#
#	Mark Dennehy
#
#-----------------------------------------------------------------------
#	Main Makefile
#-----------------------------------------------------------------------

ROOTDIR := $(shell pwd)
BINDIR := $(ROOTDIR)/bin
INCDIR := $(ROOTDIR)/include
OBJDIR := $(ROOTDIR)/obj
LIBDIR := $(ROOTDIR)/lib
SRCDIR := $(ROOTDIR)/src

MODULEDIR := /lib/modules/`uname -r`/misc

CC = /usr/bin/gcc
CFLAGS = -DMODULE -D_REENTRANT -DMODVERSIONS -Dlinux -DLINUX -g -O2 -Wall  $(INCLUDES) 
INCLUDES := -I /lib/modules/`uname -r`/build/include -include /lib/modules/`uname -r`/build/include/linux/modversions.h -I $(INCDIR)

INDENT = indent 
INDENTOPTIONS = -bli0 -cli0 -cbi0 -npcs -cs -bs -nbc -npsl -bls -i4 -lp -ts4 -l80 -hnl -bbo -nbad -bap -bbb -sob -d0 -nip -pmt 

PRINT = a2ps
PRINTFLAGS = -1 -E -g -C -T 4 -f 8

TEX = latex
TEXFLAGS = 

TEXFILES = Driver.tex
SRCS = demo.c userspacedriver.c servo.c andi_servo.c test.c
OBJS = userspacedriver.o servo.o andi_servo.o 
TARGETS = driver test userspacedemo

########################################################################

.PHONY : all clean hardcopy listtargets listobjectfiles neat tidy docs install

all : neat dirs listobjectfiles listtargets $(TARGETS)

clean : 
	@-rm $(OBJDIR)/*.o $(BINDIR)/demo 

hardcopy :
	$(PRINT) $(PRINTFLAGS) $(INCLUDEDIR)/*.h $(patsubst %,$(SRCDIR)/%,$(SRCS))

listtargets :
	@echo ========================================================================
	@echo $(TARGETS)
	@echo ========================================================================
	@echo

listobjectfiles :
	@echo ========================================================================
	@echo $(OBJS)
	@echo ========================================================================
	@echo

tidy : neat

neat : 
	@echo Tidying up code indentation ...
	@$(INDENT) $(INDENTOPTIONS) include/*.h src/*.c

docs :
	cd docs; $(foreach file, $(TEXFILES), $(TEX) $(TEXFLAGS) $(file); $(TEX) $(TEXFLAGS) $(file); )

dirs : 
	@-mkdir obj lib 

########################################################################

userspacedemo : dirs userspacedriver.o $(SRCDIR)/demo.c
	@echo
	@echo $@
	@echo ------------------------------------------------------------------------
	$(CC) $(CFLAGS) $(SRCDIR)/demo.c $(OBJDIR)/userspacedriver.o -o $(BINDIR)/userspacedemo
	@echo
	@echo

driver: dirs andi_servo.o servo.o $(INCDIR)/andi.h
	@echo
	@echo $@
	@echo ------------------------------------------------------------------------
	cd obj; $(LD) -r -o $(BINDIR)/andi.o servo.o andi_servo.o
	@echo ------------------------------------------------------------------------
	@echo

test: driver $(SRCDIR)/test.c $(INCDIR)/andi.h
	@echo
	@echo $@
	@echo ------------------------------------------------------------------------
	$(CC) $(CFLAGS) -o $(BINDIR)/test $(SRCDIR)/test.c
	@echo ------------------------------------------------------------------------
	@echo

install: clean driver
	sudo cp $(BINDIR)/andi.o $(MODULEDIR)

########################################################################
#%.o: $(SRCDIR)/%.c

$(OBJS) : %.o: $(SRCDIR)/%.c
	@echo 
	@echo $@
	@echo ------------------------------------------------------------------------
	$(CC) $(CFLAGS) -c $< -o $(OBJDIR)/$@ 
	@echo ------------------------------------------------------------------------
	@echo
	@echo

