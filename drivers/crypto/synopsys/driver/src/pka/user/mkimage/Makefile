GROFF = groff
FLEX = flex
BISON = bison
#CC = aarch64-linux-gnu-gcc
CC = gcc

OBJS = mkimage.o readmemh.o readmemh-scan.o readmemh-parse.o \
	symbol.o symbol-scan.o symbol-parse.o pkaelf.o
DEPFILES = $(OBJS:.o=.d)
LIBS = ../pka-tool-common.o -lelf

INCLUDES = -I..
ALL_CPPFLAGS = $(CPPFLAGS) $(INCLUDES)
ALL_CFLAGS = $(CFLAGS)

WARNINGS = -Wall -Wno-missing-braces -Werror=implicit-int -Werror=implicit-function-declaration
CFLAGS = -g -O $(WARNINGS)

PDFDOCS = pka-mkimage.pdf

all: pka-mkimage
.PHONY: all

doc: pdf
pdf: $(PDFDOCS)
.PHONY: doc pdf

clean distclean:
	rm -f pka-mkimage $(OBJS) $(DEPFILES)
	rm -f $(PDFDOCS)
	rm -f *.tmp
.PHONY: clean distclean

maintainer-clean: distclean
	rm -f readmemh-parse.c readmemh-parse.h readmemh-scan.c readmemh-scan.h
	rm -f symbol-parse.c symbol-parse.h symbol-scan.c symbol-scan.h
.PHONY: maintainer-clean

pka-mkimage: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(ALL_CPPFLAGS) $(ALL_CFLAGS) -c -MMD -MP $<
-include $(DEPFILES)

%.pdf: %.1
	$(GROFF) -mdoc -Tpdf $< >$@.tmp
	mv -f $@.tmp $@

%.c %.h: %.l
	$(FLEX) -o $*.c --header-file=$*.h $(FLEXFLAGS) $<

%.c %.h: %.y
	$(BISON) -o $*.c --defines=$*.h $(BISONFLAGS) $<

readmemh.o: readmemh-scan.h readmemh-parse.h
readmemh-scan.o: readmemh-parse.h
readmemh-parse.o: readmemh-scan.h

symbol.o: symbol-scan.h symbol-parse.h
symbol-scan.o: symbol-parse.h
symbol-parse.o: symbol-scan.h

.SECONDARY:
