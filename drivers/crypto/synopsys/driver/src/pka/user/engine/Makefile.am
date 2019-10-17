ACLOCAL_AMFLAGS = -I m4

enginedir = ${libdir}/engines
engine_LTLIBRARIES = libelppka.la

AM_CPPFLAGS = $(OPENSSL_INCLUDES)
AM_CFLAGS = $(LIBPKAUSER_CFLAGS)

libelppka_la_SOURCES = src/pka-engine.c
libelppka_la_LDFLAGS = -module -avoid-version -shared
libelppka_la_LIBADD = $(OPENSSL_LDFLAGS) $(OPENSSL_LIBS) $(LIBPKAUSER_LIBS)
