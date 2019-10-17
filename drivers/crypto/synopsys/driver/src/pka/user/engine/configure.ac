AC_PREREQ([2.66])
AC_INIT([pka-engine], [0.1])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])
AM_SILENT_RULES([yes])

AC_PROG_CC_C99
LT_INIT

if test x"$LIBPKAUSER_CFLAGS" = x; then
	LIBPKAUSER_CFLAGS="-I.."
fi

if test x"$LIBPKAUSER_LIBS" = x; then
	LIBPKAUSER_LIBS="-L.. -lpkauser"
fi

ELP_CHECK_LIBPKAUSER

AX_CHECK_OPENSSL([:], [AC_MSG_FAILURE([OpenSSL is required.])])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
