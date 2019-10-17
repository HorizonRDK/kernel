AC_DEFUN([_ELP_LIBPKAUSER_TEST_PROGRAM], [AC_LANG_PROGRAM([dnl
#include "pkadev.h"
], [dnl
int fd = elppka_device_open(0);
elppka_device_close(fd);
])])

AC_DEFUN([ELP_CHECK_LIBPKAUSER], [dnl
AC_ARG_VAR([LIBPKAUSER_CFLAGS], [C compiler flags for libpkauser])
AC_ARG_VAR([LIBPKAUSER_LIBS], [linker flags for libpkauser])

AC_LANG_PUSH([C])

old_cflags=$CFLAGS
old_libs=$LIBS

AC_CACHE_CHECK([libpkauser], [elp_cv_libpkauser_found], [dnl
	elp_cv_libpkauser_found=no

	CFLAGS="$LIBPKAUSER_CFLAGS $old_cflags"
	LIBS="$LIBPKAUSER_LIBS $old_libs"

	AC_LINK_IFELSE([_ELP_LIBPKAUSER_TEST_PROGRAM], [dnl
		elp_cv_libpkauser_found=yes], [:])
])

if test x"$elp_cv_libpkauser_found" = x"yes"; then
	ifelse([$1], [], [:], [$1])
else
	ifelse([$2], [], [AC_MSG_FAILURE([libpkauser is required.])], [$2])
fi

AC_LANG_POP([C])

LIBS=$old_libs
CFLAGS=$old_cflags
])
