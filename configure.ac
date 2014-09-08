#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.63])
AC_INIT([module_jitter_measurement], [3.0.3], [robert.burger@dlr.de])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# load env-file with each make invocation
if test -n "$SOURCE_ENV"; then
   echo "loading $SOURCE_ENV"
   source $SOURCE_ENV
fi

# Checks for programs.
AC_PROG_CXX
AC_PROG_AWK
AC_PROG_INSTALL
AC_PROG_LIBTOOL
AC_PROG_LN_S
AC_PROG_MAKE_SET

if test -z "$MAKE"; then
   MAKE=make
fi
# expand path with each make invocation
if test -n "$TOOLCHAIN"; then
   MAKE="$MAKE PATH=$TOOLCHAIN:$PATH"
   PATH=$TOOLCHAIN:$PATH
fi
# load env-file with each make invocation
if test -n "$SOURCE_ENV"; then
   echo "loading $SOURCE_ENV"
   source $SOURCE_ENV
   MAKE="source $SOURCE_ENV; $MAKE"
fi
if test -n "$MAKE"; then
   SET_MAKE="MAKE := $MAKE"
fi
AC_SUBST(SET_MAKE)

# setup rmpm
RMPM_ARCH

if test "$rmpm_host" == "arm-angstrom-linux-gnueabi"; then
   ARCH_CFLAGS="-DNO_RDTSC"
fi
AC_SUBST(ARCH_CFLAGS)

PKG_CHECK_MODULES([ROBOTKERNEL], [robotkernel], [], [
                    RMPM_CHECK_MODULES([ROBOTKERNEL], [robotkernel.robotkernel])
                   ])
LN_BASE=$(env -i "$srcdir"/scripts/get_ln_base.py $ROBOTKERNEL_DEPENDS)
LN_GENERATE_ARGS='-md_dir $(srcdir)/message_definitions'
LN_GENERATE="env -i $LN_BASE/scripts/ln_generate $LN_GENERATE_ARGS"

AC_SUBST(LN_BASE)
AC_SUBST(LN_GENERATE)

AC_DEFINE_UNQUOTED(BUILD_DATE,"`date -u`",[Date (usually date of extraction from SVN).])
AC_DEFINE_UNQUOTED(BUILD_USER,"`whoami`",[User (usually user logged in on shell).])
AC_DEFINE_UNQUOTED(BUILD_HOST,"`hostname`",[Hostname (build host).])

# Checks for header files.
AC_CHECK_HEADERS([stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T

# Checks for library functions.
AC_CHECK_FUNCS([strdup])

# Checks for library functions.
AC_CHECK_LIB(dl, dlopen, DL_LIBS="-ldl", DL_LIBS=)
AC_SUBST(DL_LIBS)

AC_CONFIG_FILES([Makefile])
AC_CONFIG_FILES([module_jitter_measurement.pt], [cp module_jitter_measurement.pt ${ac_abs_top_srcdir}])

AC_OUTPUT

