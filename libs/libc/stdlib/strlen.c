/* Implementation of strtod for systems with atof.
   Copyright (C) 1991-2017 Free Software Foundation, Inc.
This file is part of the libiberty library.  This library is free
software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option)
any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with GNU CC; see the file COPYING.  If not, write to
the Free Software Foundation, 51 Franklin Street - Fifth Floor, Boston, MA 02110-1301, USA.
As a special exception, if you link this library with files
compiled with a GNU compiler to produce an executable, this does not cause
the resulting executable to be covered by the GNU General Public License.
This exception does not however invalidate any other reasons why
the executable file might be covered by the GNU General Public License. */
/*
@deftypefn Supplemental double strtod (const char *@var{string}, @
  char **@var{endptr})
This ISO C function converts the initial portion of @var{string} to a
@code{double}.  If @var{endptr} is not @code{NULL}, a pointer to the
character after the last character used in the conversion is stored in
the location referenced by @var{endptr}.  If no conversion is
performed, zero is returned and the value of @var{string} is stored in
the location referenced by @var{endptr}.
@end deftypefn
*/
#include "ql_stdlib.h"

u32 strlen(const char* str)
{
    return Ql_strlen(str);
}

