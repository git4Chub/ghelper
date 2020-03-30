#include "ql_type.h"
#include "ql_stdlib.h"

s32   Ql_isspace(char c)
{
	return c == ' ' || (unsigned)c - '\t' < 5;
}
