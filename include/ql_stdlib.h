/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2019
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ql_stdlib.h 
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   this file provides some Standard library APIs
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/
 

#ifndef __QL_STDLIB_H__
#define __QL_STDLIB_H__
#include "ql_type.h"
#include <stdarg.h>

#ifndef RAND_MAX
#define RAND_MAX (0x7fffffff)
#endif

#define atoi        Ql_atoi
#define atof        Ql_atof
#define memset      Ql_memset
#define memcpy      Ql_memcpy
#define memcmp      Ql_memcmp
#define memmove     Ql_memmove
#define strcpy      Ql_strcpy
#define strncpy     Ql_strncpy
#define strcat      Ql_strcat
#define strncat     Ql_strncat
#define strcmp      Ql_strcmp
#define strncmp     Ql_strncmp
#define strchr      Ql_strchr
/* #define strlen      Ql_strlen */
#define strstr      Ql_strstr
#define strtod      Ql_strtod
#define toupper     Ql_toupper
#define tolower     Ql_tolower
#define isdigit     Ql_isdigit
#define isspace     Ql_isspace
#define sprintf     Ql_sprintf
#define snprintf    Ql_snprintf
#define sscanf      Ql_sscanf
#define realloc     Ql_realloc

#define vsprintf    Ql_vsprintf
#define vsnprintf   Ql_vsnprintf

#define rand        Ql_rand
#define srand       Ql_srand

s32    Ql_atoi(const char* s);
double Ql_atof(const char* s);
void* Ql_memset(void* dest, u8 value, u32 size);
void* Ql_memcpy(void* dest, const void* src, u32 size);
s32   Ql_memcmp(const void* dest, const void*src, u32 size);
void* Ql_memmove(void* dest, const void* src, u32 size);
char* Ql_strcpy(char* dest, const char* src);
char* Ql_strncpy(char* dest, const char* src, u32 size);
char* Ql_strcat(char* s1, const char* s2);
char* Ql_strncat(char* s1, const char* s2, u32 size);
s32   Ql_strcmp(const char*s1, const char*s2);
s32   Ql_strncmp(const char* s1, const char* s2, u32 size);
char* Ql_strchr(const char* s1, s32 ch);
u32   Ql_strlen(const char* str);
u32   strlen(const char* str);
char* Ql_strstr(const char* s1, const char* s2);
double Ql_strtod (char *str, char **ptr);
s32   Ql_toupper(s32 c);
s32   Ql_tolower(s32 c);
s32   Ql_isdigit(char c);
void* Ql_realloc(void* ptr, u32 size);
int   Ql_rand(void);
void  Ql_srand(unsigned seed);
s32 Ql_vsprintf(char* s,const char* format,va_list  arg);
extern s32 (*Ql_sprintf)(char *, const char *, ...);
extern s32 (*Ql_snprintf)(char *, u32, const char *, ...);
extern s32 (*Ql_sscanf)(const char*, const char*, ...);
extern s32 (*Ql_vsnprintf)(char *, u32, const char *, ...);

#endif  // __QL_STDLIB_H__
