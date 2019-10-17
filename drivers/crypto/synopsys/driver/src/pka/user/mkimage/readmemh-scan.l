%top{
/*
 * This Synopsys software and associated documentation (hereinafter the
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you. The
 * Software IS NOT an item of Licensed Software or a Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Products
 * with Synopsys or any supplement thereto. Synopsys is a registered trademark
 * of Synopsys, Inc. Other names included in the SOFTWARE may be the
 * trademarks of their respective owners.
 *
 * The contents of this file are dual-licensed; you may select either version
 * 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
 * ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
 * SOFTWARE. The BSD License is copied below.
 *
 * BSD-3-Clause License:
 * Copyright (c) 2013-2015 Synopsys, Inc. and/or its affiliates.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions, and the following disclaimer, without
 *    modification.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of the above-listed copyright holders may not be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// ------------------------------------------------------------------------
//
//  Project:
//
//   Driver SDK
//
//  Description:
//
//   Flex scanner for IEEE-1364 $readmemh-style input files.
//
// ------------------------------------------------------------------------

#include "readmemh-parse.h"
}

%option nodefault bison-locations reentrant never-interactive
%option noyywrap nounput noinput
%option prefix="readmemh_yy"

%x COMMENT

%{
#define FAIL(msg) do { \
   READMEMH_ERRMSG(yylloc, msg); \
   return T_LEX_ERROR; \
} while(0)

static int decode_hex(int c)
{
   switch (c) {
   case 'a': case 'A': return 0xa;
   case 'b': case 'B': return 0xb;
   case 'c': case 'C': return 0xc;
   case 'd': case 'D': return 0xd;
   case 'e': case 'E': return 0xe;
   case 'f': case 'F': return 0xf;
   }

   return c - '0';
}
%}

%%

"/*" BEGIN(COMMENT);
<COMMENT>{
   "*/" BEGIN(INITIAL);
   \n
   .
}
<INITIAL>"//".*$

[_[:xdigit:]xXzZ]+ { 
   yylval->number.val = yylval->number.zmask = 0;
   yylval->number.xmask = -1;

   for (char *c = yytext; *c; c++) {
      if (*c == '_')
         continue;

      if (yylval->number.val > UINTMAX_MAX/16
          || yylval->number.zmask > UINTMAX_MAX/16) {
         FAIL("input value is too large");
      }

      yylval->number.val *= 16;
      yylval->number.xmask *= 16;
      yylval->number.zmask *= 16;

      switch (*c) {
      case 'x': case 'X': yylval->number.xmask |= 0xf; break;
      case 'z': case 'Z': yylval->number.zmask |= 0xf; break;
      default: yylval->number.val += decode_hex(*c);
      }
   }

   return T_NUMBER;
}

@[[:xdigit:]]+ {
   yylval->number.val = yylval->number.xmask = yylval->number.zmask = 0;

   for (char *c = yytext+1; *c; c++) {
      if (yylval->number.val > UINTMAX_MAX/16)
         FAIL("input address is too large");

      yylval->number.val *= 16;
      yylval->number.val += decode_hex(*c);
   }

   return T_ADDRESS;
}

[[:space:]]+
. {
   char buf[] = "syntax error, unexpected #";
   *strchr(buf, '#') = *yytext;
   FAIL(buf);
}

%%
