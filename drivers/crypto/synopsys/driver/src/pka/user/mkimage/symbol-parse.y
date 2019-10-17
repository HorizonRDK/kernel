%code top {
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
//   Bison parser for symbol files output by the PKA assembler.
//
// ------------------------------------------------------------------------
}

%name-prefix "symbol_yy"
%parse-param {void *scanner}
%parse-param {struct symbol **out}
%lex-param {yyscan_t scanner}
%define api.pure
%error-verbose
%locations

%code requires {
#include <inttypes.h>
#include "symbol.h"
}

%code provides {
#ifndef SYMBOL_H_
#define SYMBOL_H_

#define SYMBOL_ERRMSG(loc, msg) do { \
   symbol_yyerror(loc, NULL, NULL, msg); \
} while (0)

void symbol_yyerror(YYLTYPE *, void *, struct symbol **, const char *);
int symbol_yyparse(void *, struct symbol **);

#endif
}

%union {
   struct symbol *symbol;
   uintmax_t number;
   char *string;
}

%{
#include "symbol-scan.h"

#define FAIL(msg) do { \
   SYMBOL_ERRMSG(&yylloc, msg); \
   YYERROR; \
} while (0)

static void symbol_free(struct symbol *sym)
{
   while (sym) {
      struct symbol *tmp = sym->next;
      free(sym);
      sym = tmp;
   }
}

void pka_free_symbols(struct symbol *sym)
{
   symbol_free(sym);
}

%}

%destructor { symbol_free($$); } <symbol>
%destructor { free($$); } <string>

/* Magic token for communicating lexer failures. */
%token T_LEX_ERROR;

%token T_ARROW "=>";
%token <number> T_NUMBER "number";
%token <string> T_IDENTIFIER "identifier";

%type <symbol> symbol symbols

%%

input: symbols {
   *out = $1;
}

symbols: symbol symbols {
   $1->next = $2;
   $$ = $1;
} | { $$ = NULL; }

symbol: T_IDENTIFIER T_ARROW T_NUMBER {
   size_t len = strlen($1);
   $$ = realloc($1, sizeof *$$ + len + 1);
   if (!$$)
      FAIL("failed to allocate memory");

   memmove($$->name, $$, len + 1);
   $$->address = $3;
   $$->next = NULL;
}

%%

void yyerror(YYLTYPE *loc, yyscan_t scanner, struct symbol **out, const char *err)
{
   fprintf(stderr, "%s\n", err);
}
