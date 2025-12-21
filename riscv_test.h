#ifndef _ENV_VIRTUAL_SINGLE_CORE_H
#define _ENV_VIRTUAL_SINGLE_CORE_H

// -------------------------------------------------------------------------
// 0. DEFINICIONES DE MODO (¡Lo que faltaba!)
// -------------------------------------------------------------------------
// Estas macros le dicen al test que ignore la selección de modo
// ya que estamos en bare-metal simple.
#define RVTEST_RV32U
#define RVTEST_RV64U

// Usamos x30 (t5) para contar el número de test internamente
#define TESTNUM x30

// -------------------------------------------------------------------------
// 1. INICIO
// -------------------------------------------------------------------------
#define RVTEST_CODE_BEGIN                                               \
        .section .text.init;                                            \
        .globl _start;                                                  \
        _start:                                                         \
                /* Inicializar registros clave a 0 por seguridad */     \
                li      x10, 0; /* x10 (a0) tendrá el resultado final */\
                j       run_test;                                       \
        run_test:;

// -------------------------------------------------------------------------
// 2. FINALIZACIÓN (PASS / FAIL) via REGISTROS
// -------------------------------------------------------------------------

// PASS: Escribe 1 en x10 (a0) y se queda en bucle
#define RVTEST_PASS                                                     \
        li      x10, 1;                                                 \
        pass_loop: j pass_loop

// FAIL: Escribe el número de test (TESTNUM) en x10 y se queda en bucle
// Nota: TESTNUM suele tener el nº de test desplazado (bitshifted)
#define RVTEST_FAIL                                                     \
        mv      x10, TESTNUM;                                           \
        fail_loop: j fail_loop

// -------------------------------------------------------------------------
// 3. AUXILIARES
// -------------------------------------------------------------------------
#define RVTEST_CODE_END
#define RVTEST_DATA_BEGIN .data
#define RVTEST_DATA_END

#endif
