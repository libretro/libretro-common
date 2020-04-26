/*
  libco.mips (2020-04-20)
  author: Google
  license: MIT
*/

#define LIBCO_C
#include <libco.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef __APPLE__
#include <malloc.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

static thread_local uint64_t co_active_buffer[64];
static thread_local cothread_t co_active_handle;

#if __mips >= 3
/* If we have 64-bit registers.  */
#define STORE_REG "sd"
#define LOAD_REG "ld"
#define REG_TRANSFORM(x) (x)
#else
/* 32-bit only variant.  */
#define STORE_REG "sw"
#define LOAD_REG "lw"
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define REG_TRANSFORM(x) (((uint64_t)(x)) << 64)
#else
#define REG_TRANSFORM(x) (x)
#endif
#endif
#define HAVE_FP 1

__asm__ (
      ".align 4\n"
      ".globl co_switch_mips\n"
      ".globl _co_switch_mips\n"
      "co_switch_mips:\n"
      "_co_switch_mips:\n"
      STORE_REG " $s0, 0($a1)\n"
      STORE_REG " $s1, 8($a1)\n"
      STORE_REG " $s2, 0x10($a1)\n"
      STORE_REG " $s3, 0x18($a1)\n"
      STORE_REG " $s4, 0x20($a1)\n"
      STORE_REG " $s5, 0x28($a1)\n"
      STORE_REG " $s6, 0x30($a1)\n"
      STORE_REG " $s7, 0x38($a1)\n"
      STORE_REG " $gp, 0x40($a1)\n"
      STORE_REG " $sp, 0x48($a1)\n"
      STORE_REG " $fp, 0x50($a1)\n"
      STORE_REG " $ra, 0x58($a1)\n"
#if HAVE_FP
      " swc1 $f20, 0x60($a1)\n"
      " swc1 $f21, 0x64($a1)\n"
      " swc1 $f22, 0x68($a1)\n"
      " swc1 $f23, 0x6c($a1)\n"
      " swc1 $f24, 0x70($a1)\n"
      " swc1 $f25, 0x74($a1)\n"
      " swc1 $f26, 0x78($a1)\n"
      " swc1 $f27, 0x7c($a1)\n"
      " swc1 $f28, 0x80($a1)\n"
      " swc1 $f29, 0x84($a1)\n"
      " swc1 $f30, 0x88($a1)\n"
#endif
      LOAD_REG " $s0, 0($a0)\n"
      LOAD_REG " $s1, 8($a0)\n"
      LOAD_REG " $s2, 0x10($a0)\n"
      LOAD_REG " $s3, 0x18($a0)\n"
      LOAD_REG " $s4, 0x20($a0)\n"
      LOAD_REG " $s5, 0x28($a0)\n"
      LOAD_REG " $s6, 0x30($a0)\n"
      LOAD_REG " $s7, 0x38($a0)\n"
      LOAD_REG " $gp, 0x40($a0)\n"
      LOAD_REG " $sp, 0x48($a0)\n"
      LOAD_REG " $fp, 0x50($a0)\n"
      LOAD_REG " $ra, 0x58($a0)\n"
#if HAVE_FP
      " lwc1 $f20, 0x60($a0)\n"
      " lwc1 $f21, 0x64($a0)\n"
      " lwc1 $f22, 0x68($a0)\n"
      " lwc1 $f23, 0x6c($a0)\n"
      " lwc1 $f24, 0x70($a0)\n"
      " lwc1 $f25, 0x74($a0)\n"
      " lwc1 $f26, 0x78($a0)\n"
      " lwc1 $f27, 0x7c($a0)\n"
      " lwc1 $f28, 0x80($a0)\n"
      " lwc1 $f29, 0x84($a0)\n"
      " lwc1 $f30, 0x88($a0)\n"
#endif
      " jr $ra\n"
      "  nop\n"
      ".align 4\n"
      ".globl store_gp\n"
      ".globl _store_gp\n"
      "store_gp:\n"
      "_store_gp:\n"
      STORE_REG " $gp, 0($a0)\n"
      " jr $ra\n"
      "  nop\n"      
    );

/* ASM */
void co_switch_mips(cothread_t handle, cothread_t current);
void store_gp(uint64_t *s);

cothread_t co_create(unsigned int size, void (*entrypoint)(void))
{
   size = (size + 1023) & ~1023;
   cothread_t handle = 0;
#if defined(__APPLE__) || HAVE_POSIX_MEMALIGN >= 1
   if (posix_memalign(&handle, 1024, size + 512) < 0)
      return 0;
#else
   handle = memalign(1024, size + 512);
#endif

   if (!handle)
      return handle;

   uint64_t *ptr = (uint64_t*)handle;
   memset(ptr, 0, 512);
   /* Non-volatiles.  */
   /* ptr[0],..., ptr[7] -> s0,..., s7 */
   store_gp(&ptr[8]); /* gp */
   ptr[9] = REG_TRANSFORM(((uintptr_t)ptr + size + 512 - 8)); /* sp  */
   /* ptr[10] is fp */
   ptr[11] = REG_TRANSFORM((uintptr_t)entrypoint); /* ra */
   return handle;
}

cothread_t co_active(void)
{
   if (!co_active_handle)
      co_active_handle = co_active_buffer;
   return co_active_handle;
}

void co_delete(cothread_t handle)
{
   free(handle);
}

void co_switch(cothread_t handle)
{
   cothread_t co_previous_handle = co_active();
   co_switch_mips(co_active_handle = handle, co_previous_handle);
}

#ifdef __cplusplus
}
#endif
