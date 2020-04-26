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
#define REG_TRANSFORM(x) (((uint64_t)(x)) << 32)
#else
#define REG_TRANSFORM(x) (x)
#endif
#endif
#define HAVE_FP 1
#ifdef __psp__
#define HAVE_VFP 1
#else
#define HAVE_VFP 0
#endif

#define CONTEXT_SIZE 0x300

static thread_local uint64_t co_active_buffer[CONTEXT_SIZE / 8] __attribute__((__aligned__((16))));
static thread_local cothread_t co_active_handle;

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
#if HAVE_VFP
      " sv.q	c000, 0x90($a1), wt\n"
      " sv.q	c010, 0xa0($a1), wt\n"
      " sv.q	c020, 0xb0($a1), wt\n"
      " sv.q	c030, 0xc0($a1), wt\n"
      " sv.q	c100, 0xd0($a1), wt\n"
      " sv.q	c110, 0xe0($a1), wt\n"
      " sv.q	c120, 0xf0($a1), wt\n"
      " sv.q	c130, 0x100($a1), wt\n"
      " sv.q	c200, 0x110($a1), wt\n"
      " sv.q	c210, 0x120($a1), wt\n"
      " sv.q	c220, 0x130($a1), wt\n"
      " sv.q	c230, 0x140($a1), wt\n"
      " sv.q	c300, 0x150($a1), wt\n"
      " sv.q	c310, 0x160($a1), wt\n"
      " sv.q	c320, 0x170($a1), wt\n"
      " sv.q	c330, 0x180($a1), wt\n"
      " sv.q	c400, 0x190($a1), wt\n"
      " sv.q	c410, 0x1a0($a1), wt\n"
      " sv.q	c420, 0x1b0($a1), wt\n"
      " sv.q	c430, 0x1c0($a1), wt\n"
      " sv.q	c500, 0x1d0($a1), wt\n"
      " sv.q	c510, 0x1e0($a1), wt\n"
      " sv.q	c520, 0x1f0($a1), wt\n"
      " sv.q	c530, 0x200($a1), wt\n"
      " sv.q	c600, 0x210($a1), wt\n"
      " sv.q	c610, 0x220($a1), wt\n"
      " sv.q	c620, 0x230($a1), wt\n"
      " sv.q	c630, 0x240($a1), wt\n"
      " sv.q	c700, 0x250($a1), wt\n"
      " sv.q	c710, 0x260($a1), wt\n"
      " sv.q	c720, 0x270($a1), wt\n"
      " sv.q	c730, 0x280($a1), wt\n"
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
#if HAVE_VFP
      " lv.q	c000, 0x90($a0)\n"
      " lv.q	c010, 0xa0($a0)\n"
      " lv.q	c020, 0xb0($a0)\n"
      " lv.q	c030, 0xc0($a0)\n"
      " lv.q	c100, 0xd0($a0)\n"
      " lv.q	c110, 0xe0($a0)\n"
      " lv.q	c120, 0xf0($a0)\n"
      " lv.q	c130, 0x100($a0)\n"
      " lv.q	c200, 0x110($a0)\n"
      " lv.q	c210, 0x120($a0)\n"
      " lv.q	c220, 0x130($a0)\n"
      " lv.q	c230, 0x140($a0)\n"
      " lv.q	c300, 0x150($a0)\n"
      " lv.q	c310, 0x160($a0)\n"
      " lv.q	c320, 0x170($a0)\n"
      " lv.q	c330, 0x180($a0)\n"
      " lv.q	c400, 0x190($a0)\n"
      " lv.q	c410, 0x1a0($a0)\n"
      " lv.q	c420, 0x1b0($a0)\n"
      " lv.q	c430, 0x1c0($a0)\n"
      " lv.q	c500, 0x1d0($a0)\n"
      " lv.q	c510, 0x1e0($a0)\n"
      " lv.q	c520, 0x1f0($a0)\n"
      " lv.q	c530, 0x200($a0)\n"
      " lv.q	c600, 0x210($a0)\n"
      " lv.q	c610, 0x220($a0)\n"
      " lv.q	c620, 0x230($a0)\n"
      " lv.q	c630, 0x240($a0)\n"
      " lv.q	c700, 0x250($a0)\n"
      " lv.q	c710, 0x260($a0)\n"
      " lv.q	c720, 0x270($a0)\n"
      " lv.q	c730, 0x280($a0)\n"
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
   size = (size + CONTEXT_SIZE + 1023) & ~1023;
   cothread_t handle = 0;
#if defined(__APPLE__) || HAVE_POSIX_MEMALIGN >= 1
   if (posix_memalign(&handle, 1024, size) < 0)
      return 0;
#else
   handle = memalign(1024, size);
#endif

   if (!handle)
      return handle;

   uint64_t *ptr = (uint64_t*)handle;
   memset(ptr, 0, CONTEXT_SIZE);
   /* Non-volatiles.  */
   /* ptr[0],..., ptr[7] -> s0,..., s7 */
   store_gp(&ptr[8]); /* gp */
   ptr[9] = REG_TRANSFORM(((uintptr_t)ptr + size - 8)); /* sp  */
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
