#include "socket_util.h"

#define SNAPSHOT_HANDLING_ASM()                             \
  __asm volatile(                                           \
      "push {r0-r12, lr} \n"                                \
      "mrs r0, msp\n" /* save abnormal registers */         \
      "push {r0}\n"                                         \
      "mrs r0, psp\n"                                       \
      "push {r0}\n"                                         \
      "ldr r0, =0xE000E100\n" /* save NVIC_ISER registers*/ \
      "ldr r1, [r0]\n"                                      \
      "push {r1}\n"                                         \
      "ldr r1, [r0, #4]\n"                                  \
      "push {r1}\n"                                         \
      "ldr r1, [r0, #8]\n"                                  \
      "push {r1}\n"                                         \
      "ldr r1, [r0, #12]\n"                                 \
      "push {r1}\n"                                         \
      "mov r0, sp\n" /*use stack as struct*/                \
      "bl snapshot_handler \n"                              \
      "add sp, sp, #24\n"  /* recover msp,psp,NVIC_ISER */  \
      "pop {r0, r1}\n"     /* recover r0, r1 */             \
      "add sp, sp, #48\n") /* recover r2-r12,lr */

// SET the RAM start and end from linker script
extern uint32_t __dtcm_start[];
#define RAM_START ((uint32_t)__dtcm_start)
extern uint32_t __dtcm_end[];
#define RAM_END ((uint32_t)__dtcm_end)

struct RegState {
  // specific registers
  uint32_t NVIC_ISER3;
  uint32_t NVIC_ISER2;
  uint32_t NVIC_ISER1;
  uint32_t NVIC_ISER0;
  uint32_t psp;
  uint32_t msp;
  // normal registers
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t r12;
  uint32_t lr;
  // recover registers
  uint32_t pc;
  uint32_t sp;
};

/// @brief Using differenct protocol to send snapshot
/// @param region the memory region
/// @param len length
inline void send_snapshot(const char *region, uint32_t len) {
  cam_send((void *)region, len);
}

void snapshot_handler(struct RegState *regs) {
  // reset pc & sp;
  __asm__ volatile("mov %0, lr" : "=r"(regs->pc));
  regs->sp = (uint32_t)regs;
  // reset msp & psp
  uint32_t control_ = 0;
  __asm__ volatile("mrs %0, control" : "=r"(control_));
  if (control_ & 0x2) {
    regs->psp = regs->sp;
  } else {
    regs->msp = regs->sp;
  }

  send_snapshot((char *)regs, sizeof(struct RegState));
  send_snapshot((char *)RAM_START, RAM_END - RAM_START);
}