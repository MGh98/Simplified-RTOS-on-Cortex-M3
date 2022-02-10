

#include "LPC17xx.H"

extern volatile uint8_t  clock_1s;

unsigned long ticks = 10;

typedef struct tskTaskControlBlock {
	unsigned int* pxTopOfStack;
	signed int ulState;
	unsigned long ulPriority;
	unsigned long ulTickRDV;
} tskTCB;
typedef tskTCB TCB_t;

extern volatile uint32_t ulIndexCurrentTCB;
extern volatile uint32_t ulTickCount;
extern volatile uint32_t ulFlag_Yield; //pour declencher Task_Yield et donc la commutation de tache
extern TCB_t * pxCurrentTCB; // TCB tache en cours
extern TCB_t* OS_TCB;

#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( 5 << 3 )
#define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )

unsigned long overflow = 0;

void Switch_Task(void);
void Task_Resume(void);

void SysTick_Handler (void) {

	Task_Resume();
	if (ulFlag_Yield == 1) {
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	}
	ticks++;

}

__asm void PendSV_Handler (void) {

	extern Switch_Task

	//sauvegarde de contexte
	PRESERVE8

	mrs r0, psp
	isb

	ldr	r3, =pxCurrentTCB		/* Get the location of the current TCB. */
	ldr	r2, [r3]

	stmdb r0!, {r4-r11}			/* Save the remaining registers. */
	str r0, [r2]				/* Save the new top of stack into the first member of the TCB. */

	stmdb sp!, {r3, r14}
	mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
	msr basepri, r0
	dsb
	isb
	bl Switch_Task
	mov r0, #0
	msr basepri, r0
	ldmia sp!, {r3, r14}


	ldr r1, [r3]
	ldr r0, [r1]				/* The first item in pxCurrentTCB is the task top of stack. */
	ldmia r0!, {r4-r11}			/* Pop the registers and the critical nesting count. */
	msr psp, r0
	isb
	bx r14
	nop
}


__asm void SVC_Handler (void) {

	extern pxCurrentTCB
	PRESERVE8

	ldr	r3, =pxCurrentTCB	/* Restore the context. */
	ldr r1, [r3]			/* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
	ldr r0, [r1]			/* The first item in pxCurrentTCB is the task top of stack. */
	ldmia r0!, {r4-r11}		/* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
	msr psp, r0				/* Restore the task stack pointer. */
	isb
	mov r0, #0
	msr	basepri, r0
	//mov r0, #1

	//	msr control, r0
	orr r14, #0xd
	bx r14
}

__asm void prvStartFirstTask( void ) {
	PRESERVE8

	cpsie i
	cpsie f
	dsb
	isb
	/* Call SVC to start the first task. */
	svc 0
	nop
	nop
}
