
#include <stdio.h>
#include "LPC17xx.H"

void prvStartFirstTask(void);

typedef struct tskTaskControlBlock {
	uint32_t* pxTopOfStack; // contient le registre SP de la pile PSP
	signed int ulState; //peut prendre une valeur negative
	uint32_t ulPriority;
	uint32_t ulTickRDV; //Si tache delayed, pour connaitre le timeout
	uint32_t ulNotifiedValue; //stocke la valeur passee en parametre lors de la notification
	unsigned char ucNotifyState; // stocke le statut courant de la notification
	uint32_t ulInitialPriority;//stocke la priorite initiale et permet l'heritage de priorite en mutex
} tskTCB;
typedef tskTCB TCB_t;

#define taskNOT_WAITING_NOTIFICATION ((uint8_t) 0)
#define taskWAITING_NOTIFICATION ((uint8_t) 1)
#define taskNOTIFICATION_RECEIVED ((uint8_t) 2)

uint32_t ulIndexCurrentTCB;
uint32_t ulTickCount;
uint32_t ulFlag_Yield;
TCB_t * pxCurrentTCB;

#define NB_TASK 4

TCB_t OS_TCB[NB_TASK+1];

#define MinimalStackSize 64

unsigned int stack_tache1[MinimalStackSize];
unsigned int stack_tache2[MinimalStackSize];
unsigned int stack_tache3[MinimalStackSize];
unsigned int stack_tache4[MinimalStackSize];
unsigned int stack_idle[MinimalStackSize];

unsigned int compteur = 0;


int i = 0;
int cpt_it = 0;
unsigned long tempo = 0;
volatile uint8_t  clock_1s = 0;

#define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )
#define portSY_FULL_READ_WRITE		( 15 )

unsigned long uxCriticalNesting = 0;

#define portDISABLE_INTERRUPTS()				vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS()					vPortSetBASEPRI( 0 )

#define configPRIO_BITS       5        /* 32 priority levels */

#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( 5 << (8 - configPRIO_BITS) )

#define ENTER_CRITICAL() vPortEnterCritical()
#define EXIT_CRITICAL() vPortExitCritical()

void prvStartFirstTask(void);

//un mutex
TCB_t* MutexList[NB_TASK+1];
unsigned long WaitingforMutex = 0;
unsigned long MutexState = 0;
TCB_t MutexOwner;

typedef struct FLAG {
	uint32_t FlagType;
	TCB_t* FlagList[NB_TASK+1];
} FLAG;

//atomacité
void vPortRaiseBASEPRI(void) {
	uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

	__asm {
		/* Set BASEPRI to the max syscall priority to effect a critical
		section. */
		msr basepri, ulNewBASEPRI
		dsb
		isb
	}
}

void vPortSetBASEPRI( uint32_t ulBASEPRI ) {
	__asm {
		/* Barrier instructions are not used as this function is only used to
		lower the BASEPRI value. */
		msr basepri, ulBASEPRI
	}
}

void vPortEnterCritical(void) {
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;
}


 void vPortExitCritical(void) {

	uxCriticalNesting--;
	if (uxCriticalNesting == 0) {
		portENABLE_INTERRUPTS();
	}
}

void Task_Resume(void){

	extern unsigned long ticks;

	int i = 0;
	for(i=0; i<5; i++) {
		if (ticks == OS_TCB[i].ulTickRDV) {
			OS_TCB[i].ulState=OS_TCB[i].ulPriority + 2;
			ulFlag_Yield = 1;
		}
	}
	i = 0;
}

void Switch_Task(void){

	int IndexHighPriority = 0; //idle par défaut
	unsigned long Priority = 2;

	int i = (ulIndexCurrentTCB)%(NB_TASK +1);
	int oldtcb = i;

	ENTER_CRITICAL();

	do {
		if (OS_TCB[i].ulState >= Priority) {
			Priority = OS_TCB[i].ulState;
			IndexHighPriority = i;
		}
		i=(i+1)%(NB_TASK+1);
	}
	while (i != (ulIndexCurrentTCB)%(NB_TASK +1));

	pxCurrentTCB = &OS_TCB[IndexHighPriority];
	ulIndexCurrentTCB = IndexHighPriority;

	if (&OS_TCB[IndexHighPriority] != &OS_TCB[oldtcb]) {
		ulFlag_Yield = 1;
	}
	else {
		ulFlag_Yield =0;
	}

	EXIT_CRITICAL();
}


//P3.25 led verte //P3.26 led bleu // P0.22 led rouge
void init_gpio() {
	LPC_SC->PCONP |= (1 << 15);/* power on sur GPIO & IOCON */
	LPC_GPIO3->FIODIR |= 0x03<<25;
	//P3.25 et P3.26 en sortie
	LPC_GPIO0->FIODIR |= 0x01<<22;
	//P0.22 en sortie
}

// une fonction
void actualise_ledV(int consV) {
	if (consV) {LPC_GPIO3->FIOSET = 1<<25;}
	else {LPC_GPIO3->FIOCLR = 1<<25;}
}

// une fonction
void actualise_ledB(int consB) {
	if (consB) {LPC_GPIO3->FIOSET = 1<<26;}
	else {LPC_GPIO3->FIOCLR = 1<<26;}
}

// une fonction
void actualise_ledR(int consR) {
	if (consR) {LPC_GPIO0->FIOSET = 1<<22;}
	else {LPC_GPIO0->FIOCLR = 1<<22;}
}

void init_timer0() {
	LPC_SC->PCONP     |= (1 << 1);
	LPC_TIM0->TCR =0x03;
	LPC_TIM0->CTCR =0x00;
	LPC_TIM0->MR0 =128; // 25000000/5;
	LPC_TIM0->MCR = 0x03;
	NVIC_EnableIRQ(TIMER0_IRQn);
	LPC_TIM0->TCR = 0x01;
}

void TIMER0_IRQHandler(void) {
	cpt_it++;
	actualise_ledR(cpt_it);
	LPC_TIM0->IR = 1; //acquittement
}

void init_proc() {
	init_gpio();

}

void Task_Yield() {

	//appeler commutation de tâche
	portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;	//interruption PendSV_Handler pour appeler une commutation de tache
	__dsb( portSY_FULL_READ_WRITE );
	__isb( portSY_FULL_READ_WRITE );
}

void Task_delay(unsigned long delay) {
	extern unsigned long ticks;
	ENTER_CRITICAL();

	if (delay == 0xFFFFFFFF) {
		OS_TCB[ulIndexCurrentTCB].ulState = 0;
	}
	else {
		OS_TCB[ulIndexCurrentTCB].ulTickRDV= delay + ticks;
		OS_TCB[ulIndexCurrentTCB].ulState = 1;
	}
	EXIT_CRITICAL();
	Task_Yield();

}

void xTaskNotifyGiveCPT(TCB_t *TCB_task) {

	ENTER_CRITICAL();
	TCB_task->ulNotifiedValue++;

	if (TCB_task->ucNotifyState == taskWAITING_NOTIFICATION) {
		TCB_task->ulState = TCB_task->ulPriority +2;//revient a la priorite normale de la tache
	}

	TCB_task->ucNotifyState = taskNOTIFICATION_RECEIVED;
	EXIT_CRITICAL();
	//lui donner la main si elle a une priorite plus importante que la notre
	if (TCB_task->ulState > OS_TCB[ulIndexCurrentTCB].ulState) {
		Task_Yield();
	}
}

unsigned long xTaskNotifyTakeCPT(unsigned long TicksToWait) {

	unsigned long ulreturn = OS_TCB[ulIndexCurrentTCB].ulNotifiedValue;

	ENTER_CRITICAL();
	if (OS_TCB[ulIndexCurrentTCB].ucNotifyState == taskNOT_WAITING_NOTIFICATION) {

		OS_TCB[ulIndexCurrentTCB].ucNotifyState = taskWAITING_NOTIFICATION;
		OS_TCB[ulIndexCurrentTCB].ulState = 0; //passage en liste suspended
		//si il y a un timeout
		if (TicksToWait > 0) {
			Task_delay(TicksToWait); //on passe en delai et on yield
			// si 0xFFFFFF alors task suspended
		}
	}
	else if (OS_TCB[ulIndexCurrentTCB].ucNotifyState == taskNOTIFICATION_RECEIVED){

		if (OS_TCB[ulIndexCurrentTCB].ulNotifiedValue == 0) {
			OS_TCB[ulIndexCurrentTCB].ucNotifyState = taskNOT_WAITING_NOTIFICATION;
		}

		OS_TCB[ulIndexCurrentTCB].ulNotifiedValue--;

	}
	EXIT_CRITICAL();
	return ulreturn;
}

unsigned long xTaskNotifyTakeBIN(unsigned long TicksToWait) {
	//variable pour signaler si on sort à cause d'un delay ou d'une notification
	unsigned long ulreturn = OS_TCB[ulIndexCurrentTCB].ulNotifiedValue;

	ENTER_CRITICAL();
	if (OS_TCB[ulIndexCurrentTCB].ucNotifyState == taskNOT_WAITING_NOTIFICATION) {

		OS_TCB[ulIndexCurrentTCB].ucNotifyState = taskWAITING_NOTIFICATION;
		OS_TCB[ulIndexCurrentTCB].ulState = 0; //passage en liste suspended
		//si il y a un timeout
		if (TicksToWait > 0) {
			Task_delay(TicksToWait); //on passe en delai et on yield
			// si 0xFFFFFF alors task suspended
		}
	}
	else if(OS_TCB[ulIndexCurrentTCB].ucNotifyState == taskWAITING_NOTIFICATION){
		OS_TCB[ulIndexCurrentTCB].ucNotifyState = taskNOT_WAITING_NOTIFICATION;
		OS_TCB[ulIndexCurrentTCB].ulNotifiedValue = 0;
	}
	EXIT_CRITICAL();
	return ulreturn;
}


void xTaskNotifyGiveFLAG(FLAG* Flag) {

	unsigned int i = 0;
	unsigned long SuperiorPriority=0;
	ENTER_CRITICAL();

	for (i=0 ; i<NB_TASK ; i++) {

		Flag->FlagList[i]->ulState = Flag->FlagList[i]->ulPriority+2; //priorite normale
		Flag->FlagList[i]->ucNotifyState = taskNOT_WAITING_NOTIFICATION;

		if (Flag->FlagList[i]->ulState >= OS_TCB[ulIndexCurrentTCB].ulState) {
			SuperiorPriority = 1;
		}
		Flag->FlagList[i] = NULL;//on vide notre liste car on a recu notre flag
	}
	//si une des taches reveillee est superieure en priorite on yield
	if (SuperiorPriority == 1) {
		Task_Yield();
	}
	EXIT_CRITICAL();
}

//La tache s'inscrit a un Flag et met en place un timeout (ou non -> si timeout infini rien ou si valeur 0)
unsigned long xTaskNotifyTakeFLAG(FLAG* FlagType,unsigned long TicksToWait) {

	unsigned long ulreturn = 0;//0 ou 1 , avec 0 = on sort a cause du flag avec 1 = on sort a cause du timeout
	unsigned long i = 0;
	TCB_t* LocalFlagList[5];

	//La tache s'inscrit a une flagList
	//on attend une notifiction
	ENTER_CRITICAL();
	OS_TCB[ulIndexCurrentTCB].ucNotifyState = taskWAITING_NOTIFICATION;
	//on s'endort
	OS_TCB[ulIndexCurrentTCB].ulState = 0; //suspended
	//On s'inscrit en debut de liste
	LocalFlagList[i] = &OS_TCB[ulIndexCurrentTCB];
	for (i=1; i<5; i++) {
		LocalFlagList[i] = FlagType->FlagList[i-1]; // on copie dans le local flag list
	}
	for (i=0; i<5; i++) {
		FlagType->FlagList[i] = LocalFlagList[i]; //on met a jour notre flaglist
	}
	//on met en place un timeout

	EXIT_CRITICAL();
	if (TicksToWait > 0) {
		Task_delay(TicksToWait);
		ulreturn=1;
	}
	return ulreturn;
}

void xMutexTake(void) {

	unsigned long CurrentPriority = OS_TCB[ulIndexCurrentTCB].ulState;

	unsigned long MutexIndex = 0;
	unsigned long MutexIndex2 = 0;
	unsigned long i = 0;
	unsigned long j = 0;

	TCB_t* MutexArray[NB_TASK+1];

	ENTER_CRITICAL();
	if (MutexState == 0) {
		MutexState = 1;
		MutexOwner = OS_TCB[ulIndexCurrentTCB];
	}

	else {
		WaitingforMutex ++;

		//Mise a jour tableau
		//si on est de priorite superieure on passe devant
		for(i = 0; i<WaitingforMutex; i++) {
			if (MutexList[i]->ulPriority+2 > CurrentPriority) {
				MutexIndex = i;// on sait derriere quel index on doit se placer
				MutexIndex2 = i;
				MutexArray[i] = MutexList[i]; //les tableaux sont similaires jusqu'a l'introduction
			}
		}
		MutexIndex = MutexIndex+1; //On doit se placer a la case i+1
		MutexArray[MutexIndex] = &OS_TCB[ulIndexCurrentTCB];//on intercale notre propre tcb

		for (j = MutexIndex+1; j < WaitingforMutex; j++) {
			MutexArray[j] = MutexList[MutexIndex2+1];
		}

		for (i = 0; i<WaitingforMutex; i++) {
			MutexList[i] = MutexArray[i];//on remplit le tableau en copiant l'autre tableau
		}

		OS_TCB[ulIndexCurrentTCB].ulState = 0; //tache endormie
		//heritage
		//on met le possesseur du mutex a notre meme niveau de priorite
		MutexOwner.ulPriority = CurrentPriority;
	}
	EXIT_CRITICAL();
}

void xMutexGive(TCB_t tcb) {

	unsigned int i = 0;

	ENTER_CRITICAL();
	if (MutexList[0] != &OS_TCB[0]) {
		MutexOwner.ulPriority = MutexOwner.ulInitialPriority;
		MutexOwner = *MutexList[0];
		MutexOwner.ulState = MutexOwner.ulPriority+2; //niveau de priorite normale pour notre tache
		WaitingforMutex--;//on depile!

		for (i=0; i<WaitingforMutex; i++) {
			MutexList[i] = MutexList[i+1];
		}
	}
	else {
		//Je propose de remettre le mutex dans un etat "libre" lorsqu'il n'y a que la tache idle en attente
		MutexState = 0;
	}
	EXIT_CRITICAL();
}

void Task_idle(void ) {

	while(1) {
		Task_Yield(); //armer l'IT pending_SV
	}
}


void Task_kill() { // on devra arriver ici si une tache quitte


	OS_TCB[ulIndexCurrentTCB].ulState = -1; //changement de l'etat de la tache (comme si notre tache avait ete ajoutee a la liste xTasksWaitingTermination)
	// la task kill va commuter et mettre fin tache
	Task_Yield(); // Commutation avec PendingSVC Handler
}


//Led rouge
void Task_LEDR (void * Parameters) {
	while(1){
		//allumer 0.3 s
		actualise_ledR(1);
		Task_delay(300);
		actualise_ledR(0);
		//eteindre 700 ms
		Task_delay(700);
	}
}

void Task_LEDV (void * Parameters) {
	while(1) {
		actualise_ledV(1);
		Task_delay(100);
		actualise_ledV(0);
		Task_delay(300);
	}
}

void Task_Scrutation (void * Parameters) {
	unsigned char bouton;
	static unsigned char old_bouton = 0;
	bouton = ((LPC_GPIO0->FIOPIN)&(1<<10))?1:0;
	while(1) {
		bouton = ((LPC_GPIO0->FIOPIN)&(1<<10))?1:0;
		if (old_bouton && !bouton) {
			compteur++;
			if (compteur>3){
				compteur=0;
			}

		}
		old_bouton=bouton;
		Task_delay(100);
	}
}

void Task_LEDB (void * Parameters) {
	while(1) {
		switch(compteur) {
			case 0: // 1 Hz
				actualise_ledB(1);
				Task_delay(100);
				actualise_ledB(0);
				Task_delay(900);
				break;
			case 1: // 2 Hz
				actualise_ledB(1);
				Task_delay(100);
				actualise_ledB(0);
				Task_delay(400);
				break;
			case 2: // 3 Hz
				actualise_ledB(1);
				Task_delay(100);
				actualise_ledB(0);
				Task_delay(233);
				break;
			case 3: // 4 Hz
				actualise_ledB(1);
				Task_delay(100);
				actualise_ledB(0);
				Task_delay(150);
				break;
		}
	}
}


//--------------- Main Program -----------------------------------
void Task_create(uint32_t ulStackDepth,
				 void* pvParameters,
				 uint32_t uxPriority,
				 TCB_t *TCB_task,
				 unsigned int* stack_task,
				 void* pxTaskCode) {

	// preparer le TCB
	uint32_t* pxTopOfStack = stack_task + ulStackDepth;
	TCB_task->pxTopOfStack = stack_task + ulStackDepth;

	TCB_task->ulPriority = uxPriority; //priorite
	TCB_task->ulTickRDV = 0xFFFFFFFF; //delay infini
	TCB_task->ulState = uxPriority + 2; //pas de liste d'abonnement donc 2 pour etre dans la ready list / etat
	TCB_task->ucNotifyState = 0; //Not waiting notification
	TCB_task->ulNotifiedValue = 0;
	TCB_task->ulInitialPriority = uxPriority;// on initialise la priorite

	pxTopOfStack--;
	*pxTopOfStack = 0x01000000; //xPSP
	pxTopOfStack--;
	*pxTopOfStack = (unsigned long)pxTaskCode; // PC et R15
	pxTopOfStack--;
	*pxTopOfStack = (unsigned long)Task_kill; // LR and error exit force R14
	pxTopOfStack--;
	*pxTopOfStack = 0x12121212; //R12
	pxTopOfStack--;
	*pxTopOfStack = 0x03030303; //R3
	pxTopOfStack--;
	*pxTopOfStack = 0x02020202; //R2
	pxTopOfStack--;
	*pxTopOfStack = 0x01010101; //R1
	pxTopOfStack--;
	*pxTopOfStack = (unsigned long)pvParameters; //R0
	pxTopOfStack--;
	*pxTopOfStack = 0x11111111;
	pxTopOfStack--;
	*pxTopOfStack = 0x10101010;
	pxTopOfStack--;
	*pxTopOfStack = 0x09090909;
	pxTopOfStack--;
	*pxTopOfStack = 0x08080808;
	pxTopOfStack--;
	*pxTopOfStack = 0x07070707;
	pxTopOfStack--;
	*pxTopOfStack = 0x06060606;
	pxTopOfStack--;
	*pxTopOfStack = 0x05050505;
	pxTopOfStack--;
	*pxTopOfStack = 0x04040404;
	TCB_task->pxTopOfStack = pxTopOfStack;
}

void creation_des_taches(void) {
	Task_create(64,NULL,0,&OS_TCB[0], stack_idle, &Task_idle);

	Task_create(64,NULL,1,&OS_TCB[1], stack_tache1, &Task_Scrutation); //Scrute tous les 10 Hz -> pour respecter 10 Hz on met au meme niveau de priorite

	Task_create(64,NULL,1,&OS_TCB[2], stack_tache2, &Task_LEDB);

	Task_create(64,NULL,1,&OS_TCB[3], stack_tache3, &Task_LEDV);

	Task_create(64,NULL,1,&OS_TCB[4], stack_tache4, &Task_LEDR);//priorite superieure a idle
}

void lancement_OS(void) {

	ulIndexCurrentTCB = 0;
	ulTickCount = 0;
	ulFlag_Yield = 0;
	pxCurrentTCB = &OS_TCB[ulIndexCurrentTCB];

	MutexList[0]=&OS_TCB[0];

	SysTick_Config(100000);

	NVIC_SetPriority (SVCall_IRQn, 31);
	NVIC_SetPriority (PendSV_IRQn, 31);
	NVIC_SetPriority (SysTick_IRQn, 31);

	prvStartFirstTask();

}

void gestion_plantage_OS(void) {
	while(1) ;
}

int main (void) {
	init_proc();

	creation_des_taches();
	lancement_OS();

	gestion_plantage_OS();
}
