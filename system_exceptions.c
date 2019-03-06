#include "pin_mapping.h"
#include "main.h"				// Para tener acceso a las variables compartidas con el bootloader


static unsigned int _excep_code;
static unsigned int _excep_addr;

// Declared static in case exception condition would prevent
// an auto variable from being created
static enum {
	EXCEP_IRQ = 0,      // Interrupt
	EXCEP_AdEL = 4,     // Address Error Exception (load or ifetch)
	EXCEP_AdES,         // Address Error Exception (store)
	EXCEP_IBE,          // Instruction Bus Error (ifetch)
	EXCEP_DBE,          // Data Bus Error (load/store)		// Lo más probable es que esté escribiendo en un puntero no inicializado, que es cero y no es una dirección válida en RAM
	EXCEP_Sys,          // Syscall
	EXCEP_Bp,           // Breakpoint
	EXCEP_RI,           // Reserved Instruction
	EXCEP_CpU,          // Coprocessor Unusable
	EXCEP_Overflow,     // Arithmetic Overflow
	EXCEP_Trap,         // Trap (possible divide by zero)
	EXCEP_IS1 = 16,     // Implementation Specfic 1
	EXCEP_CEU,          // CorExtend Unuseable
	EXCEP_C2E           // Coprocessor 2
}_excep_code;

// This function overrides the normal _weak_ generic handler
void _general_exception_handler(void)
{
	asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_excep_code & 0x0000007C) >> 2;

	mcrLED_R_On

	while (1)
	{
//		wait_s(30);
//		ResetMicro();
	// Examine _excep_code to identify the type of exception
	// Examine _excep_addr to find the address that caused the exception
	}
}
