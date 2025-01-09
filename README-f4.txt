README-f4.txt
01/09/25
Notes on CAN loading with F4 Discovery

Problem:
The app (e.g. shaft_encoder2) bombs after entry from the
ldrfixedDiscoveryf4.

Likely the problem comes with the latest STM32CubeMX version.

The SystemInit is called from the .S startup routine and it resets
the vector table to 0x08000000. The CAN loader routine relocates
the vector table to 0x08010000 before the jump, but the after
the jump the init routine resets the vector table address. 

In ../Src/system_stm32f4xx.c

line 195--
  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

Add this after the above--
    SCB->VTOR = 0x08010000;

Problem:
Watchdog
The 'init' of the watchdog in the beginning (of ldrfixedDiscveryf4) needs to be commented out.
MX sets the init, but IWDG is set immediately, and it is not to be set until later
in the loop waiting for incoming loader msgs. Net--MX needs to 'activate' IWDG, but
the init at the startup needs to be commented out.



