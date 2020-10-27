
MAKEFLAGS = -j8

XOPTS = -I"C:/ti/xdctools_3_50_02_20_core/packages/" -I"C:/ti/xdctools_3_50_02_20_core/packages/" -Dxdc_target_types__=C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/iar/targets/arm/std.h -Dxdc_target_name__=M3

vpath % C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/
vpath %.c C:/ti/xdctools_3_50_02_20_core/packages/

CCOPTS    = --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc -Ohz  -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_knl_Clock_stopCheckNext__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_disablePriority__D=32U -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U  --dlib_config C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/inc/c/DLib_Config_Normal.h

XDC_ROOT  = C:/ti/xdctools_3_50_02_20_core/packages/

BIOS_ROOT = C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/

BIOS_DEFS =  -Dti_sysbios_BIOS_swiEnabled__D=TRUE -Dti_sysbios_BIOS_taskEnabled__D=TRUE -Dti_sysbios_BIOS_clockEnabled__D=TRUE -Dti_sysbios_BIOS_runtimeCreatesEnabled__D=TRUE -Dti_sysbios_knl_Task_moduleStateCheckFlag__D=FALSE -Dti_sysbios_knl_Task_objectCheckFlag__D=FALSE -Dti_sysbios_hal_Hwi_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Swi_DISABLE_ALL_HOOKS -Dti_sysbios_BIOS_smpEnabled__D=FALSE -Dti_sysbios_Build_useHwiMacros -Dti_sysbios_Build_useIndirectReferences=FALSE -Dti_sysbios_knl_Swi_numPriorities__D=6 -Dti_sysbios_knl_Task_deleteTerminatedTasks__D=FALSE -Dti_sysbios_knl_Task_numPriorities__D=6 -Dti_sysbios_knl_Task_checkStackFlag__D=FALSE -Dti_sysbios_knl_Task_initStackFlag__D=TRUE -Dti_sysbios_knl_Task_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Clock_TICK_SOURCE=ti_sysbios_knl_Clock_TickSource_TIMER -Dti_sysbios_knl_Clock_TICK_MODE=ti_sysbios_knl_Clock_TickMode_DYNAMIC -Dti_sysbios_hal_Core_delegate_getId=ti_sysbios_hal_CoreNull_getId__E -Dti_sysbios_hal_Core_delegate_interruptCore=ti_sysbios_hal_CoreNull_interruptCore__E -Dti_sysbios_hal_Core_delegate_lock=ti_sysbios_hal_CoreNull_lock__E -Dti_sysbios_hal_Core_delegate_unlock=ti_sysbios_hal_CoreNull_unlock__E -Dti_sysbios_hal_Core_numCores__D=1 -Dti_sysbios_hal_CoreNull_numCores__D=1 -Dti_sysbios_utils_Load_taskEnabled__D=TRUE -Dti_sysbios_utils_Load_swiEnabled__D=FALSE -Dti_sysbios_utils_Load_hwiEnabled__D=FALSE -Dti_sysbios_family_arm_m3_Hwi_dispatcherSwiSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherTaskSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherAutoNestingSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherIrpTrackingSupport__D=TRUE -Dti_sysbios_knl_Semaphore_supportsEvents__D=FALSE -Dti_sysbios_knl_Semaphore_supportsPriority__D=FALSE -Dxdc_runtime_Assert_DISABLE_ALL -Dxdc_runtime_Log_DISABLE_ALL -DBOARD_DISPLAY_USE_LCD=1 -DBOARD_DISPLAY_USE_UART=1 -DBOARD_DISPLAY_USE_UART_ANSI=1 -DCC2640R2_LAUNCHXL -DCC26XX -DCC26XX_R2 -DDeviceFamily_CC26X0R2 -DxDisplay_DISABLE_ALL -DICALL_EVENTS -DICALL_JT -DICALL_LITE -DICALL_MAX_NUM_ENTITIES=6 -DICALL_MAX_NUM_TASKS=3 -DICALL_STACK0_ADDR -DMAX_NUM_BLE_CONNS=1 -DPOWER_SAVING -DRF_SINGLEMODE -DSTACK_LIBRARY -DTBM_ACTIVE_ITEMS_ONLY -DUSE_ICALL -Dxdc_runtime_Assert_DISABLE_ALL -Dxdc_runtime_Log_DISABLE_ALL 

ASM_DEFS =  -Dti_sysbios_BIOS_swiEnabled__D=TRUE -Dti_sysbios_BIOS_taskEnabled__D=TRUE -Dti_sysbios_BIOS_clockEnabled__D=TRUE -Dti_sysbios_BIOS_runtimeCreatesEnabled__D=TRUE -Dti_sysbios_knl_Task_moduleStateCheckFlag__D=FALSE -Dti_sysbios_knl_Task_objectCheckFlag__D=FALSE -Dti_sysbios_hal_Hwi_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Swi_DISABLE_ALL_HOOKS -Dti_sysbios_BIOS_smpEnabled__D=FALSE -Dti_sysbios_Build_useHwiMacros -Dti_sysbios_Build_useIndirectReferences=FALSE -Dti_sysbios_knl_Swi_numPriorities__D=6 -Dti_sysbios_knl_Task_deleteTerminatedTasks__D=FALSE -Dti_sysbios_knl_Task_numPriorities__D=6 -Dti_sysbios_knl_Task_checkStackFlag__D=FALSE -Dti_sysbios_knl_Task_initStackFlag__D=TRUE -Dti_sysbios_knl_Task_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Clock_TICK_SOURCE=ti_sysbios_knl_Clock_TickSource_TIMER -Dti_sysbios_knl_Clock_TICK_MODE=ti_sysbios_knl_Clock_TickMode_DYNAMIC -Dti_sysbios_hal_Core_delegate_getId=ti_sysbios_hal_CoreNull_getId__E -Dti_sysbios_hal_Core_delegate_interruptCore=ti_sysbios_hal_CoreNull_interruptCore__E -Dti_sysbios_hal_Core_delegate_lock=ti_sysbios_hal_CoreNull_lock__E -Dti_sysbios_hal_Core_delegate_unlock=ti_sysbios_hal_CoreNull_unlock__E -Dti_sysbios_hal_Core_numCores__D=1 -Dti_sysbios_hal_CoreNull_numCores__D=1 -Dti_sysbios_utils_Load_taskEnabled__D=TRUE -Dti_sysbios_utils_Load_swiEnabled__D=FALSE -Dti_sysbios_utils_Load_hwiEnabled__D=FALSE -Dti_sysbios_family_arm_m3_Hwi_dispatcherSwiSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherTaskSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherAutoNestingSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherIrpTrackingSupport__D=TRUE -Dti_sysbios_knl_Semaphore_supportsEvents__D=FALSE -Dti_sysbios_knl_Semaphore_supportsPriority__D=FALSE -Dxdc_runtime_Assert_DISABLE_ALL -Dxdc_runtime_Log_DISABLE_ALL

INTERN_FUNCS = ""

BIOS_INC  = -I"C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/"

INCS = $(BIOS_INC) -I"C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/" -I"C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/source/" -I"C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/source/ti/ble5stack/" -I"C:/ti/xdctools_3_50_02_20_core/packages/" -I"../" -I"/packages//" 

CC = LC_ALL=C  C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/bin/iccarm $(CCOPTS)
ASM = LC_ALL=C  C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/bin/iasmarm --cpu Cortex-M3 --endian little -S
AR = LC_ALL=C  C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/bin/iarchive 

CC_LIB = C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/bin/iccarm --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc -Ohz  -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_knl_Clock_stopCheckNext__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_disablePriority__D=32U -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U -I C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/include

CC_PP = C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/bin/iccarm --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc   -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_knl_Clock_stopCheckNext__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE  -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U -I C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/include  --dlib_config C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm/inc/c/DLib_Config_Normal.h

DEL = C:/ti/xdctools_3_50_02_20_core/packages/../bin/rm -f
CP = C:/ti/xdctools_3_50_02_20_core/packages/../bin/cp -f

define RM
	$(if $(wildcard $1),$(DEL) $1,:)
endef


all: rom_sysbios.arm3

include C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/rom/cortexm/cc26xx/r2/golden/CC26xx/symbolMapFile.mak

rom_arm_m3_Hwi_asm_iar_lib.obj: family/arm/m3/Hwi_asm_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/rom/cortexm/cc26xx/r2/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/Hwi_asm_iar.sv7M rom_arm_m3_Hwi_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_Hwi_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(ASM_DEFS) $(XOPTS) $(INCS) rom_arm_m3_Hwi_asm_iar_lib.asm

rom_arm_m3_Hwi_asm_switch_iar_lib.obj: family/arm/m3/Hwi_asm_switch_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/rom/cortexm/cc26xx/r2/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/Hwi_asm_switch_iar.sv7M rom_arm_m3_Hwi_asm_switch_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_Hwi_asm_switch_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(ASM_DEFS) $(XOPTS) $(INCS) rom_arm_m3_Hwi_asm_switch_iar_lib.asm

rom_arm_m3_IntrinsicsSupport_asm_iar_lib.obj: family/arm/m3/IntrinsicsSupport_asm_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/rom/cortexm/cc26xx/r2/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/IntrinsicsSupport_asm_iar.sv7M rom_arm_m3_IntrinsicsSupport_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_IntrinsicsSupport_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(ASM_DEFS) $(XOPTS) $(INCS) rom_arm_m3_IntrinsicsSupport_asm_iar_lib.asm

rom_arm_m3_TaskSupport_asm_iar_lib.obj: family/arm/m3/TaskSupport_asm_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/rom/cortexm/cc26xx/r2/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/TaskSupport_asm_iar.sv7M rom_arm_m3_TaskSupport_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_TaskSupport_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(ASM_DEFS) $(XOPTS) $(INCS) rom_arm_m3_TaskSupport_asm_iar_lib.asm


rom_sysbios.obj: BIOS.c knl/Clock.c knl/Idle.c knl/Intrinsics.c knl/Event.c knl/Mailbox.c knl/Queue.c knl/Semaphore.c knl/Swi.c knl/Task.c family/arm/m3/Hwi.c family/arm/m3/TaskSupport.c family/arm/cc26xx/Boot.c family/arm/cc26xx/Timer.c family/arm/cc26xx/TimestampProvider.c hal/Hwi.c hal/Hwi_stack.c hal/Hwi_startup.c heaps/HeapMem.c heaps/HeapCallback.c gates/GateHwi.c gates/GateMutex.c xdc/runtime/xdc_noinit.c xdc/runtime/Assert.c xdc/runtime/Core-mem.c xdc/runtime/Core-smem.c xdc/runtime/Core-label.c xdc/runtime/Core-params.c xdc/runtime/Diags.c xdc/runtime/Error.c xdc/runtime/Gate.c xdc/runtime/Log.c xdc/runtime/Memory.c xdc/runtime/Registry.c xdc/runtime/Startup.c xdc/runtime/System.c xdc/runtime/SysCallback.c xdc/runtime/Text.c xdc/runtime/Timestamp.c makefile
	@echo Creating the SYS/BIOS library that contains the APIs not included in the ROM ...
	@$(CC) -o rom_sysbios.obj $(BIOS_DEFS) $(XOPTS) $(INCS) --make_all_definitions_weak \
	$(BIOS_ROOT)BIOS.c \
	$(BIOS_ROOT)knl/Clock.c \
	$(BIOS_ROOT)knl/Idle.c \
	$(BIOS_ROOT)knl/Intrinsics.c \
	$(BIOS_ROOT)knl/Event.c \
	$(BIOS_ROOT)knl/Mailbox.c \
	$(BIOS_ROOT)knl/Queue.c \
	$(BIOS_ROOT)knl/Semaphore.c \
	$(BIOS_ROOT)knl/Swi.c \
	$(BIOS_ROOT)knl/Task.c \
	$(BIOS_ROOT)family/arm/m3/Hwi.c \
	$(BIOS_ROOT)family/arm/m3/TaskSupport.c \
	$(BIOS_ROOT)family/arm/cc26xx/Boot.c \
	$(BIOS_ROOT)family/arm/cc26xx/Timer.c \
	$(BIOS_ROOT)family/arm/cc26xx/TimestampProvider.c \
	$(BIOS_ROOT)hal/Hwi.c \
	$(BIOS_ROOT)hal/Hwi_stack.c \
	$(BIOS_ROOT)hal/Hwi_startup.c \
	$(BIOS_ROOT)heaps/HeapMem.c \
	$(BIOS_ROOT)heaps/HeapCallback.c \
	$(BIOS_ROOT)gates/GateHwi.c \
	$(BIOS_ROOT)gates/GateMutex.c \
	$(XDC_ROOT)xdc/runtime/xdc_noinit.c \
	$(XDC_ROOT)xdc/runtime/Assert.c \
	$(XDC_ROOT)xdc/runtime/Core-mem.c \
	$(XDC_ROOT)xdc/runtime/Core-smem.c \
	$(XDC_ROOT)xdc/runtime/Core-label.c \
	$(XDC_ROOT)xdc/runtime/Core-params.c \
	$(XDC_ROOT)xdc/runtime/Diags.c \
	$(XDC_ROOT)xdc/runtime/Error.c \
	$(XDC_ROOT)xdc/runtime/Gate.c \
	$(XDC_ROOT)xdc/runtime/Log.c \
	$(XDC_ROOT)xdc/runtime/Memory.c \
	$(XDC_ROOT)xdc/runtime/Registry.c \
	$(XDC_ROOT)xdc/runtime/Startup.c \
	$(XDC_ROOT)xdc/runtime/System.c \
	$(XDC_ROOT)xdc/runtime/SysCallback.c \
	$(XDC_ROOT)xdc/runtime/Text.c \
	$(XDC_ROOT)xdc/runtime/Timestamp.c \

rom_sysbios.arm3: rom_sysbios.obj rom_arm_m3_Hwi_asm_iar_lib.obj rom_arm_m3_Hwi_asm_switch_iar_lib.obj rom_arm_m3_IntrinsicsSupport_asm_iar_lib.obj rom_arm_m3_TaskSupport_asm_iar_lib.obj
	@-$(call RM, $@)
	@$(AR) $@ $^ C:/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages/ti/sysbios/rom/cortexm/cc26xx/r2/golden/CC26xx/iar_rom_sysbios_config.obj
	@echo Library build complete.

clean:
	@-$(call RM, *)
