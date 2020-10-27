PROJECT=$(BLE_REPO)/examples/rtos/CC2640R2_LAUNCHXL/ble5stack/simple_peripheral/tirtos/iar
BLE_REPO=../../../../../../..
IAR_PATH=C:/Program\ Files\ \(x86\)/IAR\ Systems/EWARM-$(IAR_VERSION)/common/bin
IAR_VERSION=8.11.1

all: simple_peripheral_src

release: clean simple_peripheral

simple_peripheral:
	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack.ewp \
	-build \
	FlashROM_Library \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app.ewp \
	-build \
	FlashROM_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral.custom_argvars

simple_peripheral_src:
	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack_src.ewp \
	-build \
	FlashOnly_Library \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral_src.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app_src.ewp \
	-build \
	FlashOnly_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral_src.custom_argvars

clean:
	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack.ewp \
	-clean \
	FlashROM_Library \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app.ewp \
	-clean \
	FlashROM_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack_src.ewp \
	-clean \
	FlashOnly_Library \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral_src.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app_src.ewp \
	-clean \
	FlashOnly_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ble5_simple_peripheral_src.custom_argvars
