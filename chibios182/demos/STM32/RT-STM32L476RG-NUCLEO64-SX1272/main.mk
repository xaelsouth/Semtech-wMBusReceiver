# List of the ChibiOS/RT CMSIS RTOS wrapper.
MAINSRC = timeutil.c \
		  rs485thread.c \
		  drivers/src/system/timer.c \
		  drivers/src/radio/t1_c1_util.c \
		  sx1272thread.c \
		  drivers/src/radio/sx1272/sx1272.c \
		  drivers/src/boards/NucleoL476/sx1272mb2das-board.c \
		  sx1276thread.c \
		  drivers/src/radio/sx1276/sx1276.c \
		  drivers/src/boards/NucleoL476/sx1276mb1mas-board.c
 
MAININC = drivers/src/system \
		  drivers/src/radio \
		  drivers/src/radio/sx1272 \
		  drivers/src/radio/sx1276 \
		  drivers/src/boards \
		  drivers/src/boards/NucleoL476

# Shared variables
ALLCSRC += $(MAINSRC)
ALLINC  += $(MAININC)
