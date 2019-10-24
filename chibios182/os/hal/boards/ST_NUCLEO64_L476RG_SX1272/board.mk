# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_L476RG_SX1272/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_L476RG_SX1272

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
