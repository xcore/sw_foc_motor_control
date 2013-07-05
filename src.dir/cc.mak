# ansi C compile

# get Operating System
OS = $(shell uname)

MAIN =	gen_sine_data

CMODS =	$(MAIN) \

# Common include files (NB triggers recompilation of all source)
CINCS = $(MAIN).h \

INC_DIR = ../app_test_adc/src

OBJ_DIR = $(OS).dir
EXE_DIR = $(OBJ_DIR)

EXE = $(MAIN:%=$(EXE_DIR)/%.x)

COBJS   = $(CMODS:%=$(OBJ_DIR)/%.o)
FLIBS   = $(LIBS:%=-l%) -lm
FINCS   = $(INC_DIR:%=-I%)

CC = gcc

# This section assigns CFLAGS ...

CFLAGS = $(OPT) -Wall

LDFLAGS = $(FLDIRS) 

$(EXE):	$(COBJS) 
	$(LINK.c) $(COBJS) $(FLIBS) -o $(EXE)

$(COBJS) : $(OBJ_DIR)/%.o: %.c %.h $(CINCS) cc.mak
	$(CC) -c -I$(INC_DIR) $(CFLAGS) $< -o $@

clean:
	\rm $(OBJ_DIR)/*.o
	\rm $(EXE)

#
