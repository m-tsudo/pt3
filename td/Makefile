PROJECT = td
OBJS = td.o

all: check_lfsr $(PROJECT)

check_lfsr : check_lfsr.c
	$(CC) -o check_lfsr check_lfsr.c

$(PROJECT) : $(OBJS)
	$(CC) -o $(PROJECT) $(OBJS)

clean:
	rm -f $(OBJS) $(PROJECT)
