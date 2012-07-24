PROJECT = td
OBJS = td.o
#CFLAGS=-m64

all: check_lfsr $(PROJECT)

check_lfsr : check_lfsr.c
	$(CC) $(CFLAGS) -o check_lfsr check_lfsr.c

$(PROJECT) : $(OBJS)
	$(CC) $(CFLAGS) -o $(PROJECT) $(OBJS)

clean:
	rm -f $(OBJS) $(PROJECT) check_lfsr *.ts
