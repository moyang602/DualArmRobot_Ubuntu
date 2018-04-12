jichu:	SerialComm.o trajectory.o communication.o control.o 
	gcc -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__ SerialComm.o control.o communication.o trajectory.o -Xlinker -rpath -Xlinker /usr/xenomai/lib -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrtdm -lrt -lX11 -lm -o jichu
control.o:	control.c SerialComm.h trajectory.h global_def.h HandControl.h
	gcc -c -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__  control.c SerialComm.h trajectory.h global_def.h HandControl.h -Xlinker -rpath -Xlinker /usr/xenomai/lib -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrtdm -lrt -lX11 -lm
communication.o:	communication.c communication.h
	gcc -c communication.c	-lm
trajectory.o:	trajectory.c trajectory.h global_def.h
	gcc -c trajectory.c	-lm
SerialComm.o:	SerialComm.c SerialComm.h
	gcc -c SerialComm.c	-lm
clean:
	rm -f *.o
