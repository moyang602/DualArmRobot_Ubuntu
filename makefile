jichu:	trajectory.o control.o 
	gcc -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__  control.o trajectory.o -Xlinker -rpath -Xlinker /usr/xenomai/lib -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrtdm -lrt -lX11 -lm -o jichu
control.o:	control.c trajectory.h global_def.h
	gcc -c -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__  control.c trajectory.h global_def.h -Xlinker -rpath -Xlinker /usr/xenomai/lib -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrtdm -lrt -lX11 -lm
trajectory.o:	trajectory.c trajectory.h
	gcc -c trajectory.c	-lm
clean:
	rm -f *.o
