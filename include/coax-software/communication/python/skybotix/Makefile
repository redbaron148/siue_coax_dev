MODULE_NAME=sbsimple

DEFINES=-D_GNU_SOURCE -DSBC_HAS_IO -DDEFENSIVE -DSBC_HAS_COMM \
	-DSBC_HAS_PTHREAD -DLINUX 

SWIG_FILE=$(MODULE_NAME).i
WRAP=$(MODULE_NAME)_wrap.c
SRC=sbsimple_util.c
OBJ=$(SRC:%.c=%.o) $(WRAP:%.c=%.o)
PYTHON_MODULE=_$(MODULE_NAME).so

PYTHON_INSTALL=$(HOME)/lib/python/skybotix


all: $(PYTHON_MODULE)

$(PYTHON_MODULE): $(OBJ)
	gcc -shared -o $@ $+\
		-L../../build-linux/src -lsbcom \
		`python2.5-config --ldflags` -lpthread

$(OBJ): %.o : %.c
	 gcc -c $(DEFINES) -o $@ $< \
		 -I../../include `python2.5-config --cflags`

$(WRAP): $(SWIG_FILE)
	swig $(DEFINES) -python $<

install: $(PYTHON_MODULE)
	mkdir -p $(PYTHON_INSTALL)
	cp -f $(PYTHON_MODULE) $(MODULE_NAME).py __init__.py $(PYTHON_INSTALL)

clean:
	-rm -f $(PYTHON_MODULE)
	-rm -f $(OBJ)
	-rm -f $(WRAP)

