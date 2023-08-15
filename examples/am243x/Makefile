SUBDIRS=MyFirst_PRU_Assembler_Program \
	MyFirst_PRU_C_Program \
	MyFirst_PRU_mixed_Program

all: $(SUBDIRS)

$(SUBDIRS):
	@$(MAKE) -C $@

clean:
	@for d in $(SUBDIRS); do (cd $$d; $(MAKE) clean ); done

.PHONY: all clean $(SUBDIRS)
