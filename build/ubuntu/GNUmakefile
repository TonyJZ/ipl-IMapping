#export ROOT_DIR=$(CURDIR)
#include $(ROOT_DIR)/Makefile.param

SUBDIRS = commonAPIs \
 core \
 modules/geometry \
 modules/SR \
 modules/SF \
 plugins/OGRSFDriver



#all: 
#  @for dir in $(SUBDIRS); \
  do \ echo "folder $$(dir)"; \
  done

all:
 cd commonAPIs; $(MAKE) \
 done
  
#clean: $(SUBDIRS) \
    for dir in $(SUBDIRS); do \
      cd $dir && make clean && cd ..; \
    done
    #rm -f `find -name "*~"` || /bin/true  

