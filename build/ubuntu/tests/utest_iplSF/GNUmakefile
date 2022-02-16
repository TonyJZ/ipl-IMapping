APP=utest_iplSF

include ../../iplMake.opt

SRC_DIR=$(IPL_TESTS)/$(APP)

_OBJS= utest_iplSF.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(APP_EXE): $(OBJS)	
	$(LD) -s $(LNKFLAGS)  $(OBJS) -liplcore -liplcommonAPIs -o  $(APP_EXE)	

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:  $(APP_EXE)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

