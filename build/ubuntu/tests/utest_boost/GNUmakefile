APP=utest_boost

include ../../iplMake.opt

SRC_DIR=$(IPL_TESTS)/$(APP)

_OBJS= utest_boost.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(APP_EXE): $(OBJS)	
	$(LD) -s $(LNKFLAGS)  $(OBJS) -lboost_system -lboost_filesystem -o  $(APP_EXE)	

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:  $(APP_EXE)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

