APP=utest_iplgeometry

include ../../iplMake.opt

SRC_DIR=$(IPL_TESTS)/$(APP)

_OBJS= data_simulate.o utest_iplgeometry.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(APP_EXE): $(OBJS)	
	$(LD) -s $(LNKFLAGS)  $(OBJS) -liplcore -liplcommonAPIs -lboost_system\
	 -lpcl_common -lpcl_io -lpcl_registration -o  $(APP_EXE)	

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:  $(APP_EXE)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

