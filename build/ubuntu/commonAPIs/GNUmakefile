MODULE=commonAPIs

include ../iplMake.opt

SRC_DIR= $(IPL_SRC)/$(MODULE)

_OBJS= iplstring.o iplutility.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

MODULE_DLL=$(BIN_DIR)/libipl$(MODULE).so

$(MODULE_DLL): $(OBJS)
	$(LD) -s -shared $(OBJS) -lboost_system -lboost_filesystem\
	 -lm -ldl -o  $(MODULE_DLL)   $(LNKFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:        $(MODULE_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

