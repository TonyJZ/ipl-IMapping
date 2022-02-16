PLUGIN=OGRSFDriver

include ../../iplMake.opt

SRC_DIR= $(IPL_PLUGIN_SRC)/$(PLUGIN)

_OBJS= XSFVectorSourceReader.o XSFVectorSourceWriter.o XPlugin.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

PLUGIN_DLL=$(PLUGIN_DIR)/lib$(PLUGIN).so

$(PLUGIN_DLL): $(OBJS)
	$(LD) -s -shared $(OBJS) -liplcommonAPIs -lgdal -lm -ldl -o  $(PLUGIN_DLL)   $(LNKFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:        $(PLUGIN_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

