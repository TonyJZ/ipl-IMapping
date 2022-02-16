MODULE=geometry

include ../../iplMake.opt

SRC_DIR= $(IPL_MODULE_SRC)/$(MODULE)

_OBJS= XGeometryTransform2D.o XGeometryTransform3D.o XPlugin.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

MODULE_DLL=$(MODULE_DIR)/libipl$(MODULE).so

$(MODULE_DLL): $(OBJS)
	$(LD) -s -shared $(OBJS) -lm -ldl -o  $(MODULE_DLL)   $(LNKFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:        $(MODULE_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

