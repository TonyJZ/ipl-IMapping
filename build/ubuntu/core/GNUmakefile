MODULE=core

include ../iplMake.opt

SRC_DIR= $(IPL_SRC)/$(MODULE)

_OBJS= iplcore.o iplCommon.o iplPluginDLL.o iplSystemModules.o iplPlatformUtil.o \
		XLogService.o XPlatform.o XProperty.o XRegisterService.o XXMLSerialize.o \
		tinyxml/tinystr.o tinyxml/tinyxmlerror.o tinyxml/tinyxml.o tinyxml/tinyxmlparser.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

MODULE_DLL=$(BIN_DIR)/libipl$(MODULE).so

$(MODULE_DLL): $(OBJS)
	$(LD) -s -shared $(OBJS) -lboost_system -liplcommonAPIs -lm -ldl -o  $(MODULE_DLL)   $(LNKFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:        $(MODULE_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o
	$(RM) $(OBJ_DIR)/tinyxml/*.o


	

