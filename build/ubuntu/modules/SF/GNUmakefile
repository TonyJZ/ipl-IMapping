MODULE=SF

include ../../iplMake.opt

SRC_DIR= $(IPL_MODULE_SRC)/$(MODULE)

_OBJS= XSFFeature.o XSFFeatureDefn.o XSFFieldDefn.o XSFGeometryCollection.o\
 XSFLineRing.o XSFMultiLineString.o XSFMultiPoint.o XSFMultiPolygon.o\
 XSFOGRVectorLayer.o XSFOGRVectorSource.o XSFPoint.o XSFPolygon.o XSFService.o XPlugin.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

MODULE_DLL=$(MODULE_DIR)/libipl$(MODULE).so

$(MODULE_DLL): $(OBJS)
	$(LD) -s -shared $(OBJS) -liplcommonAPIs -lgdal -lm -ldl -o  $(MODULE_DLL)   $(LNKFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:        $(MODULE_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o


	

