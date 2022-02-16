#pragma once

#include "core/iplmacro.h"
#include "core/interface/IVectorSource.h"
#include "geometry/GeometryDef.h"
#include "spatialreference/interface/ISpatialReference.h"

//#include "simplefeature/interface/ISFFeature.h"
#include "simplefeature/SFBaseDef.h"
#include "simplefeature/SFStyleTable.h"


namespace ipl
{
	/************************************************************************/
	/*                            ISFVectorSource                             */
	/************************************************************************/

	/**
	* This class represents a data source.  A data source potentially
	* consists of many layers (ISFVectorLayer).  A data source normally consists
	* of one, or a related set of files, though the name doesn't have to be
	* a real item in the file system.
	*
	* When an ISFVectorSource is destroyed, all it's associated VectorLayers objects
	* are also destroyed.
	*/
	interface ISFVectorLayer;

	interface ISFVectorSource : public IVectorSource
	{
	public:
		virtual size_t GetLayerCount() = 0;

		virtual ISFVectorLayer *GetLayerByName(const char *) = 0;

		virtual ISFVectorLayer *GetLayer(int) = 0;

		virtual void addLayer(ref_ptr<ISFVectorLayer> pLayer) = 0;

		virtual SFERR DeleteLayer(int) = 0;

// 		virtual iplFileFormatList GetSupportedFormats() = 0;
// 
// 		virtual bool Open(const char *pszName, int bUpdate) = 0;
// 		virtual bool Create(const char *pszName, char *pszFormat, ref_ptr<ISpatialReference> &poSpatialRef) = 0;
// 
// 		virtual int DeleteDataSource(const char *pszName) = 0;
// 
// 		virtual void Close() = 0;
// 
// 		virtual bool IsOpen() const = 0;



// 		virtual int TestCapability(const char *) = 0;
// 

// 
// 		virtual ISFVectorLayer *CreateLayer(const char *pszName,
// 			IPL_wkbGeometryType eGType, char ** papszOptions = NULL) = 0;

		// 	  virtual ISFVectorLayer *CopyLayer( ISFVectorLayer *poSrcLayer,
		// 		  const char *pszNewName,
		// 		  char **papszOptions) = 0;


		//     virtual ISFVectorSource *CopyDataSource( ISFVectorSource *poSrcDS,
		// 		const char *pszNewName,
		// 		char **papszOptions ) = 0;

		IPL_INTERFACE_DEF(IVectorSource, "SF");

	};

#define IPL_VECTORSOURCE_SF_DEFAULT		"ipl.dataSource.VectorSource.SF.OGR"

}
