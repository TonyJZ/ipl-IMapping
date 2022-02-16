#pragma once

#include "GDAL/ogr_core.h"

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////


	typedef int SFERR;

#define SFERR_NONE                0
#define SFERR_NOT_ENOUGH_DATA     1    /* not enough data to deserialize */
#define SFERR_NOT_ENOUGH_MEMORY   2
#define SFERR_UNSUPPORTED_GEOMETRY_TYPE 3
#define SFERR_UNSUPPORTED_OPERATION 4
#define SFERR_CORRUPT_DATA        5
#define SFERR_FAILURE             6
#define SFERR_UNSUPPORTED_SRS     7



	//#define ogrZMarker 0x21125711

	typedef enum
	{
		SF_wkbXDR = 0,         /* MSB/Sun/Motoroloa: Most Significant Byte First   */
		SF_wkbNDR = 1          /* LSB/Intel/Vax: Least Significant Byte First      */
	} SF_wkbByteOrder;
	/************************************************************************/
	/*                  orsFeature.h related definitions.                  */
	/************************************************************************/

	/**
	* List of feature field types.  This list is likely to be extended in the
	* future ... avoid coding applications based on the assumption that all
	* field types can be known.
	*/

// 	typedef enum
// 	{
// 		/** Simple 32bit integer */                   SF_ftInteger = 0,
// 		/** List of 32bit integers */                 SF_ftIntegerList = 1,
// 		/** Double Precision floating point */        SF_ftReal = 2,
// 		/** List of doubles */                        SF_ftRealList = 3,
// 		/** String of ASCII chars */                  SF_ftString = 4,
// 		/** Array of strings */                       SF_ftStringList = 5,
// 		/** Double byte string (unsupported) */       SF_ftWideString = 6,
// 		/** List of wide strings (unsupported) */     SF_ftWideStringList = 7,
// 		/** Raw Binary data */                        SF_ftBinary = 8,
// 		/** Date */                                   SF_ftDate = 9,
// 		/** Time */                                   SF_ftTime = 10,
// 		/** Date and Time */                          SF_ftDateTime = 11
// 	} SF_FieldType;

	typedef OGRFieldType SF_FieldType;

	/**
	* Display justification for field values.
	*/

	typedef enum
	{
		SF_JUndefined = 0,
		SF_JLeft = 1,
		SF_JRight = 2
	} SF_Justification;


#define SF_nullFID            -1
#define SF_unSetMarker        -21121
#define SF_nullMarker         -21122

	/************************************************************************/
	/*                               SFField                               */
	/************************************************************************/

	/**
	* SFFeature field attribute value union.
	*/

// 	typedef union {
// 		int         Integer;
// 		double      Real;
// 		char       *String;
// 		/* wchar    *WideString; */
// 
// 		struct {
// 			int     nCount;
// 			int     *paList;
// 		} IntegerList;
// 
// 		struct {
// 			int     nCount;
// 			double  *paList;
// 		} RealList;
// 
// 		struct {
// 			int     nCount;
// 			char    **paList;
// 		} StringList;
// 
// 		/*
// 		union {
// 		int   nCount;
// 		wchar *paList;
// 		} SFWideStringList;
// 		*/
// 
// 		struct {
// 			int     nCount;
// 			unsigned char   *paData;
// 		} Binary;
// 
// 		struct {
// 			int     nMarker1;
// 			int     nMarker2;
// 		} Set;
// 
// 		struct {
// 			short  Year;
// 			unsigned char   Month;
// 			unsigned char   Day;
// 			unsigned char   Hour;
// 			unsigned char   Minute;
// 			unsigned char   Second;
// 			unsigned char   TZFlag; /* 0=unknown, 1=localtime(ambiguous),
// 									100=GMT, 104=GMT+1, 80=GMT-5, etc */
// 		} Date;
// 	} SFField;

	typedef OGRField SFField;


#define OLCRandomRead          "RandomRead"
#define OLCSequentialWrite     "SequentialWrite"
#define OLCRandomWrite         "RandomWrite"
#define OLCFastSpatialFilter   "FastSpatialFilter"
#define OLCFastFeatureCount    "FastFeatureCount"
#define OLCFastGetExtent       "FastGetExtent"
#define OLCCreateField         "CreateField"
#define OLCTransactions        "Transactions"
#define OLCDeleteFeature       "DeleteFeature"
#define OLCFastSetNextByIndex  "FastSetNextByIndex"

#define ODCCreateLayer        "CreateLayer"
#define ODCDeleteLayer        "DeleteLayer"

#define SFSCCreateDataSource   "CreateDataSource"
#define SFSCDeleteDataSource   "DeleteDataSource"


#define SF_FILE_FORMAT_SHP		"ESRI Shapefile"
#define SF_FILE_FORMAT_KML		"KML"
#define SF_FILE_FORMAT_LIBKML		"LIBKML"

}

