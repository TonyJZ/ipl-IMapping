#pragma once

namespace ipl
{
	// interface SFFeature; //每个Feature一个STYLE

	class SFVectorLayer;

	/**
	* SFStyleTool
	*/
	typedef enum sf_style_tool_id
	{
		STNone = 0,
		STPen = 1,
		STBrush = 2,
		STSymbol = 3,
		STLabel = 4,
		//  STVector = 5   //vector在OGR以后的版本中可能不用
	} SFSTID;

	/**
	* List of units supported by SFStyleTools.
	*/
	typedef enum sf_style_tool_units_id
	{
		STUGround = 0,
		STUPixel = 1,
		STUPoints = 2,
		STUMM = 3,
		STUCM = 4,
		STUInches = 5
	} SFSTUnitId;

	/**
	* List of parameters for use with SFStylePen.
	*/
	typedef enum sf_style_tool_param_pen_id
	{
		STPenColor = 0,
		STPenWidth = 1,
		STPenPattern = 2,
		STPenId = 3,
		STPenPerOffset = 4,
		STPenCap = 5,
		STPenJoin = 6,
		STPenPriority = 7,
		STPenLast = 8

	} SFSTPenParam;

	/**
	* List of parameters for use with SFStyleBrush.
	*/
	typedef enum sf_style_tool_param_brush_id
	{
		STBrushFColor = 0,
		STBrushBColor = 1,
		STBrushId = 2,
		STBrushAngle = 3,
		STBrushSize = 4,
		STBrushDx = 5,
		STBrushDy = 6,
		STBrushPriority = 7,
		STBrushLast = 8

	} SFSTBrushParam;


	/**
	* List of parameters for use with SFStyleSymbol.
	*/
	typedef enum sf_style_tool_param_symbol_id
	{
		STSymbolId = 0,
		STSymbolAngle = 1,
		STSymbolColor = 2,
		STSymbolSize = 3,
		STSymbolDx = 4,
		STSymbolDy = 5,
		STSymbolStep = 6,
		STSymbolPerp = 7,
		STSymbolOffset = 8,
		STSymbolPriority = 9,
		STSymbolFontName = 10,
		STSymbolLast = 11

	} SFSTSymbolParam;

	/**
	* List of parameters for use with SFStyleLabel.
	*/
	typedef enum sf_style_tool_param_label_id
	{
		STLabelFontName = 0,
		STLabelSize = 1,
		STLabelTextString = 2,
		STLabelAngle = 3,
		STLabelFColor = 4,
		STLabelBColor = 5,
		STLabelPlacement = 6,
		STLabelAnchor = 7,
		STLabelDx = 8,
		STLabelDy = 9,
		STLabelPerp = 10,
		STLabelBold = 11,
		STLabelItalic = 12,
		STLabelUnderline = 13,
		STLabelPriority = 14,
		STLabelStrikeout = 15,
		STLabelStretch = 16,
		STLabelAdjHor = 17,
		STLabelAdjVert = 18,
		STLabelHColor = 19,
		STLabelLast = 20

	} SFSTLabelParam;

	//vector在OGR以后的版本中可能不用
	// typedef enum sf_style_tool_param_vector_id
	// {
	//     SFSTVectorId = 0,
	//     SFSTVectorNoCompress,
	//     SFSTVectorSprain,
	//     SFSTVectorNoSlope,
	//     SFSTVectorMirroring,
	//     SFSTVectorCentering,
	//     SFSTVectorPriority,
	//     SFSTVectorLast
	//
	// } SFSTVectorParam;

	typedef enum sf_style_type
	{
		STypeString,
		STypeDouble,
		STypeInteger,
		STypeBoolean
	}  SFSType;

	typedef struct sf_style_param
	{
		int              eParam;
		char            *pszToken;
		int              bGeoref;
		SFSType         eType;
	}SFStyleParamId;


	typedef struct sf_style_value
	{
		char            *pszValue;
		double           dfValue;
		int              nValue; // Used for both integer and boolean types
		int              bValid;
		SFSTUnitId      eUnit;
	}SFStyleValue;


	//Everytime a pszStyleString gived in parameter is NULL,
	//    the StyleString defined in the Mgr will be use.

// 	class SFStyleTable
// 	{
// 	private:
// 		char **m_papszStyleTable;
// 
// 	public:
// 		SFStyleTable();
// 		~SFStyleTable()

// 		{

// 			delete m_papszStyleTable;

// 		}
// 		int AddStyle(const char *pszName, const char *pszStyleString);
// 		int RemoveStyle(const char *pszName);
// 		int ModifyStyle(const char *pszName, const char *pszStyleString);
// 
// 		//     int SaveStyleTable(const char *pszFilename);

// 		//     int LoadStyleTable(const char *pszFilename);

// 
// 		const char *Find(const char *pszStyleString);
// 		int IsExist(const char *pszName);
// 		const char *GetStyleName(const char *pszName);

// 
// 		void  Clear();
// 		SFStyleTable   *Clone();
// 	};
// 

// 	class SFStyleMgr

// 	{

// 	private:

// 		SFStyleTable   *m_poDataSetStyleTable;

// 		char            *m_pszStyleString;

// 

// 	public:

// 		SFStyleMgr(SFStyleTable *poDataSetStyleTable = NULL);

// 		~SFStyleMgr()

// 		{

// 			delete m_poDataSetStyleTable;

// 			delete m_pszStyleString;

// 		}

// 

// 		//  bool SetFeatureStyleString(SFFeature *,const char *pszStyleString=NULL,

// 		//                                 bool bNoMatching = FALSE);

// 

// 		bool SetLayerStyleString(SFVectorLayer *, const char *pszStyleString = NULL,

// 			bool bNoMatching = false);

// 

// 

// 		//  const char *InitFromFeature(SFFeature *);

// 

// 		const char *InitFromLayer(SFVectorLayer *);

// 

// 		bool InitStyleString(const char *pszStyleString = NULL);

// 

// 		const char *GetStyleName(const char *pszStyleString = NULL);

// 		const char *GetStyleByName(const char *pszStyleName);

// 

// 		bool AddStyle(const char *pszStyleName, const char *pszStyleString = NULL);

// 

// 		//  const char *GetStyleString(SFFeature * = NULL);

// 		const char *GetStyleString(SFVectorLayer * = NULL);

// 

// 		/*  bool AddPart(OGRStyleTool *);*/

// 		bool AddPart(const char *);

// 

// 		int GetPartCount(const char *pszStyleString = NULL);

// 

// 		//  SFStyleTool *GetPart(int hPartId, const char *pszStyleString = NULL);

// 

// 		SFStyleTable *GetDataSetStyleTable() { return m_poDataSetStyleTable; }

// 

// 		//  SFStyleTool *CreateStyleToolFromStyleString(const char *pszStyleString);

// 

// 	};

}

