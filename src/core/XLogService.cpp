//#include "orsCommon.h"

#include "XLogService.h"
#include "commonAPIs/iplutility.h"
#include "commonAPIs/iplstring.h"

using namespace ipl;
extern IPlatform * _getPlatform();

bool XLogService::startup(IPlatform *platform)
{
	//加载log4cxx的配置文件，这里使用了属性文件
	//std::string exdir = getExeDir();
	std::string exdir;
	get_directory_exe(exdir);

	std::string log4cxx_profile = exdir + "/log4cxx.properties";

	//////////////////////////////////////////////////////////////////////////

	m_logLevel = IPL_LOG_DEBUG;
		
	FILE *fp = fopen( log4cxx_profile.c_str(), "r" );
	if( NULL != fp )	{
		iplChar buf[256];
		
		iplArray<iplString>	vAppenders;

		while( !feof(fp) ) {
			fgets( buf, 256, fp );

			// 起始日志级别
			if( strstr( buf, _T("log4j.rootLogger" ) ) ) {
				if( strstr( buf, _T("ALL" ) ) )
					m_logLevel = IPL_LOG_DEBUG;
				else if( strstr( buf, _T("TRACE" ) ) )
					m_logLevel = IPL_LOG_DEBUG;
				else if( strstr( buf, _T("DEBUG" ) ) )
					m_logLevel = IPL_LOG_DEBUG;
				else if( strstr( buf, _T("INFO" ) ) )
					m_logLevel = IPL_LOG_INFO;
				else if( strstr( buf, _T("WARN" ) ) )
					m_logLevel = IPL_LOG_WARNING;
				else if( strstr( buf, _T("ERROR" ) ) )
					m_logLevel = IPL_LOG_ERROR;
				else if( strstr( buf, _T("FATAL" ) ) )
					m_logLevel = IPL_LOG_FATAL;

				//////////////////////////////////////////////////////////////////////////
				// 寻找appender
				iplString strbuf = buf;
				size_t pos = strbuf.find(",");
				iplString pAppender = strbuf.substr(pos);
				//iplChar *pAppender = (iplChar *)iplString::findSubStr_i( buf, "," );
				
				// 替换","为空格
				boost::algorithm::replace_all(pAppender, ",", " ");
// 				iplChar *s=pAppender;
// 
// 				// 替换,为空格
// 				while( *s )	{
// 					if( ',' == *s )	
// 						*s = ' ';
// 					s++;
// 				}

				// 扫描appender
				char buf1[80];

				do 
				{
					if( sscanf( pAppender.c_str(), "%s", buf1 ) > 0  )	{
						int len= strlen(buf1);
						if( len > 0 )
						{
							iplString appender = "log4j.appender.";
							appender = appender + buf1;
							vAppenders.push_back(appender);
							
							log4AppenderINFO info;
							m_vAppendInfos.push_back(info);
						}
						
						pos = pAppender.find(buf1);
						pAppender = pAppender.substr(pos + strlen(buf1));
						//pAppender = strstr( pAppender, buf1 ) + strlen(buf1);						
					}
					else 
						break;

				} while (true);
			}
			else {
				char buf1[80];

				int i;
				for( i=0; i<vAppenders.size(); i++)
				{
					if( strstr( buf, vAppenders[i].c_str() ) ) 
					{
						if( '=' == buf[vAppenders[i].length()] ) {
							sscanf( buf+ vAppenders[i].length()+1, "%s", buf1 );
							m_vAppendInfos[i].appenerID = buf1;
						}
						else if( strstr( buf, "File=" ) )	{
							const iplChar *pFileStr = strstr( buf, "File=");
							sscanf( pFileStr+strlen("File="), "%s", buf1 );
							m_vAppendInfos[i].logFileName = buf1;
						}
						else if( strstr( buf, "Append=" ) )	{
							if( strstr( buf, "true" ) )	
								m_vAppendInfos[i].bAppend = true;
							else
								m_vAppendInfos[i].bAppend = false;
						}
						else if( strstr( buf, "layout=" ) )	{
							const iplChar *layout = strstr( buf, "layout=");
							sscanf( layout+strlen("layout="), "%s", buf1 );
							m_vAppendInfos[i].layout = buf1;
						}
						else if( strstr( buf, "layout.ConversionPattern=" ) )	{
							iplChar *pattern = (iplChar *)strstr( buf, "layout.ConversionPattern=");

							pattern = pattern + +strlen("layout.ConversionPattern=");

							int len=strlen(pattern);

							if( '\r' == pattern[len-1] )
								pattern[len-1] = 0;
							else if( '\n' == pattern[len-1] )
								pattern[len-1] = 0;

							m_vAppendInfos[i].conversionPattern = pattern;
						}
					}
				}
			}
		};

		fclose(fp);
	}
	else	{
		printf("Can not open %s \n", log4cxx_profile.c_str() );
	}

	return true;
}



void XLogService::shutdown()
{
}

bool XLogService::isok()
{
	return true;
}


// 任务名称，只需最开始设置一次
void XLogService::setTaskName(const iplChar *taskName, const iplChar *logFile  )
{
	m_taskName = taskName;

	//创建日志对象

	int i;
	for( i=0; i<m_vAppendInfos.size(); i++)
	{
		ref_ptr<ILogger> logger(IPL_PTR_CAST( ILogger, _getPlatform()->createObject( m_vAppendInfos[i].appenerID.c_str() ) )); 

		if( NULL != logger.get() )	{
			logger->setTaskName( taskName );
			logger->setLayout( m_vAppendInfos[i].layout.c_str(), m_vAppendInfos[i].conversionPattern.c_str() );
			
			if( NULL != logFile ) {
				if( !m_vAppendInfos[i].logFileName.empty() ) {
					m_vAppendInfos[i].logFileName = logFile;
				}
			}
			
			if( !m_vAppendInfos[i].logFileName.empty() )
				logger->setLogFile( m_vAppendInfos[i].logFileName.c_str(), m_vAppendInfos[i].bAppend );
			
			m_vAppenders.push_back( logger );			
		}
	}
}


// 总是log
void XLogService::fatal(const char * strModule, const char * msg,
						const char * file, int row)
{
	if( m_vAppenders.size() > 0 )	{
		int i;
		for( i=0; i<m_vAppenders.size(); i++)
			m_vAppenders[i]->fatal( msg );

		return ;
	}

	if( NULL != file )
		printf("fatal: %s: %s row%d\n", msg, file, row);
	else
		printf("fatal: %s\n", msg);

#ifdef _WIN32
	if( m_bInteractive )	{
		char buf[256];
		
		if( NULL != file )
			sprintf( buf, "fatal: %s: %s row%d\n", msg, file, row );	
		else 
		sprintf( buf, "fatal: %s\n", msg);	

		//MessageBox( NULL, msg, "OpenRS", MB_ICONERROR );
	}
#endif

}

// 总是log
void XLogService::error(const char * strModule, const char * msg,
						const char * file, int row)
{
	if( m_vAppenders.size() > 0 )	{
		int i;
		for( i=0; i<m_vAppenders.size(); i++)
			m_vAppenders[i]->error( msg );
		
		return ;
	}

	if( NULL != file )
		printf("error: %s: %s row%d\n",msg,file,row);
	else
		printf("error: %s\n", msg);

#ifdef _WIN32
	if( m_bInteractive )	{
		char buf[256];
		
		if( NULL != file )
			sprintf( buf, "error: %s: %s row%d\n", msg, file, row );	
		else 
			sprintf( buf, "error: %s\n", msg);	
		
		//MessageBox( NULL, msg, "OpenRS", MB_ICONERROR );
	}
#endif
}

void XLogService::warn(const char * strModule, const char * msg,
						   const char * file, int row)
{
	if( m_vAppenders.size() > 0 )	{
		int i;
		for( i=0; i<m_vAppenders.size(); i++)
			m_vAppenders[i]->warn( msg );
		
		return ;
	}

	if( m_logLevel <= IPL_LOG_WARNING ) {		
		if( NULL != file )
			printf("warning: %s:%s row%d\n",msg,file,row);
		else
			printf("warning: %s\n", msg);
	}

}



void XLogService::info(const char * strModule, const char * msg,
					   const char * file, int row)
{

	if( m_vAppenders.size() > 0 )	{
		int i;
		for( i=0; i<m_vAppenders.size(); i++)
			m_vAppenders[i]->info( msg );
		
		return ;
	}

	iplString str = msg;
	#ifdef IPL_PLATFORM_WINDOWS
 		if( iplString::npos != str.find("process" ) && strchr( msg, '%') ) {
	 		printf("info: %s\r", msg);
			return;
		}
	#else
		if( iplString::npos != str.find("process" ) && strchr( msg, '%') ) {
			printf("info: %s\r", msg);
			fflush(stdout);
			return;
		}
	#endif

	if( m_logLevel <= IPL_LOG_WARNING ) {
		if( NULL != file )
			printf("info: %s: %s row%d\n",msg,file,row);
		else
			printf("info: %s\n", msg);
	}

}

void	XLogService::debug(const char * strModule, const char * msg,
						const char * file, int row)
{
	if( m_vAppenders.size() > 0 )	{
		// 系统注销中
		iplString str = msg;
		if( iplString::npos !=str.find(_T("XRegisterService: shutdown") ) )	{
			m_vAppenders.clear();
			m_vAppendInfos.clear();
		}
		else	{
			int i;
			for( i=0; i<m_vAppenders.size(); i++)
				m_vAppenders[i]->debug( msg );
			
			return ;
		}
	}
	
	if( m_logLevel <= IPL_LOG_DEBUG ) {		
		if( NULL != file )
			printf("debug: %s: %s row%d\n",msg,file,row);
		else
			printf("debug: %s\n", msg);
	}

}