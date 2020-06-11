///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains the IceImageWork API.
 *	\file		IceImageWorkAPI.h
 *	\author		Pierre Terdiman
 *	\date		April, 18, 2003
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEIMAGEWORKAPI_H
#define ICEIMAGEWORKAPI_H

	class IceImageWorkAPI : public IceAPI
	{
		public:
		// Constructor/Destructor
											IceImageWorkAPI();
		virtual								~IceImageWorkAPI();

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Initializes the API. Must be called first, else the API behaviour is undefined.
		 *	\param		create	[in] creation parameters
		 *	\return		true if the API is initialized for the first time
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		override2(IceAPI)	bool			Init(const APICREATE* create);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Closes the API.
		 *	\param		force_release	[in] true to force the lib to be released, else use refcounting
		 *	\return		false if the API is still beeing referenced
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		override2(IceAPI)	bool			Close(bool force_release);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets the API's name.
		 *	\return		the API name
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		override2(IceAPI)	const char*		GetAPIName()			const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets the compile-time string. This can be used to check which DLL is the most recent.
		 *	\return		compile-time string
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		override2(IceAPI)	const char*		GetCompileTimeString()	const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Class factory.
		 *	\param		class_name	[in] name of desired class
		 *	\return		new class instance, or null
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		override2(IceAPI)	void*			CreateClass(const char* class_name);
		override2(IceAPI)	void			NewInterface(IceInterface* it);
	};

	FUNCTION ICEIMAGEWORK_API bool InitIceImageWork();
	FUNCTION ICEIMAGEWORK_API bool CloseIceImageWork(bool force_release=false);

#endif // ICEIMAGEWORKAPI_H
