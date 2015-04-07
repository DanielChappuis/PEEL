///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains message-related code.
 *	\file		IceMsg.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEMSG_H
#define ICEMSG_H

//#define OLD_ICE_MESSAGES

	// Messages codes for IceCore
	extern ICECORE_API const char* KM_SELF_DESTRUCT;
	extern ICECORE_API const char* KM_CORRUPTED_DELETION;
	extern ICECORE_API const char* KM_INVALID_REFERENCE;
	extern ICECORE_API const char* KM_CELL_MODIFICATION;
	extern ICECORE_API const char* KM_FIELD_MODIFICATION;

#ifdef OLD_ICE_MESSAGES

	//! Message types
	enum MsgType
	{
		MSG_ERROR			= 1,			//!< Error message
		MSG_SYSTEM			= 2,			//!< Kernel message
		MSG_USER			= 3,			//!< User-defined message

		MSG_FORCE_DWORD		= 0x7fffffff
	};

	// Message structure
	class ICECORE_API IceMsg
	{
		public:
		// Constructor/Destructor
		inline_						IceMsg() : mType(MSG_SYSTEM), mParam(0)	{}
		inline_						~IceMsg()								{}

		inline_			bool		SetCustomText(const char* txt)	{	return mText.Set(txt);	}

						MsgType		mType;			//!< Message type
						String		mID;			//!< Message identifier
						udword		mParam;			//!< Misc parameter
						String		mText;			//!< Custom text
	};

	FUNCTION ICECORE_API bool			SendMsg(const IceMsg& msg);
	FUNCTION ICECORE_API const String*	TranslateMsg(const IceMsg& msg);

#else

	struct ICECORE_API IceMessageParams
	{
	};

	// Message structure
	class ICECORE_API IceMsg : public Allocateable
	{
		public:
		// Constructor/Destructor
		inline_						IceMsg()					: mID(null), mParams(null), mRecipient(INVALID_KID)					{}
		inline_						IceMsg(const IceMsg& msg)	: mID(msg.mID), mParams(msg.mParams), mRecipient(msg.mRecipient)	{}
		inline_						~IceMsg()																						{}

//		inline_			bool		SetCustomText(const char* txt)	{	return mText.Set(txt);	}
		inline_			void		SetRecipient(const Cell* recipient)
									{
										mRecipient = recipient ? recipient->GetKernelID() : INVALID_KID;
									}

						const char*	mID;			//!< Message identifier
						void*		mParams;		//!< Generic parameters (depends on mID)
						KID			mRecipient;
//						String		mText;			//!< Custom text
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	A global function to send a new message.
	 *	\relates	IceMsg
	 *	\fn			SendMsg(const IceMsg& msg, float delay, KID recipient)
	 *	\param		msg			[in] the message
	 *	\param		delay		[in] delay before message delivery
	 *	\return		true if success
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FUNCTION ICECORE_API bool			SendMsg(const IceMsg& msg, float delay=0.0f);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	A global function to set the router's global time.
	 *	\relates	IceMsg
	 *	\fn			SetRouterGlobalTime(float global_time)
	 *	\param		global_time		[in] new global time for the router
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FUNCTION ICECORE_API void			SetRouterGlobalTime(float global_time);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	A global function to update the router. Messages are sent if delivery time is reached.
	 *	\relates	IceMsg
	 *	\fn			UpdateRouter(float delta_time)
	 *	\param		delta_time	[in] elapsed time
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FUNCTION ICECORE_API void			UpdateRouter(float delta_time);

#endif

	// Global messages

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	A global function to get last message
	 *	\relates	IceMsg
	 *	\fn			GetMsg(IceMsg& msg)
	 *	\param		msg		[out] the message
	 *	\return		true if success, else false if there was no pending message
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FUNCTION ICECORE_API bool			GetMsg(IceMsg& msg);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	A global function to flush all messages.
	 *	\relates	IceMsg
	 *	\fn			FlushMsgs(CustomArray* log)
	 *	\param		log		[out] the flushed messages
	 *	\return		the number of flushed messages
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FUNCTION ICECORE_API udword			FlushMsgs(CustomArray* log=null);

#endif // ICEMSG_H