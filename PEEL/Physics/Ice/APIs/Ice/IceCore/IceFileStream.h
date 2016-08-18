///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for file streams.
 *	\file		IceFileStream.h
 *	\author		Pierre Terdiman
 *	\date		September, 13, 2004
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEFILESTREAM_H
#define ICEFILESTREAM_H

	class ICECORE_API FileReadStream : public ReadStream
	{
		public:
												FileReadStream(const char* filename);
		virtual									~FileReadStream();

		inline_					bool			IsValid()								const	{ return mFp!=null;	}

		overriden(ReadStream)	bool			Seek(udword offset)						const;

		// Loading API
		overriden(ReadStream)	ubyte			ReadByte()								const;
		overriden(ReadStream)	uword			ReadWord()								const;
		overriden(ReadStream)	udword			ReadDword()								const;
		overriden(ReadStream)	float			ReadFloat()								const;
		overriden(ReadStream)	double			ReadDouble()							const;
		overriden(ReadStream)	bool			ReadBuffer(void* buffer, udword size)	const;

		private:
								FILE*			mFp;
	};

	class ICECORE_API FileWriteStream : public WriteStream
	{
		public:
												FileWriteStream(const char* filename);
		virtual									~FileWriteStream();

		inline_					bool			IsValid()								const	{ return mFp!=null;	}

		// Saving API
		overriden(WriteStream)	WriteStream&	StoreByte(ubyte b);
		overriden(WriteStream)	WriteStream&	StoreWord(uword w);
		overriden(WriteStream)	WriteStream&	StoreDword(udword d);
		overriden(WriteStream)	WriteStream&	StoreFloat(float f);
		overriden(WriteStream)	WriteStream&	StoreDouble(double f);
		overriden(WriteStream)	WriteStream&	StoreBuffer(const void* buffer, udword size);

		private:
								FILE*			mFp;
	};

#endif	// ICEFILESTREAM_H
