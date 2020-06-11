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

		override2(ReadStream)	bool			Seek(udword offset)						const;

		// Loading API
		override2(ReadStream)	ubyte			ReadByte()								const;
		override2(ReadStream)	uword			ReadWord()								const;
		override2(ReadStream)	udword			ReadDword()								const;
		override2(ReadStream)	float			ReadFloat()								const;
		override2(ReadStream)	double			ReadDouble()							const;
		override2(ReadStream)	bool			ReadBuffer(void* buffer, udword size)	const;

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
		override2(WriteStream)	WriteStream&	StoreByte(ubyte b);
		override2(WriteStream)	WriteStream&	StoreWord(uword w);
		override2(WriteStream)	WriteStream&	StoreDword(udword d);
		override2(WriteStream)	WriteStream&	StoreFloat(float f);
		override2(WriteStream)	WriteStream&	StoreDouble(double f);
		override2(WriteStream)	WriteStream&	StoreBuffer(const void* buffer, udword size);

		private:
								FILE*			mFp;
	};

#endif	// ICEFILESTREAM_H
