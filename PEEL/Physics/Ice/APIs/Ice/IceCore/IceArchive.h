///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEARCHIVE_H
#define ICEARCHIVE_H

	struct ICECORE_API ArchiveHandle
	{
		inline_					ArchiveHandle()	: mArchive(null), mIndex(INVALID_ID)	{}

		inline_	bool			IsValid()			const	{ return mArchive!=null;	}
		inline_	void			Invalidate()				{ mArchive=null;			}

				void*			mArchive;
				udword			mIndex;
	};

	struct ICECORE_API VirtualFileHandle
	{
		inline_	bool			IsInArchive()	const	{ return mHandle.IsValid();	}

				ArchiveHandle	mHandle;
				String			mFilename;
	};

	FUNCTION ICECORE_API bool RegisterArchive(const char* filename);
	FUNCTION ICECORE_API bool ReleaseArchives();
	FUNCTION ICECORE_API bool IsInArchives(const char* filename, ArchiveHandle* handle=null);

	class File;

	class ICECORE_API VirtualFile : public ReadStream
	{
		public:
											VirtualFile(const VirtualFileHandle& handle);
											VirtualFile(const char* filename);
		virtual								~VirtualFile();

		inline_					bool		IsValid()	const	{ return mBuffer!=null || mFile!=null;	}
		inline_					udword		GetSize()	const	{ return mSize;							}

		override2(ReadStream)	bool		Seek(udword offset)							const;

		// Loading API
		override2(ReadStream)	ubyte		ReadByte()									const;
		override2(ReadStream)	uword		ReadWord()									const;
		override2(ReadStream)	udword		ReadDword()									const;
		override2(ReadStream)	float		ReadFloat()									const;
		override2(ReadStream)	double		ReadDouble()								const;
		override2(ReadStream)	bool		ReadBuffer(void* buffer, udword size)		const;

								const char*	ReadString()								const;
		//
								bool		GetLine(char* buffer, udword buffer_size, udword* length=null)	const;
								ubyte*		Load(udword& length)						const;

		inline_			const	IceFile*	GetFile()			const	{ return mFile;	}

		private:
								udword		mSize;
								ubyte*		mBuffer;
		mutable					ubyte*		mCurrentAddress;

								IceFile*	mFile;
		// Internal methods
								bool		ReadFromZip(const char* filename);
	};

#endif // ICEARCHIVE_H
