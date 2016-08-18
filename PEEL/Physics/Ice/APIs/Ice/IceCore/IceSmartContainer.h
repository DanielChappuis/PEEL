///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a smart container.
 *	\file		IceSmartContainer.h
 *	\author		Pierre Terdiman
 *	\date		April, 13, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICESMARTCONTAINER_H
#define ICESMARTCONTAINER_H

	class ICECORE_API SmartContainer : public Cell
	{
									DECLARE_ICE_CLASS(SmartContainer, Cell);
		public:
		// Management

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Validates a given cell - checks it's appropriate for the container.
		 *	\param		object		[in] object to check
		 *	\return		true if the object can be stored in the container
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual			bool		Validate(Cell* object)			{ return true;						}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets sorting priority of a given object.
		 *	\param		object		[in] object to check
		 *	\return		object's sorting priority for this container
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual			udword		GetPriority(Cell* object)		{ return object->GetPriority();		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Adds a cell to the container.
		 *	- This is a O(1) method
		 *	- The cell is validated before beeing added (and only added if validation succeeds)
		 *	- The container is automatically resized if needed.
		 *	- A new reference to the cell is created. (owner is the container)
		 *	\param		cell	[in] the cell to store in the container
		 *	\see		Remove()
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool		Add(Cell* cell);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Removes a cell from the container.
		 *	- This is a O(n) method
		 *	- Reference to the cell is deleted.
		 *	\param		cell	[in] the cell to remove from the container
		 *	\see		Add()
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool		Remove(Cell* cell);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Removes all cells from the container.
		 *	This is a slow O(n*log n) method in theory, but since we always remove the first entry then
		 *	OnInvalidReference() returns in O(1), and the whole process actually is O(n)
		 *	\see		ForceCellsDestruction()
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool		Reset();

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Deletes all contained cells.
		 *	This is very different from Reset() since here we actually force contained cells to be deleted,
		 *	regardless of their current number of owners. (whereas Reset() only removes a single reference
		 *	to the cells, only leading to destruction if the container was the last owner)
		 *	Please note ICE is robust enough to allow that kind of savage destruction. It does not crash,
		 *	the kernel fixes everything as long as the relationships between owners & refs have been declared.
		 *	\param		context		[in] context parameter for the SelfDestruct() method
		 *	\param		user_data	[in] user_data parameter for the SelfDestruct() method
		 *	\see		Reset()
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool		ForceCellsDestruction(udword context, void* user_data);

		// Stats

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets the ram used by the container.
		 *	\return		the ram used in bytes.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						udword		GetUsedRam()		const;
		// Data access.

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Returns the current number of cells.
		 *	\see		GetCell()
		 *	\see		GetCells()
		 *	\return		current number of cells
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			udword		GetNbCells()		const	{ return mCurNbCells;	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Returns ith cell.
		 *	\param		i	[in] cell index
		 *	\return		ith cell pointer
		 *	\see		GetNbCells()
		 *	\see		GetCells()
		 *	\warning	no bounds checking here!
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			Cell*		GetCell(udword i)	const	{ return mCells[i];		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Returns the list of cells.
		 *	\return		list of contained cells
		 *	\see		GetCell()
		 *	\see		GetNbCells()
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			Cell**		GetCells()			const	{ return mCells;		}

		//! Access as an array
		inline_			Cell*&		operator[](udword i)const	{ ASSERT(i<mCurNbCells); return mCells[i];	}

		// Access sorted cells
		inline_			Cell*		GetSortedCell(udword i)
									{
										// Lazy-sorting
										if(!mIsSorted)	Sort();
										return GetCell(i);
									}
		inline_			Cell**		GetSortedCells()
									{
										// Lazy-sorting
										if(!mIsSorted)	Sort();
										return GetCells();
									}

//		inline_			udword		GetSignature()		const	{ return mSignature;	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Finds a contained cell, given its name.
		 *	\param		name	[in] wanted cell's name
		 *	\return		wanted cell, or null if not found
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						Cell*		FindCell(const char* name)	const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Finds a contained cell.
		 *	\param		cell	[in] wanted cell
		 *	\return		cell index, or INVALID_ID
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						udword		FindCellIndex(const Cell* cell)		const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	This method is called by the kernel after a referenced cell has been dereferenced or deleted.
		 *	\param		invalid_ref		[in] the now invalid referenced cell
		 *	\return		true if the method has been overriden
		 *	\warning	Only called if CF_KERNEL_INVALIDREF is enabled
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		overriden(Cell)	bool		OnInvalidReference(const Cell* invalid_ref);

		private:
		// Internal methods
			// Resizing
						bool		Resize();
			// Sorting
						bool		Sort();
		// Data
						udword		mMaxNbCells;		//!< Maximum possible number of cells
						udword		mCurNbCells;		//!< Current number of cells
						Cell**		mCells;				//!< List of cells
//						udword		mSignature;			//!< Timestamp
						bool		mIsSorted;			//!< True in sorted state
	};

#endif // ICESMARTCONTAINER_H
