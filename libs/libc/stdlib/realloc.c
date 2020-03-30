#include "Ql_stdlib.h"
#include "Ql_memory.h"

void* Ql_realloc(void* ptr, u32 size)
{
	void* new_data = NULL;

	if(size)
	{
		if(!ptr)
		{
			return Ql_MEM_Alloc(size);
		}

		new_data = Ql_MEM_Alloc(size);
		if(new_data)
		{
			Ql_memcpy(new_data, ptr, size); // TODO: unsafe copy...
			Ql_MEM_Free(ptr); // we always move the data. free.
		}
	}

	return new_data;
}

void* Ql_reallocf(void* ptr, u32 size)
{
	void* p = Ql_realloc(ptr, size);

	if((p == NULL) && (ptr != NULL))
	{
		Ql_MEM_Free(ptr);
	}

	return p;
}

