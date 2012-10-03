//
//
//
////*****************************************************************************
////
////! \addtogroup os_api
////! @{
////
////*****************************************************************************
//
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//#include "os.h"
//
//
///**
// * \brief allocate memory
// *
// * Function allocates size bytes of uninitialized memory, and returns a pointer to the allocated memory.\n
// * The allocated space is suitably aligned (after possible pointer coercion) for storage of any type of object.
// *
// * \param[in] size    number of memory bytes to allocate
// *
// * \return   On success return a pointer to the allocated space, on failure return null
// * \sa OS_free
// * \note
// *
// * \warning
// */
//
//void * OS_malloc( unsigned long size )
//{
//    return malloc( size );
//}
//
///**
// * \brief free allocated memory
// *
// * Function causes the allocated memory referenced by ptr to
// *  be made available for future allocations.\n If ptr is
// * NULL, no action occurs.
// *
// *  \param[in] ptr   pointer to previously allocated memory
// *
// * \return   None
// *
// * \sa OS_malloc
// * \note
// *
// * \warning
// */
//
//void   OS_free( void * ptr )
//{
//    free( ptr );
//}
////*****************************************************************************
////
//// Close the Doxygen group.
////! @}
////
////*****************************************************************************
