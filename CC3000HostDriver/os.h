//
//
//#ifndef __OS_H__
//#define __OS_H__
//
////*****************************************************************************
////
//// If building with a C++ compiler, make all of the definitions in this header
//// have a C binding.
////
////*****************************************************************************
//#ifdef  __cplusplus
//extern "C" {
//#endif
//
////*****************************************************************************
//
////*****************************************************************************
////
//// Prototypes for the APIs.
////
////*****************************************************************************
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
//extern void * OS_malloc( unsigned long size );
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
//extern void   OS_free( void * ptr );
////*****************************************************************************
////
//// Mark the end of the C bindings section for C++ compilers.
////
////*****************************************************************************
//#ifdef  __cplusplus
//}
//#endif // __cplusplus
//
//#endif // __OS_H__
//
//
