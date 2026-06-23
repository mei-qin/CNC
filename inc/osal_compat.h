/* osal_compat.h — Override SOEM v2.x Linux osal thread macros
 * 
 * SOEM v2.x defines OSAL_THREAD_FUNC as 'void' (no return value).
 * This project uses pthread conventions (void* return), so we override.
 * 
 * Include this AFTER any SOEM header in every .h file that
 * declares thread functions with OSAL_THREAD_FUNC / OSAL_THREAD_FUNC_RT.
 */
#ifdef OSAL_THREAD_FUNC
#undef OSAL_THREAD_FUNC
#endif
#define OSAL_THREAD_FUNC void *

#ifdef OSAL_THREAD_FUNC_RT
#undef OSAL_THREAD_FUNC_RT
#endif
#define OSAL_THREAD_FUNC_RT void *
