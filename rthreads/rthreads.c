/* Copyright  (C) 2010-2016 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (rthreads.c).
 * ---------------------------------------------------------------------------------------
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifdef __unix__
#define _POSIX_C_SOURCE 199309
#endif

#include <stdlib.h>

#include <boolean.h>
#include <rthreads/rthreads.h>

/* with RETRO_WIN32_USE_PTHREADS, pthreads can be used even on win32. Maybe only supported in MSVC>=2005  */

#if defined(_WIN32) && !defined(RETRO_WIN32_USE_PTHREADS)
#define USE_WIN32_THREADS
#ifdef _XBOX
#include <xtl.h>
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#elif defined(GEKKO)
#include "gx_pthread.h"
#elif defined(PSP)
#include "psp_pthread.h"
#elif defined(__CELLOS_LV2__)
#include <pthread.h>
#include <sys/sys_time.h>
#else
#include <pthread.h>
#include <time.h>
#endif

#if defined(VITA)
#include <sys/time.h>
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

struct thread_data
{
   void (*func)(void*);
   void *userdata;
};

struct sthread
{
#ifdef USE_WIN32_THREADS
   HANDLE thread;
#else
   pthread_t id;
#endif
};

struct slock
{
#ifdef USE_WIN32_THREADS
   HANDLE lock;
#else
   pthread_mutex_t lock;
#endif
};

struct scond
{
#ifdef USE_WIN32_THREADS

   /* The syntax we'll use is mind-bending unless we use a struct. Plus, we might want to store more info later */
   /* This will be used as a linked list immplementing a queue of waiting threads */
   struct QueueEntry
   {
      struct QueueEntry* next;
   };

   /* With this implementation of scond, we don't have any way of waking (or even identifying) specific threads */
   /* But we need to wake them in the order indicated by the queue. */
   /* This potato token will get get passed around every waiter. The bearer can test whether he's next, and hold onto the potato if he is. */
   /* When he's done he can then put it back into play to progress the queue further */
   HANDLE hot_potato;

   /* The primary signalled event. Hot potatoes are passed until this is set. */
   HANDLE event; 

   /* the head of the queue; NULL if queue is empty */
   struct QueueEntry*  head; 

   /* equivalent to the queue length */
   int waiters; 

   /* how many waiters in the queue have been conceptually wakened by signals (even if we haven't managed to actually wake them yet */
   int wakens; 

#else
   pthread_cond_t cond;
#endif
};

#ifdef USE_WIN32_THREADS
static DWORD CALLBACK thread_wrap(void *data_)
#else
static void *thread_wrap(void *data_)
#endif
{
   struct thread_data *data = (struct thread_data*)data_;
   if (!data)
	   return 0;
   data->func(data->userdata);
   free(data);
   return 0;
}

/**
 * sthread_create:
 * @start_routine           : thread entry callback function
 * @userdata                : pointer to userdata that will be made
 *                            available in thread entry callback function
 *
 * Create a new thread.
 *
 * Returns: pointer to new thread if successful, otherwise NULL.
 */
sthread_t *sthread_create(void (*thread_func)(void*), void *userdata)
{
   bool thread_created      = false;
   struct thread_data *data = NULL;
   sthread_t *thread        = (sthread_t*)calloc(1, sizeof(*thread));

   if (!thread)
      return NULL;

   data                     = (struct thread_data*)calloc(1, sizeof(*data));
   if (!data)
      goto error;

   data->func = thread_func;
   data->userdata = userdata;

#ifdef USE_WIN32_THREADS
   thread->thread = CreateThread(NULL, 0, thread_wrap, data, 0, NULL);
   thread_created = !!thread->thread;
#else
#if defined(VITA)
   pthread_attr_t thread_attr;
   pthread_attr_init(&thread_attr);
   pthread_attr_setstacksize(&thread_attr , 0x10000 );
   thread_created = pthread_create(&thread->id, &thread_attr, thread_wrap, data) == 0;
#else
   thread_created = pthread_create(&thread->id, NULL, thread_wrap, data) == 0;
#endif
#endif

   if (!thread_created)
      goto error;

   return thread;

error:
   if (data)
      free(data);
   free(thread);
   return NULL;
}

/**
 * sthread_detach:
 * @thread                  : pointer to thread object
 *
 * Detach a thread. When a detached thread terminates, its
 * resource sare automatically released back to the system
 * without the need for another thread to join with the
 * terminated thread.
 *
 * Returns: 0 on success, otherwise it returns a non-zero error number.
 */
int sthread_detach(sthread_t *thread)
{
#ifdef USE_WIN32_THREADS
   CloseHandle(thread->thread);
   free(thread);
   return 0;
#else
   return pthread_detach(thread->id);
#endif
}

/**
 * sthread_join:
 * @thread                  : pointer to thread object
 *
 * Join with a terminated thread. Waits for the thread specified by
 * @thread to terminate. If that thread has already terminated, then
 * it will return immediately. The thread specified by @thread must
 * be joinable.
 *
 * Returns: 0 on success, otherwise it returns a non-zero error number.
 */
void sthread_join(sthread_t *thread)
{
#ifdef USE_WIN32_THREADS
   WaitForSingleObject(thread->thread, INFINITE);
   CloseHandle(thread->thread);
#else
   pthread_join(thread->id, NULL);
#endif
   free(thread);
}

/**
 * sthread_isself:
 * @thread                  : pointer to thread object
 *
 * Returns: true (1) if calling thread is the specified thread
 */
bool sthread_isself(sthread_t *thread)
{
  /* This thread can't possibly be a null thread */
  if (!thread) return false;

#ifdef USE_WIN32_THREADS
   return GetCurrentThread() == thread->thread;
#else
   return pthread_equal(pthread_self(),thread->id);
#endif
}

/**
 * slock_new:
 *
 * Create and initialize a new mutex. Must be manually
 * freed.
 *
 * Returns: pointer to a new mutex if successful, otherwise NULL.
 **/
slock_t *slock_new(void)
{
   bool mutex_created = false;
   slock_t      *lock = (slock_t*)calloc(1, sizeof(*lock));
   if (!lock)
      return NULL;

#ifdef USE_WIN32_THREADS
   lock->lock         = CreateMutex(NULL, FALSE, NULL);
   mutex_created      = !!lock->lock;
#else
   mutex_created      = (pthread_mutex_init(&lock->lock, NULL) == 0);
#endif

   if (!mutex_created)
      goto error;

   return lock;

error:
   free(lock);
   return NULL;
}

/**
 * slock_free:
 * @lock                    : pointer to mutex object
 *
 * Frees a mutex.
 **/
void slock_free(slock_t *lock)
{
   if (!lock)
      return;

#ifdef USE_WIN32_THREADS
   CloseHandle(lock->lock);
#else
   pthread_mutex_destroy(&lock->lock);
#endif
   free(lock);
}

/**
 * slock_lock:
 * @lock                    : pointer to mutex object
 *
 * Locks a mutex. If a mutex is already locked by
 * another thread, the calling thread shall block until
 * the mutex becomes available.
**/
void slock_lock(slock_t *lock)
{
   if (!lock)
      return;
#ifdef USE_WIN32_THREADS
   WaitForSingleObject(lock->lock, INFINITE);
#else
   pthread_mutex_lock(&lock->lock);
#endif
}

/**
 * slock_unlock:
 * @lock                    : pointer to mutex object
 *
 * Unlocks a mutex.
 **/
void slock_unlock(slock_t *lock)
{
   if (!lock)
      return;
#ifdef USE_WIN32_THREADS
   ReleaseMutex(lock->lock);
#else
   pthread_mutex_unlock(&lock->lock);
#endif
}

/**
 * scond_new:
 *
 * Creates and initializes a condition variable. Must
 * be manually freed.
 *
 * Returns: pointer to new condition variable on success,
 * otherwise NULL.
 **/
scond_t *scond_new(void)
{
   scond_t      *cond = (scond_t*)calloc(1, sizeof(*cond));

   if (!cond)
      return NULL;

#ifdef USE_WIN32_THREADS
   /* This is very complex because recreating condition variable semantics with win32 parts is not easy */
   /* The main problem is that a condition variable can be used to wake up a thread, but only if the thread is already waiting; */
   /* whereas a win32 event will 'wake up' a thread in advance (the event will be set in advance, so a 'waiter' wont even have to wait on it) */
   /* So at the very least, we need to do something clever. But there's bigger problems. */
   /* We don't even have a straightforward way in win32 to satisfy pthread_cond_wait's atomicity requirement. The bulk of this algorithm is solving that. */
   /* Note: We might could simplify this using vista+ condition variables, but we wanted an XP compatible solution. */
   cond->event = CreateEvent(NULL, FALSE, FALSE, NULL);
   if(!cond->event) goto error;
   cond->hot_potato = CreateEvent(NULL, FALSE, FALSE, NULL);
   if(!cond->hot_potato)
   {
      CloseHandle(cond->event);
      goto error;
   }
   cond->waiters = cond->wakens = 0;
   cond->head = NULL;

#else
   if(pthread_cond_init(&cond->cond, NULL) != 0)
      goto error;
#endif

   return cond;

error:
   free(cond);
   return NULL;
}

/**
 * scond_free:
 * @cond                    : pointer to condition variable object
 *
 * Frees a condition variable.
**/
void scond_free(scond_t *cond)
{
   if (!cond)
      return;

#ifdef USE_WIN32_THREADS
   CloseHandle(cond->event);
   CloseHandle(cond->hot_potato);
#else
   pthread_cond_destroy(&cond->cond);
#endif
   free(cond);
}

/**
 * scond_wait:
 * @cond                    : pointer to condition variable object
 * @lock                    : pointer to mutex object
 *
 * Block on a condition variable (i.e. wait on a condition).
 **/
void scond_wait(scond_t *cond, slock_t *lock)
{
#ifdef USE_WIN32_THREADS
   
   /* add ourselves to a queue of waiting threads */
   struct QueueEntry myentry;
   struct QueueEntry** ptr = &cond->head;
   while(*ptr) /* walk to the end of the linked list */
      ptr = &((*ptr)->next);
   *ptr = &myentry;
   myentry.next = NULL;

   cond->waiters++;

   /* now the conceptual lock release and condition block are supposed to be atomic. */
   /* we can't do that in windows, but we can simulate the effects by using the queue, by the following analysis: */
   /* What happens if they aren't atomic? */
   /* 1. a signaller can rush in and signal, expecting a waiter to get it; but the waiter wouldn't, because he isn't blocked yet */
   /* solution: win32 events make this easy. the event will sit there enabled */
   /* 2. a signaller can rush in and signal, and then turn right around and wait */
   /* solution: the signaller will get queued behind the waiter, who's enqueued before he releases the mutex */

   /* It's my turn if I'm the head of the queue. Check to see if it's my turn. */
   while (cond->head != &myentry)
   {
      /* As long as someone is even going to be able to wake up when they receive the potato, keep it going round */
      if (cond->wakens > 0)
         SetEvent(cond->hot_potato);

      /* Wait to catch the hot potato before checking for my turn again */
      SignalObjectAndWait(lock->lock, cond->hot_potato, INFINITE, FALSE);
      slock_lock(lock);
   }
	
   /* It's my turn now -- I hold the potato */
   SignalObjectAndWait(lock->lock, cond->event, INFINITE, FALSE);
   slock_lock(lock);

   /* Remove ourselves from the queue */
   cond->head = myentry.next;
   cond->waiters--;

   /* If any other wakenings are pending, go ahead and set it up  */
   /* There may actually be no waiters. That's OK. The first waiter will come in, find it's his turn, and immediately get the signaled event */
   cond->wakens--;
   if(cond->wakens>0)
   {
      SetEvent(cond->event);

      /* Progress the queue: Put the hot potato back into play. It'll be tossed around until next in line gets it */
      SetEvent(cond->hot_potato); 
   }

#else
   pthread_cond_wait(&cond->cond, &lock->lock);
#endif
}

/**
 * scond_broadcast:
 * @cond                    : pointer to condition variable object
 *
 * Broadcast a condition. Unblocks all threads currently blocked
 * on the specified condition variable @cond.
 **/
int scond_broadcast(scond_t *cond)
{
#ifdef USE_WIN32_THREADS
   
   /* remember: we currently have mutex */
   if(cond->waiters == 0) return 0;
   
   /* awaken everything which is currently queued up */
   if(cond->wakens == 0) SetEvent(cond->event);
   cond->wakens = cond->waiters;

   /* Since there is now at least one pending waken, the potato must be in play */
   SetEvent(cond->hot_potato); 

   return 0;
#else
   return pthread_cond_broadcast(&cond->cond);
#endif
}

/**
 * scond_signal:
 * @cond                    : pointer to condition variable object
 *
 * Signal a condition. Unblocks at least one of the threads currently blocked
 * on the specified condition variable @cond.
 **/
void scond_signal(scond_t *cond)
{
#ifdef USE_WIN32_THREADS

   /* remember: we currently have mutex */
   if(cond->waiters == 0) return;
 
   /* wake up the next thing in the queue */
   if(cond->wakens == 0) SetEvent(cond->event);
   cond->wakens++;

   /* Since there is now at least one pending waken, the potato must be in play */
   SetEvent(cond->hot_potato); 

#else
   pthread_cond_signal(&cond->cond);
#endif
}

/**
 * scond_wait_timeout:
 * @cond                    : pointer to condition variable object
 * @lock                    : pointer to mutex object
 * @timeout_us              : timeout (in microseconds)
 *
 * Try to block on a condition variable (i.e. wait on a condition) until
 * @timeout_us elapses.
 *
 * Returns: false (0) if timeout elapses before condition variable is
 * signaled or broadcast, otherwise true (1).
 **/
bool scond_wait_timeout(scond_t *cond, slock_t *lock, int64_t timeout_us)
{
#ifdef USE_WIN32_THREADS
   DWORD ret;

   /* TODO: this is woefully inadequate. It needs to be solved with the newer approach used above */
   WaitForSingleObject(cond->event, 0);
   ret = SignalObjectAndWait(lock->lock, cond->event,
         (DWORD)(timeout_us) / 1000, FALSE);

   slock_lock(lock);
   return ret == WAIT_OBJECT_0;
#else
   int ret;
   int64_t seconds, remainder;
   struct timespec now = {0};

#ifdef __MACH__
   /* OSX doesn't have clock_gettime. */
   clock_serv_t cclock;
   mach_timespec_t mts;

   host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
   clock_get_time(cclock, &mts);
   mach_port_deallocate(mach_task_self(), cclock);
   now.tv_sec = mts.tv_sec;
   now.tv_nsec = mts.tv_nsec;
#elif defined(__CELLOS_LV2__)
   sys_time_sec_t s;
   sys_time_nsec_t n;

   sys_time_get_current_time(&s, &n);
   now.tv_sec  = s;
   now.tv_nsec = n;
#elif defined(__mips__) || defined(VITA)
   struct timeval tm;

   gettimeofday(&tm, NULL);
   now.tv_sec = tm.tv_sec;
   now.tv_nsec = tm.tv_usec * 1000;
#elif defined(RETRO_WIN32_USE_PTHREADS)
   _ftime64_s(&now);
#elif !defined(GEKKO)
   /* timeout on libogc is duration, not end time. */
   clock_gettime(CLOCK_REALTIME, &now);
#endif

   seconds      = timeout_us / INT64_C(1000000);
   remainder    = timeout_us % INT64_C(1000000);

   now.tv_sec  += seconds;
   now.tv_nsec += remainder * INT64_C(1000);

   ret = pthread_cond_timedwait(&cond->cond, &lock->lock, &now);
   return (ret == 0);
#endif
}

#ifdef HAVE_THREAD_STORAGE
bool sthread_tls_create(sthread_tls_t *tls)
{
#ifdef USE_WIN32_THREADS
   return (*tls = TlsAlloc()) != TLS_OUT_OF_INDEXES;
#else
   return pthread_key_create((pthread_key_t*)tls, NULL) == 0;
#endif
}

bool sthread_tls_delete(sthread_tls_t *tls)
{
#ifdef USE_WIN32_THREADS
   return TlsFree(*tls) != 0;
#else
   return pthread_key_delete(*tls) == 0;
#endif
}

void *sthread_tls_get(sthread_tls_t *tls)
{
#ifdef USE_WIN32_THREADS
   return TlsGetValue(*tls);
#else
   return pthread_getspecific(*tls);
#endif
}

bool sthread_tls_set(sthread_tls_t *tls, const void *data)
{
#ifdef USE_WIN32_THREADS
   return TlsSetValue(*tls, (void*)data) != 0;
#else
   return pthread_setspecific(*tls, data) == 0;
#endif
}
#endif
