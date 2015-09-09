/***************************************************************************
 *   xmlIO.h     - description
 *
 *   This program is part of the ETISEO project.
 *
 *   See http://www.etiseo.net  http://www.silogic.fr    
 *
 *   (C) Silogic - ETISEO Consortium
 ***************************************************************************/


#ifndef _ETI_XML_IO_H_
#define _ETI_XML_IO_H_

#include <ostream>
#include <istream>
#include <libxml/xmlIO.h>
#include <libxml/xmlwriter.h>

#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include "params.h"

inline void WaitFor(int ms)
{
  usleep(ms * 1000);
}

class ThreadCondition
{
public:
    ThreadCondition();
    ~ThreadCondition();

    bool Wait(int ms = 0);
    void Set();

private:
    pthread_cond_t mCond;
    pthread_mutex_t mMutex;
};

class ThreadMutex
{
public:
    ThreadMutex();
    ~ThreadMutex();

    void Lock();
    void UnLock();

private:
    pthread_mutex_t mMutex;
};

#define SCOPED_LOCK_ \
    static ThreadMutex lock; \
    ScopedMutex mutex(lock);

class ScopedMutex {
 public:
  explicit ScopedMutex(ThreadMutex &mutex): mutex_(mutex) {
    if (Params::ins().threads > 1) {
      mutex_.Lock();
    }
  }

  ~ScopedMutex() {
    if (Params::ins().threads > 1) {
      mutex_.UnLock();
    }
  }

 private:
  // Disallow default constructors and copy constructors.
  ScopedMutex();
  ScopedMutex(const ScopedMutex&);
  void operator=(const ScopedMutex&);

  ThreadMutex& mutex_;
};

class SimpleThread
{
  SimpleThread(const SimpleThread &);
  const SimpleThread &operator=(const SimpleThread &);

public:
  SimpleThread(): mThread(0) {
  }

  virtual ~SimpleThread() {

  }

  void Start();
  void Join();

private:
  static void *Spawner(void *v);
  virtual void StartRoutine() = 0;

private:
  pthread_t mThread;
};

namespace Etiseo {

	//!  A function to handle libxml2 initialization
	class UtilXml
	{
		public:
		
			static void	Init();
			static void	Cleanup();
			
			static int ReadCallback(void * context, char * buffer, int length);
			static int InputCloseCallback(void * context);

			static int WriteCallback(void * context, const char * buffer, int length);
			static int OutputCloseCallback(void * context);
	};

	//!  mapping output from libxml2 to ostream
	class XmlOutputHandler 
	{
		public:
			
			XmlOutputHandler(std::ostream& stream);
			virtual ~XmlOutputHandler();
			
		int	write(const char* buffer, int length);
		int close(); 
		
		inline const struct _xmlOutputBuffer* xmlOutputBuffer() const 
			{ return mOutBuffer; }
		inline struct _xmlOutputBuffer* xmlOutputBuffer() { return mOutBuffer; }
		
		private:
			std::ostream					&mStream;
			struct _xmlOutputBuffer			*mOutBuffer; 
			
			
	};

	//!  mapping intput from libxml2 from istream
	class XmlInputHandler 
	{
		public:
			
			XmlInputHandler(std::istream& stream);
			virtual ~XmlInputHandler();
			
		int	read(char* buffer, int length);
		int close(); 
		
		inline const struct _xmlParserInputBuffer* xmlInputBuffer() const 
			{ return mInBuffer; }
		inline struct _xmlParserInputBuffer* xmlInputBuffer() { return mInBuffer; }
		
		private:
			std::istream					&mStream;
			struct _xmlParserInputBuffer	*mInBuffer; 
			
			
	};
	


};

#endif
