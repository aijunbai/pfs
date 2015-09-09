/***************************************************************************
 *   xmlIO.cpp   - description
 *
 *   This program is part of the Etiseo project.
 *
 *   See http://www.etiseo.net  http://www.silogic.fr   
 *
 *   (C) Silogic - Etiseo Consortium
 ***************************************************************************/

#include "xmlUtil.h"


ThreadCondition::ThreadCondition()
{
  pthread_mutex_init(&mMutex, 0);
  pthread_cond_init(&mCond, 0);
}

ThreadCondition::~ThreadCondition()
{
  pthread_mutex_destroy(&mMutex);
  pthread_cond_destroy(&mCond);
}

bool ThreadCondition::Wait(int ms)
{
  while (pthread_mutex_lock(&mMutex))
  {
  }

  int ret = ETIMEDOUT;
  if (ms > 0)
  {
    struct timeval now;
    struct timespec timeout;

    gettimeofday(&now, 0);
    timeout.tv_sec = now.tv_sec + ms / 1000;
    timeout.tv_nsec = (now.tv_usec + ms % 1000 * 1000) * 1000;

    while((ret = pthread_cond_timedwait(&mCond, &mMutex, &timeout)) == EINTR )
    {
    }
  }
  else
  {
    ret = pthread_cond_wait(&mCond, &mMutex);
  }

  while (pthread_mutex_unlock(&mMutex))
  {
  }
  return (ret == ETIMEDOUT);
}

void ThreadCondition::Set()
{
  while(pthread_mutex_lock(&mMutex))
  {
  }
  while(pthread_cond_signal(&mCond))
  {
  }
  while(pthread_mutex_unlock(&mMutex))
  {
  }
}

ThreadMutex::ThreadMutex()
{
  pthread_mutex_init (&mMutex,0);
}

ThreadMutex::~ThreadMutex() {
  pthread_mutex_destroy(&mMutex);
}

void ThreadMutex::Lock()
{
  while (pthread_mutex_lock(&mMutex))
  {
  }
}

void ThreadMutex::UnLock()
{
  while (pthread_mutex_unlock(&mMutex))
  {
  }
}

void *SimpleThread::Spawner(void *thread)
{
  static_cast<SimpleThread*>(thread)->StartRoutine();
  return (void*) 0;
}

void SimpleThread::Start()
{
  pthread_create(&mThread, 0, &Spawner, this);
}

void SimpleThread::Join()
{
  pthread_join(mThread, 0);
}


using namespace Etiseo;

void UtilXml::Init()
{
  LIBXML_TEST_VERSION
  xmlKeepBlanksDefault(0);
  xmlSetExternalEntityLoader(xmlNoNetExternalEntityLoader);
}

void UtilXml::Cleanup()
{
  xmlCleanupParser();

}

/******************************************************************************************
 *
 * Output to ostream
 *
 ******************************************************************************************/

int UtilXml::WriteCallback(void * context, const char * buffer, int length)
{
  XmlOutputHandler* handler = (XmlOutputHandler*)context;
  return handler->write(buffer,length);
}

int UtilXml::OutputCloseCallback(void * obj)
{
  XmlOutputHandler* handler = (XmlOutputHandler*)obj;
  return handler->close();
}

XmlOutputHandler::XmlOutputHandler(std::ostream& stream): mStream(stream)
{
  mOutBuffer = xmlOutputBufferCreateIO(UtilXml::WriteCallback,
                                       UtilXml::OutputCloseCallback,
                                       this, NULL);
}

XmlOutputHandler::~XmlOutputHandler()
{
  close();
  mOutBuffer = 0;
}

int XmlOutputHandler::write(const char* buffer, int length)
{
  mStream.write(buffer,length);
  return length;
}

int XmlOutputHandler::close()
{
  mStream.flush();
  return 0;
}

/******************************************************************************************
 *
 * Input from istream
 *
 ******************************************************************************************/

int UtilXml::ReadCallback(void * context, char * buffer, int length)
{
  std::istream * is = (std::istream *)context;

  is->read(buffer,length);
  int size= is->gcount();
  if (size < length)
    buffer[size] = '\0';

  return size;
}

int UtilXml::InputCloseCallback(void * obj)
{
  XmlInputHandler* handler = (XmlInputHandler*)obj;
  return handler->close();
}


XmlInputHandler::XmlInputHandler(std::istream& stream) : mStream(stream)
{
  mInBuffer = xmlParserInputBufferCreateIO(UtilXml::ReadCallback,
                                           UtilXml::InputCloseCallback,
                                           this, XML_CHAR_ENCODING_NONE);
}

XmlInputHandler::~XmlInputHandler()
{
  close();
  mInBuffer = 0;
}

int XmlInputHandler::read(char* buffer, int length)
{
  mStream.read(buffer,length);
  return length;
}

int XmlInputHandler::close()
{
  return 0;
}



